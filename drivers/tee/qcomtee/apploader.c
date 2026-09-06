// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loading Trusted Applications into QTEE.
 *
 * QTEE will not run an application until someone hands it the signed image,
 * and on Android that someone is a userspace daemon.  Nothing on a mainline
 * system does it, so a driver that wants to talk to an application has to ask
 * for it first.  QTEE offers the AppLoader service for exactly that.
 */

#include <linux/debugfs.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/tee_core.h>
#include <linux/tee_drv.h>
#include <linux/mutex.h>
#include <linux/sizes.h>
#include <linux/vmalloc.h>

#include "qcomtee.h"

/* The AppLoader service, and the two ways it takes an image. */
#define QCOMTEE_APP_LOADER_UID			3
#define QCOMTEE_APP_LOADER_OP_LOAD_FROM_BUFFER	0
#define QCOMTEE_APP_LOADER_OP_LOAD_FROM_REGION	1

/* The controller QTEE hands back for a loaded application. */
#define QCOMTEE_APP_CONTROLLER_OP_UNLOAD	1
#define QCOMTEE_APP_CONTROLLER_OP_GET_APP	2

/*
 * An application that predates the object interface keeps its old shape: one
 * method taking a request buffer and filling in a response buffer.
 */
#define QCOMTEE_APP_OP_SEND_REQUEST		0
#define QCOMTEE_APP_MAX_RESPONSE		SZ_4K

#define QCOMTEE_TA_FW_PATH "qcom/sm8750/xiaomi/piano"

/**
 * qcomtee_ta_image_size() - Work out how big the assembled image is.
 * @mdt: the metadata file.
 *
 * The signer splits an application into one file per ELF segment, keeping the
 * ELF header, the program headers and the hash segment together in a separate
 * metadata file.  Reassembling means putting the metadata at offset zero and
 * each segment at the file offset its program header gives, so the image is as
 * long as the furthest segment reaches.
 *
 * Return: Size of the assembled image, or < 0 if the metadata is unusable.
 */
static ssize_t qcomtee_ta_image_size(const struct firmware *mdt)
{
	const struct elf64_hdr *ehdr = (const struct elf64_hdr *)mdt->data;
	const struct elf64_phdr *phdrs;
	ssize_t size = mdt->size;
	unsigned int i;

	if (mdt->size < sizeof(*ehdr) || memcmp(ehdr->e_ident, ELFMAG, SELFMAG))
		return -EINVAL;

	if (ehdr->e_ident[EI_CLASS] != ELFCLASS64 ||
	    ehdr->e_phentsize != sizeof(*phdrs))
		return -EINVAL;

	if (ehdr->e_phoff > mdt->size ||
	    array_size(ehdr->e_phnum, sizeof(*phdrs)) > mdt->size - ehdr->e_phoff)
		return -EINVAL;

	phdrs = (const struct elf64_phdr *)(mdt->data + ehdr->e_phoff);

	for (i = 0; i < ehdr->e_phnum; i++) {
		ssize_t end;

		if (check_add_overflow((ssize_t)phdrs[i].p_offset,
				       (ssize_t)phdrs[i].p_filesz, &end))
			return -EINVAL;

		size = max(size, end);
	}

	return size;
}

static int qcomtee_ta_assemble(struct device *dev, const char *name,
			       const struct firmware *mdt, void *img)
{
	const struct elf64_hdr *ehdr = (const struct elf64_hdr *)mdt->data;
	const struct elf64_phdr *phdrs;
	unsigned int i;
	int ret;

	phdrs = (const struct elf64_phdr *)(mdt->data + ehdr->e_phoff);
	memcpy(img, mdt->data, mdt->size);

	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct firmware *seg;
		char *fname;

		if (!phdrs[i].p_filesz)
			continue;

		fname = kasprintf(GFP_KERNEL, "%s/%s.b%02u", QCOMTEE_TA_FW_PATH,
				  name, i);
		if (!fname)
			return -ENOMEM;

		ret = request_firmware(&seg, fname, dev);
		if (ret) {
			dev_err(dev, "%s is missing\n", fname);
			kfree(fname);
			return ret;
		}

		if (seg->size != phdrs[i].p_filesz) {
			dev_err(dev, "%s is %zu bytes, header says %llu\n",
				fname, seg->size, phdrs[i].p_filesz);
			release_firmware(seg);
			kfree(fname);
			return -EINVAL;
		}

		memcpy(img + phdrs[i].p_offset, seg->data, seg->size);
		release_firmware(seg);
		kfree(fname);
	}

	return 0;
}

static int qcomtee_ctx_match(struct tee_ioctl_version_data *ver,
			     const void *data)
{
	return ver->impl_id == TEE_IMPL_ID_QTEE;
}

/* Which method of the application the next request goes to. */
static u32 qcomtee_ta_op;

/*
 * Which QTEE service loads the application, and how.  QTEE offers more than
 * one: the plain application loader, and a compatibility loader for
 * applications written against the older interface.
 */
static u32 qcomtee_ta_loader_uid = QCOMTEE_APP_LOADER_UID;
static u32 qcomtee_ta_loader_op = QCOMTEE_APP_LOADER_OP_LOAD_FROM_REGION;

/*
 * Whether the image goes in as a buffer or as a region of shared memory.
 * Anything but a small application has to go in as a region: the message
 * buffer an invocation can carry is bounded.
 */
static u32 qcomtee_ta_load_by_region = 1;

/* What to tell the application when opening a session with it. */
static u32 qcomtee_ta_connection_method;
static u32 qcomtee_ta_connection_data;
static u32 qcomtee_ta_param_types;

/* How much room to give each session parameter. */
static u32 qcomtee_ta_param_size = 64;

/*
 * How much shared memory to offer the application when opening a session.
 * Some applications refuse to open one without any: "No TA Memory object,
 * memory objects not supported".
 */
static u32 qcomtee_ta_session_mem = SZ_64K;

/* The application currently loaded, if any. */
static struct {
	struct mutex lock;
	struct tee_context *ctx;
	struct qcomtee_object *controller;
	struct qcomtee_object *app;
	char name[32];
	void *response;
	size_t response_len;
} qcomtee_ta = { .lock = __MUTEX_INITIALIZER(qcomtee_ta.lock) };

static void qcomtee_unload_ta_locked(void)
{
	struct qcomtee_object_invoke_ctx *oic;
	int result;

	if (!qcomtee_ta.ctx)
		return;

	oic = qcomtee_object_invoke_ctx_alloc(qcomtee_ta.ctx);
	if (oic) {
		struct qcomtee_arg u[1] = { 0 };

		qcomtee_object_do_invoke(oic, qcomtee_ta.controller,
					 QCOMTEE_APP_CONTROLLER_OP_UNLOAD, u,
					 &result);
		kfree(oic);
	}

	qcomtee_object_put(qcomtee_ta.app);
	qcomtee_object_put(qcomtee_ta.controller);
	tee_client_close_context(qcomtee_ta.ctx);

	qcomtee_ta.app = NULL_QCOMTEE_OBJECT;
	qcomtee_ta.controller = NULL_QCOMTEE_OBJECT;
	qcomtee_ta.ctx = NULL;
	kfree(qcomtee_ta.response);
	qcomtee_ta.response = NULL;
	qcomtee_ta.response_len = 0;
}

/**
 * qcomtee_load_ta() - Ask QTEE to run a Trusted Application.
 * @name: name of the application, which is also the name of its files.
 *
 * Return: Zero if QTEE took the application, < 0 if it did not.
 */
static int qcomtee_load_ta(const char *name)
{
	struct qcomtee_object *client_env, *app_loader, *app;
	struct qcomtee_arg u[3] = { 0 };
	const struct firmware *mdt;
	struct tee_context *ctx;
	struct device *dev;
	struct qcomtee_object *region = NULL_QCOMTEE_OBJECT;
	struct tee_shm *shm = NULL;
	ssize_t img_size;
	void *img = NULL;
	char *fname;
	int result;
	int ret;

	ctx = tee_client_open_context(NULL, qcomtee_ctx_match, NULL, NULL);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	dev = &ctx->teedev->dev;

	struct qcomtee_object_invoke_ctx *oic __free(kfree) =
		qcomtee_object_invoke_ctx_alloc(ctx);
	if (!oic) {
		ret = -ENOMEM;
		goto out_put_ctx;
	}

	client_env = qcomtee_object_get_client_env(oic);
	if (client_env == NULL_QCOMTEE_OBJECT) {
		dev_err(dev, "QTEE would not give us a client environment\n");
		ret = -ENODEV;
		goto out_put_ctx;
	}

	app_loader = qcomtee_object_get_service(oic, client_env,
					        qcomtee_ta_loader_uid);
	if (app_loader == NULL_QCOMTEE_OBJECT) {
		dev_err(dev, "QTEE has no service %u\n", qcomtee_ta_loader_uid);
		ret = -ENODEV;
		goto out_put_env;
	}

	dev_info(dev, "loader service %u is %s\n", qcomtee_ta_loader_uid,
		 qcomtee_object_name(app_loader));

	fname = kasprintf(GFP_KERNEL, "%s/%s.mdt", QCOMTEE_TA_FW_PATH, name);
	if (!fname) {
		ret = -ENOMEM;
		goto out_put_loader;
	}

	ret = request_firmware(&mdt, fname, dev);
	kfree(fname);
	if (ret) {
		dev_err(dev, "cannot read the %s metadata: %d\n", name, ret);
		goto out_put_loader;
	}

	img_size = qcomtee_ta_image_size(mdt);
	if (img_size < 0) {
		dev_err(dev, "%s has an unusable metadata segment\n", name);
		ret = img_size;
		goto out_release_mdt;
	}

	if (qcomtee_ta_load_by_region) {
		shm = tee_shm_alloc_kernel_buf(ctx, img_size);
		if (IS_ERR(shm)) {
			dev_err(dev, "cannot share %zd bytes with QTEE: %ld\n",
				img_size, PTR_ERR(shm));
			ret = PTR_ERR(shm);
			goto out_release_mdt;
		}

		img = tee_shm_get_va(shm, 0);
	} else {
		img = vzalloc(img_size);
		if (!img) {
			ret = -ENOMEM;
			goto out_release_mdt;
		}
	}

	ret = qcomtee_ta_assemble(dev, name, mdt, img);
	if (ret)
		goto out_free_img;

	if (qcomtee_ta_load_by_region) {
		ret = qcomtee_memobj_from_shm(&region, shm);
		if (ret)
			goto out_free_img;

		u[0].type = QCOMTEE_ARG_TYPE_IO;
		u[0].o = region;
		u[1].type = QCOMTEE_ARG_TYPE_OO;
	} else {
		u[0].type = QCOMTEE_ARG_TYPE_IB;
		u[0].b.addr = img;
		u[0].b.size = img_size;
		u[1].type = QCOMTEE_ARG_TYPE_OO;
	}

	dev_info(dev, "handing QTEE %zd bytes of %s as %s, service %u op %u\n",
		 img_size, name,
		 qcomtee_ta_load_by_region ? "a region" : "a buffer",
		 qcomtee_ta_loader_uid, qcomtee_ta_loader_op);

	ret = qcomtee_object_do_invoke(oic, app_loader, qcomtee_ta_loader_op,
				       u, &result);
	if (ret) {
		dev_err(dev, "the invocation for %s failed: %d\n", name, ret);
		goto out_free_img;
	}

	if (result) {
		dev_err(dev, "QTEE refused %s: %d\n", name, result);
		ret = -EIO;
		goto out_free_img;
	}

	app = u[1].o;
	dev_info(dev, "%s is running, held by object %s\n", name,
		 qcomtee_object_name(app));

	/* Ask the controller for the application itself. */
	memset(u, 0, sizeof(u));
	u[0].type = QCOMTEE_ARG_TYPE_OO;

	ret = qcomtee_object_do_invoke(oic, app,
				       QCOMTEE_APP_CONTROLLER_OP_GET_APP, u,
				       &result);
	if (ret) {
		qcomtee_object_put(app);
		goto out_free_img;
	}

	qcomtee_ta.ctx = ctx;
	qcomtee_ta.controller = app;

	if (result) {
		/*
		 * An application that predates the object interface has no
		 * separate object of its own; the controller is what answers.
		 */
		dev_info(dev, "%s has no app object (%d), talking to the controller\n",
			 name, result);
		qcomtee_ta.app = app;
	} else {
		qcomtee_ta.app = u[0].o;
	}
	strscpy(qcomtee_ta.name, name, sizeof(qcomtee_ta.name));

	dev_info(dev, "%s answers on object %s\n", name,
		 qcomtee_object_name(qcomtee_ta.app));

	/* The context has to outlive this call now. */
	ctx = NULL;

out_free_img:
	if (shm)
		tee_shm_free(shm);
	else
		vfree(img);
out_release_mdt:
	release_firmware(mdt);
out_put_loader:
	qcomtee_object_put(app_loader);
out_put_env:
	qcomtee_object_put(client_env);
out_put_ctx:
	if (ctx)
		tee_client_close_context(ctx);

	return ret;
}

static ssize_t qcomtee_load_ta_write(struct file *file,
				     const char __user *ubuf, size_t len,
				     loff_t *ppos)
{
	char name[32];
	int ret;

	if (len == 0 || len >= sizeof(name))
		return -EINVAL;

	if (copy_from_user(name, ubuf, len))
		return -EFAULT;

	name[len] = '\0';
	strim(name);

	guard(mutex)(&qcomtee_ta.lock);

	qcomtee_unload_ta_locked();

	if (!strcmp(name, "none"))
		return len;

	ret = qcomtee_load_ta(name);

	return ret ? ret : len;
}

static const struct file_operations qcomtee_load_ta_fops = {
	.owner = THIS_MODULE,
	.write = qcomtee_load_ta_write,
	.llseek = noop_llseek,
};

/*
 * Writing here sends a request to the loaded application; reading gets back
 * whatever it answered.
 */
static ssize_t qcomtee_ta_send_write(struct file *file,
				     const char __user *ubuf, size_t len,
				     loff_t *ppos)
{
	struct qcomtee_object_invoke_ctx *oic;
	struct qcomtee_arg u[3] = { 0 };
	void *request, *response;
	int result;
	int ret;

	if (!len || len > SZ_4K)
		return -EINVAL;

	guard(mutex)(&qcomtee_ta.lock);

	if (!qcomtee_ta.ctx)
		return -ENODEV;

	request = memdup_user(ubuf, len);
	if (IS_ERR(request))
		return PTR_ERR(request);

	response = kzalloc(QCOMTEE_APP_MAX_RESPONSE, GFP_KERNEL);
	if (!response) {
		kfree(request);
		return -ENOMEM;
	}

	oic = qcomtee_object_invoke_ctx_alloc(qcomtee_ta.ctx);
	if (!oic) {
		kfree(response);
		kfree(request);
		return -ENOMEM;
	}

	u[0].type = QCOMTEE_ARG_TYPE_IB;
	u[0].b.addr = request;
	u[0].b.size = len;
	u[1].type = QCOMTEE_ARG_TYPE_OB;
	u[1].b.addr = response;
	u[1].b.size = QCOMTEE_APP_MAX_RESPONSE;

	ret = qcomtee_object_do_invoke(oic, qcomtee_ta.app, qcomtee_ta_op, u,
				       &result);

	pr_info("qcomtee: %s op %u, %zu bytes in: ret %d, result %d, %zu back\n",
		qcomtee_ta.name, qcomtee_ta_op, len, ret, result, u[1].b.size);

	if (!ret && !result) {
		kfree(qcomtee_ta.response);
		qcomtee_ta.response = response;
		qcomtee_ta.response_len = u[1].b.size;
	} else {
		kfree(response);
		if (!ret)
			ret = -EIO;
	}

	kfree(oic);
	kfree(request);

	return ret ? ret : len;
}

static ssize_t qcomtee_ta_send_read(struct file *file, char __user *ubuf,
				    size_t len, loff_t *ppos)
{
	guard(mutex)(&qcomtee_ta.lock);

	if (!qcomtee_ta.response)
		return 0;

	return simple_read_from_buffer(ubuf, len, ppos, qcomtee_ta.response,
				       qcomtee_ta.response_len);
}

/*
 * An application written against the GlobalPlatform interface is reached
 * through a session rather than directly.  Opening one takes the parameter
 * block the specification defines, four in and four out parameters, and four
 * slots for shared memory, and hands back the session object.
 */
static int qcomtee_ta_open_session(void)
{
	struct {
		u32 cancel_code;
		u32 connection_method;
		u32 connection_data;
		u32 param_types;
		u32 ex_param_types;
	} req = {
		.connection_method = qcomtee_ta_connection_method,
		.connection_data = qcomtee_ta_connection_data,
		.param_types = qcomtee_ta_param_types,
	};
	struct {
		u32 memref_out_sz[4];
		u32 ret_value;
		u32 ret_origin;
	} rsp = { 0 };
	struct qcomtee_object *mem = NULL_QCOMTEE_OBJECT;
	struct qcomtee_object_invoke_ctx *oic;
	struct qcomtee_arg u[16] = { 0 };
	struct tee_shm *shm = NULL;
	u8 in[4][256] = { 0 };
	u8 out[4][256] = { 0 };
	size_t psize;
	int result;
	int ret, i;

	if (!qcomtee_ta.ctx)
		return -ENODEV;

	oic = qcomtee_object_invoke_ctx_alloc(qcomtee_ta.ctx);
	if (!oic)
		return -ENOMEM;

	psize = min_t(size_t, qcomtee_ta_param_size, sizeof(in[0]));

	/* Five input buffers: the parameter block and the four parameters. */
	u[0].type = QCOMTEE_ARG_TYPE_IB;
	u[0].b.addr = &req;
	u[0].b.size = sizeof(req);
	for (i = 0; i < 4; i++) {
		u[1 + i].type = QCOMTEE_ARG_TYPE_IB;
		u[1 + i].b.addr = in[i];
		u[1 + i].b.size = psize;
	}

	/* Five output buffers: the result block and the four parameters. */
	u[5].type = QCOMTEE_ARG_TYPE_OB;
	u[5].b.addr = &rsp;
	u[5].b.size = sizeof(rsp);
	for (i = 0; i < 4; i++) {
		u[6 + i].type = QCOMTEE_ARG_TYPE_OB;
		u[6 + i].b.addr = out[i];
		u[6 + i].b.size = psize;
	}

	/*
	 * Four shared-memory slots. The application is given one region to
	 * work in, unless the caller asked for none.
	 */
	if (qcomtee_ta_session_mem) {
		shm = tee_shm_alloc_kernel_buf(qcomtee_ta.ctx,
					       qcomtee_ta_session_mem);
		if (IS_ERR(shm)) {
			ret = PTR_ERR(shm);
			shm = NULL;
			goto out_free_oic;
		}

		memset(tee_shm_get_va(shm, 0), 0, qcomtee_ta_session_mem);

		ret = qcomtee_memobj_from_shm(&mem, shm);
		if (ret)
			goto out_free_shm;
	}

	for (i = 0; i < 4; i++) {
		u[10 + i].type = QCOMTEE_ARG_TYPE_IO;
		u[10 + i].o = i ? NULL_QCOMTEE_OBJECT : mem;
	}

	/* And the session that comes back. */
	u[14].type = QCOMTEE_ARG_TYPE_OO;

	ret = qcomtee_object_do_invoke(oic, qcomtee_ta.controller, 0, u,
				       &result);

	pr_info("qcomtee: %s openSession types %#x psize %zu mem %u: ret %d, result %d, value %#x, origin %u\n",
		qcomtee_ta.name, qcomtee_ta_param_types, psize,
		qcomtee_ta_session_mem, ret, result, rsp.ret_value,
		rsp.ret_origin);

	if (!ret && !result) {
		qcomtee_object_put(qcomtee_ta.app);
		qcomtee_ta.app = u[14].o;
		pr_info("qcomtee: %s session is object %s\n", qcomtee_ta.name,
			qcomtee_object_name(qcomtee_ta.app));
	} else if (!ret) {
		ret = -EIO;
	}

out_free_shm:
	if (shm)
		tee_shm_free(shm);
out_free_oic:
	kfree(oic);

	return ret;
}

static ssize_t qcomtee_ta_session_write(struct file *file,
					const char __user *ubuf, size_t len,
					loff_t *ppos)
{
	int ret;

	guard(mutex)(&qcomtee_ta.lock);

	ret = qcomtee_ta_open_session();

	return ret ? ret : len;
}

static const struct file_operations qcomtee_ta_session_fops = {
	.owner = THIS_MODULE,
	.write = qcomtee_ta_session_write,
	.llseek = noop_llseek,
};

static const struct file_operations qcomtee_ta_send_fops = {
	.owner = THIS_MODULE,
	.write = qcomtee_ta_send_write,
	.read = qcomtee_ta_send_read,
	.llseek = default_llseek,
};

static struct dentry *qcomtee_ta_debugfs;

void qcomtee_apploader_init(void)
{
	qcomtee_ta_debugfs = debugfs_create_dir("qcomtee", NULL);
	debugfs_create_file("load_ta", 0200, qcomtee_ta_debugfs, NULL,
			    &qcomtee_load_ta_fops);
	debugfs_create_file("send", 0600, qcomtee_ta_debugfs, NULL,
			    &qcomtee_ta_send_fops);
	debugfs_create_u32("op", 0600, qcomtee_ta_debugfs, &qcomtee_ta_op);
	debugfs_create_u32("loader_uid", 0600, qcomtee_ta_debugfs,
			   &qcomtee_ta_loader_uid);
	debugfs_create_u32("loader_op", 0600, qcomtee_ta_debugfs,
			   &qcomtee_ta_loader_op);
	debugfs_create_u32("load_by_region", 0600, qcomtee_ta_debugfs,
			   &qcomtee_ta_load_by_region);
	debugfs_create_file("open_session", 0200, qcomtee_ta_debugfs, NULL,
			    &qcomtee_ta_session_fops);
	debugfs_create_u32("connection_method", 0600, qcomtee_ta_debugfs,
			   &qcomtee_ta_connection_method);
	debugfs_create_u32("connection_data", 0600, qcomtee_ta_debugfs,
			   &qcomtee_ta_connection_data);
	debugfs_create_x32("param_types", 0600, qcomtee_ta_debugfs,
			   &qcomtee_ta_param_types);
	debugfs_create_u32("param_size", 0600, qcomtee_ta_debugfs,
			   &qcomtee_ta_param_size);
	debugfs_create_u32("session_mem", 0600, qcomtee_ta_debugfs,
			   &qcomtee_ta_session_mem);
}

void qcomtee_apploader_exit(void)
{
	debugfs_remove_recursive(qcomtee_ta_debugfs);

	mutex_lock(&qcomtee_ta.lock);
	qcomtee_unload_ta_locked();
	mutex_unlock(&qcomtee_ta.lock);
}
