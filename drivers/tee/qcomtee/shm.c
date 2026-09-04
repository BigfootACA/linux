// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cma.h>
#include <linux/dma-map-ops.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/mm.h>

#include "qcomtee.h"

/**
 * define MAX_OUTBOUND_BUFFER_SIZE - Maximum size of outbound buffers.
 *
 * The size of outbound buffer depends on QTEE callback requests.
 */
#define MAX_OUTBOUND_BUFFER_SIZE SZ_4K

/**
 * define MAX_INBOUND_BUFFER_SIZE - Maximum size of the inbound buffer.
 *
 * The size of the inbound buffer depends on the user's requests,
 * specifically the number of IB and OB arguments. If an invocation
 * requires a size larger than %MAX_INBOUND_BUFFER_SIZE, the user should
 * consider using another form of shared memory with QTEE.
 */
#define MAX_INBOUND_BUFFER_SIZE SZ_4M

/**
 * qcomtee_msg_buffers_alloc() - Allocate inbound and outbound buffers.
 * @oic: context to use for the current invocation.
 * @u: array of arguments for the current invocation.
 *
 * It calculates the size of inbound and outbound buffers based on the
 * arguments in @u. It allocates the buffers from the teedev pool.
 *
 * Return: On success, returns 0. On error, returns < 0.
 */
int qcomtee_msg_buffers_alloc(struct qcomtee_object_invoke_ctx *oic,
			      struct qcomtee_arg *u)
{
	struct tee_context *ctx = oic->ctx;
	struct tee_shm *shm;
	size_t size;
	int i;

	/* Start offset in a message for buffer arguments. */
	size = qcomtee_msg_buffer_args(struct qcomtee_msg_object_invoke,
				       qcomtee_args_len(u));
	if (size > MAX_INBOUND_BUFFER_SIZE)
		return -EINVAL;

	/* Add size of IB arguments. */
	qcomtee_arg_for_each_input_buffer(i, u) {
		size = size_add(size, qcomtee_msg_offset_align(u[i].b.size));
		if (size > MAX_INBOUND_BUFFER_SIZE)
			return -EINVAL;
	}

	/* Add size of OB arguments. */
	qcomtee_arg_for_each_output_buffer(i, u) {
		size = size_add(size, qcomtee_msg_offset_align(u[i].b.size));
		if (size > MAX_INBOUND_BUFFER_SIZE)
			return -EINVAL;
	}

	shm = tee_shm_alloc_priv_buf(ctx, size);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	/* Allocate inbound buffer. */
	oic->in_shm = shm;
	shm = tee_shm_alloc_priv_buf(ctx, MAX_OUTBOUND_BUFFER_SIZE);
	if (IS_ERR(shm)) {
		tee_shm_free(oic->in_shm);

		return PTR_ERR(shm);
	}
	/* Allocate outbound buffer. */
	oic->out_shm = shm;

	oic->in_msg.addr = tee_shm_get_va(oic->in_shm, 0);
	oic->in_msg.size = tee_shm_get_size(oic->in_shm);
	oic->out_msg.addr = tee_shm_get_va(oic->out_shm, 0);
	oic->out_msg.size = tee_shm_get_size(oic->out_shm);
	/* QTEE assume unused buffers are zeroed. */
	memzero_explicit(oic->in_msg.addr, oic->in_msg.size);
	memzero_explicit(oic->out_msg.addr, oic->out_msg.size);

	return 0;
}

void qcomtee_msg_buffers_free(struct qcomtee_object_invoke_ctx *oic)
{
	tee_shm_free(oic->in_shm);
	tee_shm_free(oic->out_shm);
}

/* Dynamic shared memory pool based on tee_dyn_shm_alloc_helper(). */

static int qcomtee_shm_register(struct tee_context *ctx, struct tee_shm *shm,
				struct page **pages, size_t num_pages,
				unsigned long start)
{
	return qcom_tzmem_shm_bridge_create(shm->paddr, shm->size,
					    &shm->sec_world_id);
}

static int qcomtee_shm_unregister(struct tee_context *ctx, struct tee_shm *shm)
{
	qcom_tzmem_shm_bridge_delete(shm->sec_world_id);

	return 0;
}

static int qcomtee_shm_alloc_cma(struct tee_shm_pool *pool,
				 struct tee_shm *shm, size_t size)
{
#if IS_ENABLED(CONFIG_DMA_CMA)
	struct cma *cma = pool->private_data;
	struct page *page;
	struct page **pages;
	size_t num_pages;
	size_t i;
	int ret;

	if (!cma || !size || size > SIZE_MAX - (PAGE_SIZE - 1))
		return -ENOMEM;

	shm->size = PAGE_ALIGN(size);
	num_pages = shm->size >> PAGE_SHIFT;
	pages = kcalloc(num_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	page = cma_alloc(cma, num_pages, 0, false);
	if (!page) {
		ret = -ENOMEM;
		goto err_free_pages;
	}

	shm->kaddr = page_address(page);
	if (!shm->kaddr) {
		ret = -ENOMEM;
		goto err_release_cma;
	}

	shm->paddr = page_to_phys(page);
	shm->pages = pages;
	shm->num_pages = num_pages;
	for (i = 0; i < num_pages; i++)
		pages[i] = pfn_to_page(page_to_pfn(page) + i);

	memset(shm->kaddr, 0, shm->size);
	ret = qcomtee_shm_register(shm->ctx, shm, pages, num_pages,
				   (unsigned long)shm->kaddr);
	if (ret)
		goto err_release_cma;

	return 0;

err_release_cma:
	shm->kaddr = NULL;
	shm->pages = NULL;
	shm->num_pages = 0;
	cma_release(cma, page, num_pages);
err_free_pages:
	kfree(pages);
	return ret;
#else
	return -ENOMEM;
#endif
}

static void qcomtee_shm_free_cma(struct tee_shm_pool *pool,
				 struct tee_shm *shm)
{
#if IS_ENABLED(CONFIG_DMA_CMA)
	struct cma *cma = pool->private_data;
	struct page *page = shm->pages[0];

	qcomtee_shm_unregister(shm->ctx, shm);
	WARN_ON_ONCE(!cma_release(cma, page, shm->num_pages));

	shm->kaddr = NULL;
	kfree(shm->pages);
	shm->pages = NULL;
	shm->num_pages = 0;
#endif
}

static bool qcomtee_shm_needs_cma(size_t size)
{
	return size && get_order(size) > MAX_PAGE_ORDER;
}

/* Some legacy QSEE applications only accept 32-bit physical addresses. */
static int qcomtee_shm_alloc_user_lowmem(struct tee_shm *shm, size_t size)
{
	struct page **pages;
	size_t num_pages;
	size_t i;
	int ret;

	if (!size || size > SIZE_MAX - (PAGE_SIZE - 1))
		return -ENOMEM;

	shm->size = PAGE_ALIGN(size);
	shm->kaddr = alloc_pages_exact(shm->size,
				       GFP_KERNEL | GFP_DMA | __GFP_ZERO);
	if (!shm->kaddr)
		return -ENOMEM;

	shm->paddr = virt_to_phys(shm->kaddr);
	if (shm->paddr >= SZ_4G || shm->size > SZ_4G - shm->paddr) {
		ret = -ENOMEM;
		goto err_free_exact;
	}

	num_pages = shm->size >> PAGE_SHIFT;
	pages = kcalloc(num_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		goto err_free_exact;
	}

	for (i = 0; i < num_pages; i++)
		pages[i] = virt_to_page((u8 *)shm->kaddr + i * PAGE_SIZE);

	shm->pages = pages;
	shm->num_pages = num_pages;
	ret = qcomtee_shm_register(shm->ctx, shm, pages, num_pages,
				   (unsigned long)shm->kaddr);
	if (ret)
		goto err_free_page_array;

	return 0;

err_free_page_array:
	kfree(shm->pages);
	shm->pages = NULL;
	shm->num_pages = 0;
err_free_exact:
	free_pages_exact(shm->kaddr, shm->size);
	shm->kaddr = NULL;
	return ret;
}

static int pool_op_alloc(struct tee_shm_pool *pool, struct tee_shm *shm,
			 size_t size, size_t align)
{
	if (qcomtee_shm_needs_cma(size))
		return qcomtee_shm_alloc_cma(pool, shm, size);
	/* TEE_IOC_SHM_ALLOC assigns user-visible buffers a nonnegative ID. */
	if (size && shm->id >= 0)
		return qcomtee_shm_alloc_user_lowmem(shm, size);

	return tee_dyn_shm_alloc_helper(shm, size, align, qcomtee_shm_register);
}

static void pool_op_free(struct tee_shm_pool *pool, struct tee_shm *shm)
{
	if (qcomtee_shm_needs_cma(shm->size)) {
		qcomtee_shm_free_cma(pool, shm);
		return;
	}

	tee_dyn_shm_free_helper(shm, qcomtee_shm_unregister);
}

static void pool_op_destroy_pool(struct tee_shm_pool *pool)
{
	kfree(pool);
}

static const struct tee_shm_pool_ops pool_ops = {
	.alloc = pool_op_alloc,
	.free = pool_op_free,
	.destroy_pool = pool_op_destroy_pool,
};

struct tee_shm_pool *qcomtee_shm_pool_alloc(struct device *dev)
{
	struct tee_shm_pool *pool;

	pool = kzalloc_obj(*pool);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->ops = &pool_ops;
#if IS_ENABLED(CONFIG_DMA_CMA)
	pool->private_data = dev_get_cma_area(dev);
#endif

	return pool;
}
