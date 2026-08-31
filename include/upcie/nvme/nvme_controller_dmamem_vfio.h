// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Simon Andreas Frimann Lund <os@safl.dk>

/**
 * NVMe Controller Extension: dmamem over vfio-cdev + iommufd
 * ==========================================================
 *
 * Sibling of nvme_controller_vfio for the vfio-cdev + iommufd + dmamem
 * world. The caller owns the iommufd handle and the target IOAS; this
 * module owns the vfio device attached to that IOAS, the mmap'd BAR0,
 * and the admin queue buffers sub-allocated from a caller-provided
 * dmamem_heap.
 *
 * Only the acquisition half lives here. The queue and admin layer built
 * on the heap is shared with the type1 and uio backings and lives in
 * nvme_controller_dmamem.h, which this header requires.
 *
 * @file nvme_controller_dmamem_vfio.h
 * @version 0.7.0
 */

/**
 * Context for a single NVMe controller reached via vfio-cdev + iommufd.
 *
 * The iommufd handle and IOAS are caller-owned. The vfio_device_path
 * (e.g. /dev/vfio/devices/vfio3) identifies the cdev; the caller
 * resolves it from BDF via sysfs, then hands it in.
 */
struct nvme_dmamem_vfio_ctx {
	struct iommufd *iommufd;   ///< Not owned; caller lifetime; carries the IOAS id
	struct vfio_cdev dev;      ///< Owned; vfio cdev bound + attached
	void *bar0;                ///< mmap'd BAR0
	size_t bar0_size;          ///< BAR0 mapping length
	size_t aq_sq_offset;       ///< Admin SQ offset in the caller-provided heap
	size_t aq_cq_offset;       ///< Admin CQ offset in the caller-provided heap
	size_t aq_prp_offset;      ///< Admin PRP-scratch offset in the caller-provided heap
	int attached;              ///< Whether dev is attached to the IOAS
	int aq_alive;              ///< Whether aq_{sq,cq,prp}_offset hold live allocations
};

static inline void
nvme_dmamem_vfio_ctx_init(struct nvme_dmamem_vfio_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->dev.fd = -1;
}

/**
 * Release the vfio device + BAR0 mapping.
 *
 * The caller-owned iommufd handle and IOAS stay alive.
 */
static inline int
nvme_dmamem_vfio_ctx_close(struct nvme_dmamem_vfio_ctx *ctx)
{
	int first_err = 0;
	int err;

	if (ctx->bar0 && ctx->bar0_size) {
		munmap(ctx->bar0, ctx->bar0_size);
		ctx->bar0 = NULL;
		ctx->bar0_size = 0;
	}

	if (ctx->attached) {
		err = iommufd_detach(&ctx->dev);
		if (err && !first_err) {
			first_err = err;
		}
		ctx->attached = 0;
	}

	if (ctx->dev.fd >= 0) {
		err = vfio_cdev_close(&ctx->dev);
		if (err && !first_err) {
			first_err = err;
		}
	}

	nvme_dmamem_vfio_ctx_init(ctx);

	return first_err;
}

/**
 * Open an NVMe controller through vfio-cdev + iommufd.
 *
 * @param ctrlr           Controller descriptor to populate.
 * @param ctx             Controller context (owned by caller until close).
 * @param iommufd         Open iommufd handle (caller-owned).
 * @param heap            dmamem_heap holding admin queue backing storage.
 *                        The heap must outlive the controller; its SQ/CQ
 *                        offsets are stored in ctx and freed by
 *                        nvme_controller_close_dmamem_vfio.
 * @param vfio_device_path Path to the vfio cdev, e.g. /dev/vfio/devices/vfio3.
 *
 * @return 0 on success, negative errno on error.
 */
static inline int
nvme_controller_open_dmamem_vfio(struct nvme_controller *ctrlr, struct nvme_dmamem_vfio_ctx *ctx,
			    struct iommufd *iommufd, struct dmamem_heap *heap,
			    const char *vfio_device_path)
{
	uint64_t sq_iova = 0, cq_iova = 0;
	uint64_t cap;
	void *bar0;
	int err;

	if (!ctrlr || !ctx || !iommufd || !heap || !vfio_device_path) {
		return -EINVAL;
	}

	memset(ctrlr, 0, sizeof(*ctrlr));
	nvme_dmamem_vfio_ctx_init(ctx);
	ctx->iommufd = iommufd;

	nvme_qid_bitmap_init(ctrlr->qids);

	err = vfio_cdev_open(vfio_device_path, &ctx->dev);
	if (err) {
		UPCIE_DEBUG("FAILED: vfio_cdev_open('%s'); err(%d)", vfio_device_path, err);
		goto fail;
	}

	err = iommufd_bind(iommufd, &ctx->dev);
	if (err) {
		UPCIE_DEBUG("FAILED: iommufd_bind(); err(%d)", err);
		goto fail;
	}

	err = iommufd_attach(iommufd, &ctx->dev);
	if (err) {
		UPCIE_DEBUG("FAILED: iommufd_attach(); err(%d)", err);
		goto fail;
	}
	ctx->attached = 1;

	err = nvme_vfio_pci_acquire_bar0(ctx->dev.fd, ctrlr, &ctx->bar0, &ctx->bar0_size);
	if (err) {
		goto fail;
	}
	bar0 = ctx->bar0;

	err = nvme_controller_reset_via_bar0(ctrlr, bar0, &cap);
	if (err) {
		goto fail;
	}

	err = nvme_qpair_dmamem_init(&ctrlr->aq, 0, 256, bar0, heap, &ctx->aq_sq_offset,
				     &ctx->aq_cq_offset, &ctx->aq_prp_offset, &sq_iova, &cq_iova);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_qpair_dmamem_init(aq); err(%d)", err);
		goto fail;
	}
	ctx->aq_alive = 1;

	nvme_mmio_aq_setup(bar0, sq_iova, cq_iova, ctrlr->aq.depth);

	err = nvme_controller_enable_via_bar0(ctrlr, bar0, cap);
	if (err) {
		goto fail;
	}

	return 0;

fail:
	if (ctx->aq_alive) {
		nvme_qpair_dmamem_term(&ctrlr->aq, heap, ctx->aq_sq_offset, ctx->aq_cq_offset,
				       ctx->aq_prp_offset);
		ctx->aq_alive = 0;
	}
	nvme_dmamem_vfio_ctx_close(ctx);
	memset(ctrlr, 0, sizeof(*ctrlr));
	return err;
}

/**
 * Close a controller opened with nvme_controller_open_dmamem_vfio.
 *
 * Disables the controller, releases the admin queue's SQ/CQ back to
 * heap (using the offsets ctx recorded at open), releases the vfio
 * device attachment, and munmaps BAR0. The dmamem_heap itself stays
 * with the caller.
 *
 * @param ctrlr  Controller populated by nvme_controller_open_dmamem_vfio;
 *               memset to zero on return.
 * @param ctx    Context populated by nvme_controller_open_dmamem_vfio;
 *               reset to a fresh state on return.
 * @param heap   The same heap that was passed to open; must still be
 *               valid.
 *
 * @return 0 on success; first negative errno encountered during
 * teardown otherwise.
 */
static inline int
nvme_controller_close_dmamem_vfio(struct nvme_controller *ctrlr, struct nvme_dmamem_vfio_ctx *ctx,
			     struct dmamem_heap *heap)
{
	int first_err = 0;
	int err;

	if (ctx->bar0) {
		nvme_mmio_cc_disable(ctx->bar0);
		if (ctrlr->timeout_ms) {
			err = nvme_mmio_csts_wait_until_not_ready(ctx->bar0, ctrlr->timeout_ms);
			if (err) {
				first_err = err;
			}
		}
	}

	if (ctx->aq_alive) {
		nvme_qpair_dmamem_term(&ctrlr->aq, heap, ctx->aq_sq_offset, ctx->aq_cq_offset,
				       ctx->aq_prp_offset);
		ctx->aq_alive = 0;
	}

	err = nvme_dmamem_vfio_ctx_close(ctx);
	if (err && !first_err) {
		first_err = err;
	}

	memset(ctrlr, 0, sizeof(*ctrlr));

	return first_err;
}

