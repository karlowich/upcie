// SPDX-License-Identifier: BSD-3-Clause

/**
 * NVMe Controller Extension: the dmamem-backed queue and admin layer
 * ==================================================================
 *
 * The part of the dmamem controller path that does not care how the
 * device was acquired. Queue-pairs are carved from a caller-provided
 * dmamem_heap and addressed through whatever translator the underlying
 * dmamem carries, so the same code serves vfio-cdev + iommufd
 * (nvme_controller_dmamem_vfio.h), a legacy type1 container
 * (nvme_controller_dmamem_type1.h), and uio_pci_generic
 * (nvme_controller_dmamem_uio.h).
 *
 * What varies between those three -- the device fd, the DMA mapping, the
 * BAR0 acquisition -- stays with each of them. What is shared lives here:
 * SQ/CQ allocation and release, the synchronous admin helper the dmamem
 * admin queue needs, and I/O queue-pair creation and deletion.
 *
 * The submit/reap path itself is the heap-agnostic nvme_qpair primitives
 * (nvme_qpair_enqueue, nvme_qpair_sqdb_update, nvme_qpair_reap_cpl); only
 * the SQ/CQ backing and the DMA-address arithmetic differ from the base
 * upcie hostmem_heap path.
 *
 * @file nvme_controller_dmamem.h
 * @version 0.7.0
 */

/**
 * Allocate SQ, CQ, and a request pool with per-request PRP-list scratch
 * for a queue pair from a dmamem_heap, and populate the nvme_qpair
 * fields the submit/reap primitives read.
 *
 * The heap stays with the caller; qp->heap is left NULL to signal that
 * qp is not managed by hostmem_dma_free. Use nvme_qpair_dmamem_term to
 * free, passing back the same offsets returned here.
 *
 * @param qp             Queue pair to populate; fully memset before use.
 * @param qid            NVMe queue identifier (0 for the admin queue).
 * @param depth          Queue depth in entries.
 * @param bar0           mmap'd BAR0 base; used to derive SQ/CQ doorbell
 *                       addresses via CAP.DSTRD.
 * @param heap           dmamem_heap the SQ, CQ, and PRP scratch are
 *                       carved from.
 * @param sq_offset_out  On success, heap offset of the SQ allocation.
 * @param cq_offset_out  On success, heap offset of the CQ allocation.
 * @param prp_offset_out On success, heap offset of the per-request PRP
 *                       scratch region (needed by nvme_qpair_dmamem_term).
 * @param sq_iova_out    On success, IOVA of the SQ (for AQA/ASQ or
 *                       CREATE_IO_SQ).
 * @param cq_iova_out    On success, IOVA of the CQ (for AQA/ACQ or
 *                       CREATE_IO_CQ).
 *
 * @return 0 on success; negative errno on allocation failure.
 */
static inline int
nvme_qpair_dmamem_init(struct nvme_qpair *qp, uint32_t qid, uint16_t depth, uint8_t *bar0,
		       struct dmamem_heap *heap, size_t *sq_offset_out, size_t *cq_offset_out,
		       size_t *prp_offset_out, uint64_t *sq_iova_out, uint64_t *cq_iova_out)
{
	int dstrd = nvme_reg_cap_get_dstrd(nvme_mmio_cap_read(bar0));
	size_t queue_bytes = 1024 * 64;
	size_t sq_offset = 0, cq_offset = 0, prp_offset = 0;
	int err;

	memset(qp, 0, sizeof(*qp));
	qp->qid = qid;
	qp->depth = depth;
	qp->phase = 1;
	qp->tail_last_written = UINT16_MAX;
	qp->sqdb = bar0 + 0x1000 + ((2 * qid) << (2 + dstrd));
	qp->cqdb = bar0 + 0x1000 + ((2 * qid + 1) << (2 + dstrd));

	/* One element: the controller walks the queue from a single base. */
	err = dmamem_heap_alloc_array_aligned(heap, 1, queue_bytes, 4096, &sq_offset);
	if (err) {
		UPCIE_DEBUG("FAILED: dmamem_heap_alloc_array_aligned(sq); err(%d)", err);
		return err;
	}

	err = dmamem_heap_alloc_array_aligned(heap, 1, queue_bytes, 4096, &cq_offset);
	if (err) {
		UPCIE_DEBUG("FAILED: dmamem_heap_alloc_array_aligned(cq); err(%d)", err);
		dmamem_heap_free(heap, sq_offset);
		return err;
	}

	qp->rpool = calloc(1, sizeof(*qp->rpool));
	if (!qp->rpool) {
		UPCIE_DEBUG("FAILED: calloc(rpool); errno(%d)", errno);
		dmamem_heap_free(heap, cq_offset);
		dmamem_heap_free(heap, sq_offset);
		return -errno;
	}
	nvme_request_pool_init(qp->rpool);

	err = nvme_request_pool_init_prps_dmamem(qp->rpool, heap, &prp_offset);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_request_pool_init_prps_dmamem(); err(%d)", err);
		free(qp->rpool);
		qp->rpool = NULL;
		dmamem_heap_free(heap, cq_offset);
		dmamem_heap_free(heap, sq_offset);
		return err;
	}

	qp->sq = dmamem_heap_at_va(heap, sq_offset);
	qp->cq = dmamem_heap_at_va(heap, cq_offset);
	memset(qp->sq, 0, queue_bytes);
	memset(qp->cq, 0, queue_bytes);

	*sq_offset_out = sq_offset;
	*cq_offset_out = cq_offset;
	*prp_offset_out = prp_offset;
	*sq_iova_out = dmamem_heap_at_iova(heap, sq_offset);
	*cq_iova_out = dmamem_heap_at_iova(heap, cq_offset);

	return 0;
}

/**
 * Release the SQ, CQ, and request pool of a qp initialised by
 * nvme_qpair_dmamem_init.
 *
 * @param qp          Queue pair to release; memset to zero on return.
 * @param heap        The heap the SQ, CQ, and PRP scratch were carved from.
 * @param sq_offset   SQ offset returned by nvme_qpair_dmamem_init.
 * @param cq_offset   CQ offset returned by nvme_qpair_dmamem_init.
 * @param prp_offset  PRP scratch offset returned by nvme_qpair_dmamem_init.
 */
static inline void
nvme_qpair_dmamem_term(struct nvme_qpair *qp, struct dmamem_heap *heap, size_t sq_offset,
		       size_t cq_offset, size_t prp_offset)
{
	if (qp->rpool) {
		nvme_request_pool_term_prps_dmamem(qp->rpool, heap, prp_offset);
		free(qp->rpool);
		qp->rpool = NULL;
	}
	if (qp->sq) {
		dmamem_heap_free(heap, sq_offset);
	}
	if (qp->cq) {
		dmamem_heap_free(heap, cq_offset);
	}
	memset(qp, 0, sizeof(*qp));
}

/**
 * Synchronous admin-command helper for the dmamem admin queue.
 *
 * The dmamem admin queue does not carry a request pool, so submit_sync
 * cannot be used. This helper does a manual enqueue + doorbell + reap
 * with a caller-supplied CID.
 */
static inline int
nvme_admin_sync_dmamem(struct nvme_controller *ctrlr, struct nvme_command *cmd, uint16_t cid,
		       struct nvme_completion *cpl)
{
	int err;

	cmd->cid = cid;

	err = nvme_qpair_enqueue(&ctrlr->aq, cmd);
	if (err) {
		return err;
	}
	nvme_qpair_sqdb_update(&ctrlr->aq);

	err = nvme_qpair_reap_cpl(&ctrlr->aq, ctrlr->timeout_ms, cpl);
	if (err) {
		return err;
	}
	if ((cpl->status >> 1) & 0x7FF) {
		UPCIE_DEBUG("FAILED: admin CQE status=0x%x", cpl->status);
		return -EIO;
	}
	return 0;
}

/**
 * Create an I/O queue pair on the dmamem path.
 *
 * Allocates SQ/CQ from the caller's dmamem_heap, then programs the
 * controller via admin CREATE_IO_CQ + CREATE_IO_SQ so the controller
 * knows about the new qpair. The resulting nvme_qpair is compatible
 * with the heap-agnostic submit/reap primitives (nvme_qpair_enqueue,
 * nvme_qpair_sqdb_update, nvme_qpair_reap_cpl).
 *
 * The qid is allocated from the controller's bitmap; the caller must
 * hold on to the returned sq_offset/cq_offset until
 * nvme_controller_delete_io_qpair_dmamem is called.
 */
static inline int
nvme_controller_create_io_qpair_dmamem(struct nvme_controller *ctrlr, struct nvme_qpair *qp,
				       uint16_t depth, struct dmamem_heap *heap,
				       size_t *sq_offset_out, size_t *cq_offset_out,
				       size_t *prp_offset_out)
{
	struct nvme_command cmd = {0};
	struct nvme_completion cpl = {0};
	uint64_t sq_iova = 0, cq_iova = 0;
	uint16_t qid;
	int err;

	err = nvme_qid_find_free(ctrlr->qids);
	if (err < 1) {
		return -ENOMEM;
	}
	qid = err;

	err = nvme_qid_alloc(ctrlr->qids, qid);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_qid_alloc; err(%d)", err);
		return err;
	}

	err = nvme_qpair_dmamem_init(qp, qid, depth, ctrlr->func.bars[0].region, heap, sq_offset_out,
				     cq_offset_out, prp_offset_out, &sq_iova, &cq_iova);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_qpair_dmamem_init(io); err(%d)", err);
		nvme_qid_free(ctrlr->qids, qid);
		return err;
	}

	memset(&cmd, 0, sizeof(cmd));
	cmd.opc = 0x5; /* Create I/O Completion Queue */
	cmd.prp1 = cq_iova;
	cmd.cdw10 = ((uint32_t)(depth - 1) << 16) | qid;
	cmd.cdw11 = 0x1; /* Physically contiguous, no interrupts */
	err = nvme_admin_sync_dmamem(ctrlr, &cmd, 2, &cpl);
	if (err) {
		UPCIE_DEBUG("FAILED: CREATE_IO_CQ(qid=%u); err(%d)", qid, err);
		goto rollback_qpair;
	}

	memset(&cmd, 0, sizeof(cmd));
	memset(&cpl, 0, sizeof(cpl));
	cmd.opc = 0x1; /* Create I/O Submission Queue */
	cmd.prp1 = sq_iova;
	cmd.cdw10 = ((uint32_t)(depth - 1) << 16) | qid;
	cmd.cdw11 = ((uint32_t)qid << 16) | 0x1; /* CQID | physically contiguous */
	err = nvme_admin_sync_dmamem(ctrlr, &cmd, 3, &cpl);
	if (err) {
		struct nvme_command drop = {0};
		struct nvme_completion drop_cpl = {0};

		UPCIE_DEBUG("FAILED: CREATE_IO_SQ(qid=%u); err(%d)", qid, err);
		drop.opc = 0x4; /* Delete I/O Completion Queue */
		drop.cdw10 = qid;
		(void)nvme_admin_sync_dmamem(ctrlr, &drop, 4, &drop_cpl);
		goto rollback_qpair;
	}

	return 0;

rollback_qpair:
	nvme_qpair_dmamem_term(qp, heap, *sq_offset_out, *cq_offset_out, *prp_offset_out);
	nvme_qid_free(ctrlr->qids, qid);
	return err;
}

/**
 * Tear down an I/O queue pair created with the dmamem variant.
 */
static inline int
nvme_controller_delete_io_qpair_dmamem(struct nvme_controller *ctrlr, struct nvme_qpair *qp,
				       struct dmamem_heap *heap, size_t sq_offset, size_t cq_offset,
				       size_t prp_offset)
{
	struct nvme_command cmd = {0};
	struct nvme_completion cpl = {0};
	uint16_t qid = qp->qid;
	int first_err = 0;
	int err;

	cmd.opc = 0x0; /* Delete I/O Submission Queue */
	cmd.cdw10 = qid;
	err = nvme_admin_sync_dmamem(ctrlr, &cmd, 5, &cpl);
	if (err && !first_err) {
		first_err = err;
	}

	memset(&cmd, 0, sizeof(cmd));
	memset(&cpl, 0, sizeof(cpl));
	cmd.opc = 0x4; /* Delete I/O Completion Queue */
	cmd.cdw10 = qid;
	err = nvme_admin_sync_dmamem(ctrlr, &cmd, 6, &cpl);
	if (err && !first_err) {
		first_err = err;
	}

	nvme_qpair_dmamem_term(qp, heap, sq_offset, cq_offset, prp_offset);
	nvme_qid_free(ctrlr->qids, qid);
	return first_err;
}
