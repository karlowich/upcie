// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Simon Andreas Frimann Lund <os@safl.dk>

/**
 * Rudimentary Representation of Controller, BAR Mapping, Registers, and Derived Values
 * ====================================================================================
 *
 * This header defines basic structures and access patterns for working with an NVMe controller,
 * including BAR-space mappings, controller registers, and values derived from register content.
 *
 * @file nvme_controller.h
 * @version 0.7.0
 */

/**
 * This is one way of combining the various components needed
 */
struct nvme_controller {
	struct pci_func func;                 ///< The PCIe function and mapped bars
	struct nvme_qpair aq;                 ///< Admin qpair
	uint64_t qids[NVME_QID_BITMAP_WORDS]; ///< Allocation status of IO queues

	struct hostmem_heap *heap; ///< Heap for DMA-capable memory
	void *buf;                 ///< IO-buffer for identify-commands, io-qpair-creation etc.

	uint32_t csts; ///< Controller Status Register Value
	uint32_t cap;  ///< Controller Capabilities Register Value
	uint32_t cc;   ///< Controller configuration Register Value

	int timeout_ms; ///< Command timeout in milliseconds (derived from cap.to)
};

static inline void
nvme_controller_close(struct nvme_controller *ctrlr)
{
	if (ctrlr->aq.rpool) {
		nvme_qpair_term(&ctrlr->aq);
		memset(&ctrlr->aq, 0, sizeof(ctrlr->aq));
	}

	if (ctrlr->buf) {
		hostmem_dma_free(ctrlr->heap, ctrlr->buf);
		ctrlr->buf = NULL;
	}

	pci_func_close(&ctrlr->func);
	memset(ctrlr, 0, sizeof(*ctrlr));
}

/**
 * Disables the NVMe controller at 'bdf', sets up admin-queues and enables it again
 */
static inline int
nvme_controller_open(struct nvme_controller *ctrlr, const char *bdf, struct hostmem_heap *heap)
{
	uint64_t cap;
	void *bar0;
	int err;

	memset(ctrlr, 0, sizeof(*ctrlr));
	ctrlr->heap = heap;

	ctrlr->buf = hostmem_dma_malloc(ctrlr->heap, 4096);
	if (!ctrlr->buf) {
		UPCIE_DEBUG("FAILED: hostmem_dma_malloc(buf); errno(%d)\n", errno);
		return -errno;
	}
	memset(ctrlr->buf, 0, 4096);

	nvme_qid_bitmap_init(ctrlr->qids);

	err = pci_func_open(bdf, &ctrlr->func);
	if (err) {
		UPCIE_DEBUG("FAILED: pci_func_open(%.*s); err(%d)", 13, bdf, err);
		return -err;
	}

	err = pci_bar_map(ctrlr->func.bdf, 0, &ctrlr->func.bars[0]);
	if (err) {
		UPCIE_DEBUG("FAILED: pci_bar_map(BAR0); err(%d)", err);
		return -err;
	}
	bar0 = ctrlr->func.bars[0].region;

	cap = nvme_mmio_cap_read(bar0);
	// CAP.TO is encoded in units of 500 ms.
	ctrlr->timeout_ms = nvme_reg_cap_get_to(cap) * 500;

	nvme_mmio_cc_disable(bar0);

	err = nvme_mmio_csts_wait_until_not_ready(bar0, ctrlr->timeout_ms);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_mmio_csts_wait_until_ready(); err(%d)\n", err);
		return -err;
	}

	err = nvme_qpair_init(&ctrlr->aq, 0, 256, ctrlr->func.bars[0].region, ctrlr->heap);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_qpair_init(); err(%d)", err);
		return -err;
	}

	nvme_mmio_aq_setup(bar0, hostmem_dma_v2p(heap, ctrlr->aq.sq),
			   hostmem_dma_v2p(heap, ctrlr->aq.cq), ctrlr->aq.depth);

	{
		uint32_t css = (nvme_reg_cap_get_css(cap) & (1 << 6)) ? 0x6 : 0x0;
		uint32_t cc = 0;

		cc = nvme_reg_cc_set_css(cc, css);
		cc = nvme_reg_cc_set_shn(cc, 0x0);
		cc = nvme_reg_cc_set_mps(cc, 0x0);
		cc = nvme_reg_cc_set_ams(cc, 0x0);
		cc = nvme_reg_cc_set_iosqes(cc, 6);
		cc = nvme_reg_cc_set_iocqes(cc, 4);
		cc = nvme_reg_cc_set_en(cc, 0x1);

		nvme_mmio_cc_write(bar0, cc);
	}

	err = nvme_mmio_csts_wait_until_ready(bar0, ctrlr->timeout_ms);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_mmio_csts_wait_until_ready(); err(%d)", err);
		return -err;
	}

	return 0;
}

/**
 * Sends a Delete I/O Completion Queue admin command for `qid`
 *
 * @param ctrlr Pointer to a pre-allocated NVMe controller
 * @param qid The identifier of the completion-queue to delete
 *
 * @return 0 on success, negative errno on error.
 */
static inline int
nvme_controller_delete_io_cq(struct nvme_controller *ctrlr, uint16_t qid)
{
	struct nvme_command cmd;
	struct nvme_completion cpl = {0};

	nvme_command_delete_io_cq(&cmd, qid);

	return nvme_qpair_submit_sync(&ctrlr->aq, &cmd, ctrlr->timeout_ms, &cpl);
}

/**
 * Tell the controller about an already-allocated I/O queue pair.
 *
 * Sends Create I/O CQ then Create I/O SQ, which is the required order: the
 * submission queue names the completion queue it reports into, and a controller
 * asked for it the other way round answers SCT=1/SC=0x0, "Completion Queue
 * Invalid". If the submission queue is refused, the completion queue this call
 * created is deleted again, so a failure leaves the controller as it was found.
 *
 * The queue memory is the caller's: this programs the addresses and nothing
 * else, which is what lets one implementation serve the hostmem, dmamem and GPU
 * backings. `sq_addr` and `cq_addr` are the addresses as the controller sees
 * them -- an IOVA where a mapping was installed, a physical address where none
 * was.
 *
 * Should that rollback fail too, the controller is left holding a completion
 * queue under `qid` and `*qid_orphaned` is set. The caller reserved the qid, so
 * only the caller can retire it, and retire it it must: handing the same qid out
 * again collides with the queue the controller still has. Freeing the queue
 * memory stays safe either way, since no submission queue was ever bound to that
 * completion queue and nothing can post into it.
 *
 * @param ctrlr         Pointer to a pre-allocated NVMe controller
 * @param qid           Identifier for the pair; already reserved by the caller
 * @param depth         Queue depth in entries
 * @param sq_addr       Address of the submission queue
 * @param cq_addr       Address of the completion queue
 * @param qid_orphaned  Set to 1 when the controller kept a completion queue
 *                      under `qid`; left untouched otherwise
 *
 * @return 0 on success, negative errno on error.
 */
static inline int
nvme_controller_program_io_qpair(struct nvme_controller *ctrlr, uint16_t qid, uint16_t depth,
				 uint64_t sq_addr, uint64_t cq_addr, int *qid_orphaned)
{
	struct nvme_command cmd;
	struct nvme_completion cpl = {0};
	int err;

	nvme_command_create_io_cq(&cmd, qid, depth, cq_addr);
	err = nvme_qpair_submit_sync(&ctrlr->aq, &cmd, ctrlr->timeout_ms, &cpl);
	if (err) {
		UPCIE_DEBUG("FAILED: Create I/O CQ(qid=%u); err(%d)", qid, err);
		return err;
	}

	memset(&cpl, 0, sizeof(cpl));
	nvme_command_create_io_sq(&cmd, qid, depth, sq_addr);
	err = nvme_qpair_submit_sync(&ctrlr->aq, &cmd, ctrlr->timeout_ms, &cpl);
	if (err) {
		int del_err;

		UPCIE_DEBUG("FAILED: Create I/O SQ(qid=%u); err(%d)", qid, err);

		/* Kept out of err, which carries the failure being unwound. */
		del_err = nvme_controller_delete_io_cq(ctrlr, qid);
		if (del_err) {
			UPCIE_DEBUG("FAILED: nvme_controller_delete_io_cq(); err(%d)", del_err);
			*qid_orphaned = 1;
		}

		return err;
	}

	return 0;
}

/**
 * Tell the controller to forget an I/O queue pair.
 *
 * Sends Delete I/O SQ then Delete I/O CQ, the required order: a completion
 * queue still bound to a submission queue cannot be deleted. Both are attempted
 * even if the first fails, so the controller is left as clean as it can be, and
 * the first error is what comes back.
 *
 * The queue memory is the caller's and is not touched here.
 *
 * @param ctrlr Pointer to a pre-allocated NVMe controller
 * @param qid   Identifier of the pair to delete
 *
 * @return 0 on success, negative errno of the first failure otherwise.
 */
static inline int
nvme_controller_unprogram_io_qpair(struct nvme_controller *ctrlr, uint16_t qid)
{
	struct nvme_command cmd;
	struct nvme_completion cpl = {0};
	int first_err = 0;
	int err;

	nvme_command_delete_io_sq(&cmd, qid);
	err = nvme_qpair_submit_sync(&ctrlr->aq, &cmd, ctrlr->timeout_ms, &cpl);
	if (err) {
		UPCIE_DEBUG("FAILED: Delete I/O SQ(qid=%u); err(%d)", qid, err);
		first_err = err;
	}

	memset(&cpl, 0, sizeof(cpl));
	nvme_command_delete_io_cq(&cmd, qid);
	err = nvme_qpair_submit_sync(&ctrlr->aq, &cmd, ctrlr->timeout_ms, &cpl);
	if (err) {
		UPCIE_DEBUG("FAILED: Delete I/O CQ(qid=%u); err(%d)", qid, err);
		if (!first_err) {
			first_err = err;
		}
	}

	return first_err;
}

/**
 * Deletes the submission-queue and completion-queue and frees host-side resources.
 *
 * Sends Delete I/O SQ and Delete I/O CQ admin commands to the controller, then
 * releases the host DMA memory and returns the queue ID to the free pool.
 *
 * @param ctrlr Pointer to a pre-allocated NVMe controller
 * @param qpair Pointer to a queue-pair (from nvme_controller_create_io_qpair)
 *
 * @return 0 on success, negative errno on error. Resources are freed regardless.
 */
static inline int
nvme_controller_delete_io_qpair(struct nvme_controller *ctrlr, struct nvme_qpair *qpair)
{
	uint16_t qid = qpair->qid;
	int err;

	err = nvme_controller_unprogram_io_qpair(ctrlr, qid);

	nvme_qpair_term(qpair);
	nvme_qid_free(ctrlr->qids, qid);

	return err;
}

/**
 * Allocates a submission-queue, a completion-queue, and wraps them in the nvme_qpair struct
 */
static inline int
nvme_controller_create_io_qpair(struct nvme_controller *ctrlr, struct nvme_qpair *qpair,
				uint16_t depth)
{
	uint16_t qid;
	int qid_orphaned = 0;
	int err;

	err = nvme_qid_find_free(ctrlr->qids);
	if (err < 1) {
		return -ENOMEM;
	}
	qid = err;

	err = nvme_qid_alloc(ctrlr->qids, qid);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_qid_alloc(): err(%d)", err);
		return err;
	}

	err = nvme_qpair_init(qpair, qid, depth, ctrlr->func.bars[0].region, ctrlr->heap);
	if (err) {
		UPCIE_DEBUG("FAILED: nvme_qpair_init(); err(%d)", err);
		goto free_qid;
	}

	err = nvme_controller_program_io_qpair(ctrlr, qid, depth,
					       hostmem_dma_v2p(ctrlr->heap, qpair->sq),
					       hostmem_dma_v2p(ctrlr->heap, qpair->cq),
					       &qid_orphaned);
	if (err) {
		goto term_qpair;
	}

	return 0;

term_qpair:
	nvme_qpair_term(qpair);
	memset(qpair, 0, sizeof(*qpair));
free_qid:
	if (!qid_orphaned) {
		nvme_qid_free(ctrlr->qids, qid);
	}

	return err;
}
