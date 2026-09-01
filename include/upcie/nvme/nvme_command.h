// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Simon Andreas Frimann Lund <os@safl.dk>

/**
 * Rudimentary Representation of Commands and Completions
 * ======================================================
 *
 * This header defines minimal representations of NVMe commands and their completions,
 * suitable for low-level or embedded NVMe driver implementations.
 *
 * It also carries the builders for the I/O queue-pair admin commands. They take
 * the queue address the controller should use and nothing else, so a caller
 * resolves that through whichever translator its backing provides -- a
 * hostmem_heap physical address, a dmamem IOVA, or a GPU heap address -- and the
 * command layout stays in one place.
 *
 * @file nvme_command.h
 * @version 0.7.0
 */

struct nvme_completion {
	uint32_t cdw0;
	uint32_t rsvd;
	uint16_t sqhd;
	uint16_t sqid;
	uint16_t cid;
	uint16_t status;
};

struct nvme_command {
	uint8_t opc;
	uint8_t fuse;
	uint16_t cid;
	uint32_t nsid;
	uint64_t rsvd2;
	uint64_t mptr;
	uint64_t prp1;
	uint64_t prp2;
	uint32_t cdw10;
	uint32_t cdw11;
	uint32_t cdw12;
	uint32_t cdw13;
	uint32_t cdw14;
	uint32_t cdw15;
};

/**
 * Build a Create I/O Completion Queue command (opcode 0x5).
 *
 * The queue must be physically contiguous, which is what every uPCIe
 * backing produces: one allocation, one base address.
 *
 * @param cmd     Command to populate; zeroed first.
 * @param qid     Identifier to give the new completion queue.
 * @param depth   Queue depth in entries; programmed as the zero-based
 *                maximum, so a depth of 1 is the smallest legal value.
 * @param cq_addr Address of the completion queue as the controller sees
 *                it: an IOVA where a mapping was installed, a physical
 *                address where none was.
 */
static inline void
nvme_command_create_io_cq(struct nvme_command *cmd, uint16_t qid, uint16_t depth, uint64_t cq_addr)
{
	memset(cmd, 0, sizeof(*cmd));
	cmd->opc = 0x5;
	cmd->prp1 = cq_addr;
	cmd->cdw10 = ((uint32_t)(depth - 1) << 16) | qid;
	cmd->cdw11 = 0x1; ///< Physically contiguous, interrupts disabled
}

/**
 * Build a Create I/O Submission Queue command (opcode 0x1).
 *
 * The submission queue is bound to the completion queue of the same
 * identifier, which is the pairing every uPCIe caller uses; create that
 * completion queue first or the controller rejects this with
 * SCT=1/SC=0x0, "Completion Queue Invalid".
 *
 * @param cmd     Command to populate; zeroed first.
 * @param qid     Identifier to give the new submission queue, and the
 *                completion queue it reports into.
 * @param depth   Queue depth in entries; see nvme_command_create_io_cq.
 * @param sq_addr Address of the submission queue as the controller sees
 *                it; see nvme_command_create_io_cq.
 */
static inline void
nvme_command_create_io_sq(struct nvme_command *cmd, uint16_t qid, uint16_t depth, uint64_t sq_addr)
{
	memset(cmd, 0, sizeof(*cmd));
	cmd->opc = 0x1;
	cmd->prp1 = sq_addr;
	cmd->cdw10 = ((uint32_t)(depth - 1) << 16) | qid;
	cmd->cdw11 = ((uint32_t)qid << 16) | 0x1; ///< CQID | physically contiguous
}

/**
 * Build a Delete I/O Submission Queue command (opcode 0x0).
 *
 * Delete the submission queue before its completion queue; the reverse
 * order is rejected while the completion queue still has a submission
 * queue bound to it.
 *
 * @param cmd Command to populate; zeroed first.
 * @param qid Identifier of the submission queue to delete.
 */
static inline void
nvme_command_delete_io_sq(struct nvme_command *cmd, uint16_t qid)
{
	memset(cmd, 0, sizeof(*cmd));
	cmd->opc = 0x0;
	cmd->cdw10 = qid;
}

/**
 * Build a Delete I/O Completion Queue command (opcode 0x4).
 *
 * @param cmd Command to populate; zeroed first.
 * @param qid Identifier of the completion queue to delete.
 */
static inline void
nvme_command_delete_io_cq(struct nvme_command *cmd, uint16_t qid)
{
	memset(cmd, 0, sizeof(*cmd));
	cmd->opc = 0x4;
	cmd->cdw10 = qid;
}
