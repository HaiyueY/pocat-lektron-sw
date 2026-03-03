/**
 * @file tx_queue.h
 * @brief TX packet queue for the COMMS subsystem.
 *
 * Provides a fixed-capacity FIFO of pending outgoing packets.
 * Each entry carries the raw packet bytes plus per-entry metadata.
 * The queue is a module-level singleton owned by tx_queue.c.
 */

#pragma once

#include <stdint.h>
#include "comms.h"  /* COMMS_PKT_SIZE */

#define TX_QUEUE_CAPACITY 8

/**
 * @brief One entry in the TX queue.
 */
typedef struct {
    uint8_t data[COMMS_PKT_SIZE]; /**< Raw packet bytes (pre-interleaving). */
    uint8_t length;               /**< Packet length in bytes. */
    uint8_t tries;                /**< Number of transmission attempts so far. */
    uint8_t stop_and_wait;        /**< If 1, send only this packet then wait for ACK. */
} TxQueueEntry_t;

/**
 * @brief Enqueue a packet.
 * @param data          Pointer to packet bytes to copy in.
 * @param length        Length of the packet in bytes (must be <= COMMS_PKT_SIZE).
 * @param stop_and_wait Set to 1 if the transmit state should pause after this entry.
 * @return 0 on success, -1 if the queue is full.
 */
int txq_enqueue(const uint8_t *data, uint8_t length, uint8_t stop_and_wait);

/**
 * @brief Return a pointer to the head entry without removing it.
 * @return Pointer to the head TxQueueEntry_t, or NULL if the queue is empty.
 */
TxQueueEntry_t *txq_peek(void);

/**
 * @brief Remove the head entry from the queue.
 * @return 0 on success, -1 if the queue is already empty.
 */
int txq_dequeue(void);

/**
 * @brief Check whether the queue is empty.
 * @return 1 if empty, 0 otherwise.
 */
int txq_is_empty(void);

/**
 * @brief Check whether the queue is full.
 * @return 1 if full, 0 otherwise.
 */
int txq_is_full(void);
