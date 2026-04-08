/**
 * @file transceiver.h
 * @brief Interrupt-driven RF transceiver task.
 *
 * Handles all RadioLib hardware operations (RX/TX) driven by DIO1 hardware interrupts.
 * Communicates with comms_task via FreeRTOS queues (rx_queue, tx_queue) and task notifications.
 */

#pragma once

/**
 * @brief Transceiver task function.
 *
 * Runs in an infinite loop:
 * - Waits on task notifications (RADIO_IRQ_BIT from DIO1, TX_READY_BIT from comms_task)
 * - On RX_DONE: reads packet, deinterleaves, pushes to rx_queue
 * - On TX_READY: drains tx_queue, transmits each packet, waits for TX_DONE
 * - Always returns to RX mode when idle
 */
void transceiver_task(void *pv_parameters);
