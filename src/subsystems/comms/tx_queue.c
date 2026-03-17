/* ---- Includes ---- */

#include "tx_queue.h"
#include <string.h>


/* ---- Module-level state ---- */

typedef struct {
    TxQueueEntry_t entries[TX_QUEUE_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} CommsTxQueue_t;

static CommsTxQueue_t queue = {0};


/* ---- Public function definitions ---- */

int txq_is_empty(void)
{
    return queue.count == 0;
}

int txq_is_full(void)
{
    return queue.count == TX_QUEUE_CAPACITY;
}

int txq_enqueue(const uint8_t *data, uint8_t length, uint8_t stop_and_wait, uint8_t is_ack)
{
    if (txq_is_full()) return -1;

    TxQueueEntry_t *e = &queue.entries[queue.tail];
    memcpy(e->data, data, length);
    e->length        = length;
    e->tries         = 0;
    e->stop_and_wait = stop_and_wait;
    e->is_ack        = is_ack;

    queue.tail = (uint8_t)((queue.tail + 1) % TX_QUEUE_CAPACITY);
    queue.count++;
    return 0;
}

TxQueueEntry_t *txq_peek(void)
{
    if (txq_is_empty()) return NULL;
    return &queue.entries[queue.head];
}

int txq_dequeue(void)
{
    if (txq_is_empty()) return -1;

    memset(&queue.entries[queue.head], 0, sizeof(TxQueueEntry_t));
    queue.head = (uint8_t)((queue.head + 1) % TX_QUEUE_CAPACITY);
    queue.count--;
    return 0;
}
