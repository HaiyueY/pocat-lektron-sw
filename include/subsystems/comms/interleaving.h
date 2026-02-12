/**
 * @file interleaving.h
 * @brief Interleaving and deinterleaving functions for COMMS packets.
 */

#ifndef INC_INTERLEAVING_H_
#define INC_INTERLEAVING_H_

#include <stdint.h>

/**
 * @brief Interleaves an array by redistributing elements across 6 groups.
 *
 * @param inputarr Pointer to the array to interleave (modified in place).
 * @param size     Size of the array. Must be a multiple of 6.
 */
void Interleave(uint8_t *inputarr, int size);

/**
 * @brief Deinterleaves an array, reversing the Interleave operation.
 *
 * @param inputarr Pointer to the array to deinterleave (modified in place).
 * @param size     Size of the array. Must be a multiple of 6.
 */
void Deinterleave(uint8_t *inputarr, int size);

#endif /* INC_INTERLEAVING_H_ */
