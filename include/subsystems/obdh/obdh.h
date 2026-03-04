/**
 * @file obdh.h
 * @author Medir Segura medir.segura@estudiantat.upc.edu
 * @brief 
 * @version 0.1
 * @date 2026-03-4
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef INC_OBDH_H_
#define INC_OBDH_H_
/* --- Includes obligatoris --- */
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* ---- Type definitions ---- */
typedef enum {
    FLASH_READ = 0,
    FLASH_WRITE = 1
} read_write;

typedef struct {
    read_write op;       // Operació: llegir o escriure
    uint32_t addr;       // Adreça de la Flash
    size_t len;          // Longitud en bytes
    uint8_t *buf;        // Punter al buffer de dades
    TaskHandle_t client; // Tarea que demana l'operació (per notificar-la)
    HAL_StatusTypeDef *res; //Punter que retorna l'estatus de la escriptura/lectura
} obdh_request;

/* ---- Module-level variables (Exposed) ---- */
extern QueueHandle_t obdh_queue_handle;

/* ---- Function Prototypes ---- */
void obdh_task(void *pv_parameters);


/**
 * @brief Communications task function, it runs the OBC state machine.
 */
void obdh_task(void *pv_parameters);

#endif /* INC_OBDH_H_ */