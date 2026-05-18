/**
 * @file obdh.h
 * @author Medir Segura medir.segura@estudiantat.upc.edu
 * @brief Resposible for the data management (housekeeping data, scientific data, and configurations) within the spacecraft.
 * Primary focus now is saving and retrieving data from flash.
 * 
 */

#ifndef INC_OBDH_H_
#define INC_OBDH_H_

#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/**
 * @brief Flash operation type.
 */
typedef enum {
    FLASH_READ = 0,  /**< Read data from flash into the destination buffer. */
    FLASH_WRITE = 1  /**< Write source buffer data to flash. */
} read_write;

/**
 * @brief Flash access request processed by the OBDH task.
 *
 * Requesters send this structure to obdh_queue_handle. The OBDH task performs
 * the selected operation, writes the HAL status through res when provided, and
 * notifies client when the operation completes.
 */
typedef struct {
    read_write op;       // Operació: llegir o escriure
    uint32_t addr;       // Adreça de la Flash
    size_t len;          // Longitud en bytes
    union {
        const uint8_t *src;  // FLASH_WRITE: dades a escriure (només lectura)
        uint8_t       *dst;  // FLASH_READ:  buffer a omplir
    } buf;
    TaskHandle_t client; // Tarea que demana l'operació (per notificar-la)
    HAL_StatusTypeDef *res; //Punter que retorna l'estatus de la escriptura/lectura
} obdh_request;

/** @brief Queue used to send flash access requests to the OBDH task. */
extern QueueHandle_t obdh_queue_handle;

/**
 * @brief OBDH FreeRTOS task entry point.
 * @param pv_parameters Task parameter provided by xTaskCreate(); currently unused.
 */
void obdh_task(void *pv_parameters);

#endif /* INC_OBDH_H_ */
