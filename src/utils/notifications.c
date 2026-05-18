#include "notifications.h"

uint32_t wait_for_notification(void) {

    uint32_t notificationValue = 0;
    xTaskNotifyWait( 0,          // don’t clear on entry
                    0xFFFFFFFF,  // clear all bits on exit
                    &notificationValue,
                    0 );
    return notificationValue;

}