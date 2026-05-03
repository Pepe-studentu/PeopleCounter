#pragma once
#include <stdint.h>
#include "classifier.h"

/*
 * Initialise the reporter: creates the internal FreeRTOS queue and spawns
 * the TCP reporter task. Call once after WiFi is connected.
 */
void reporter_init(void);

/*
 * Enqueue an IN/OUT event for transmission. Non-blocking: if the queue is
 * full (server unreachable for a prolonged period), the event is dropped.
 */
void reporter_send(Classification event, int64_t timestamp_ms);
