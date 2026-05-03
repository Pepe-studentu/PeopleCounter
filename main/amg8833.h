#pragma once
#include <stdbool.h>
#include "driver/i2c.h"

/*
 * Initialise the AMG8833 on the given I2C port.
 * Wire.begin(), full reset, status clear, normal mode, frame-rate.
 * Returns false if the sensor does not ACK on the bus.
 */
bool amg_init(i2c_port_t port);

/*
 * Read all 64 pixels into out[].
 * Values are degrees Celsius (12-bit signed raw × 0.25).
 * Returns false on I2C failure.
 */
bool amg_read_pixels(i2c_port_t port, float out[64]);
