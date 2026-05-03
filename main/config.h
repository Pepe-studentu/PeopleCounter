#pragma once
#include <stdint.h>

/* --- WiFi --- */
#define WIFI_SSID               "ADDRESS"
#define WIFI_PASS               "PASSWORD"

/* --- TCP reporter --- */
#define SERVER_HOST             "192.168.1.XYZ"
#define SERVER_PORT             4444
#define TCP_RECONNECT_MS        3000

/* --- Sensor (AMG8833) --- */
#define AMG_I2C_PORT            I2C_NUM_0
#define AMG_I2C_SDA             21
#define AMG_I2C_SCL             22
#define AMG_I2C_ADDR            0x69
#define AMG_I2C_FREQ_HZ         400000
#define AMG_FRAME_RATE_HZ       10   /* 1 or 10 */

/* --- Detector --- */
#define TEMP_OFFSET             5.0f   /* °C above ambient to classify as hot */
#define MIN_HOT_PIXELS          4      /* minimum hot pixels for a valid blob */
#define PIXELS_PER_PERSON       10
#define MERGE_RATIO             1.6f
#define SPLIT_THRESHOLD         16     /* PIXELS_PER_PERSON * MERGE_RATIO */
#define AMBIENT_SAMPLE_N        10     /* mean of N coldest pixels for ambient */

/* --- Tracker --- */
#define MAX_TRACKS              8
#define MAX_BLOBS               8
#define MAX_TRAJ_LEN            100
#define MAX_MATCH_DIST          3.0f   /* pixels; Euclidean distance threshold */
#define ABSENT_FRAMES_MAX       3      /* frames absent before track is closed */

/* --- Classifier --- */
#define MIN_TRAJ_FRAMES         3
#define MIN_DISPLACEMENT        3.0f   /* minimum net travel along GOOD_AXIS */
#define MIN_R_SQUARED           0.6f
/*
 * GOOD_AXIS: axis perpendicular to the door.
 *   0 = row (Y axis)
 *   1 = col (X axis)
 *
 * IN_DIRECTION: sign of displacement counted as IN.
 *   +1 = increasing index (row 0 to 7) is IN
 *   -1 = decreasing index (row 7 to 0) is IN
 */
#define GOOD_AXIS               0
#define IN_DIRECTION            1
