#pragma once
#include "transportino_error.h"

typedef struct _temp_sensor {
    double temperature; // Readings are expressed in Celsius degrees
} temp_sensor;

terror temp_init(temp_sensor* temp_sensor);
terror temp_read(temp_sensor* temp_sensor);