#pragma once

#include <stdbool.h>

#define T_FUNCTION(tboard, fn) { terror err = fn; if(!transportino_is_error_null(err)) { if(err.crash) {transportino_panic(tboard, fn);} else {transportino_error(tboard, fn);} } }

typedef enum {
    TRANSPORTINO_LED = 1,
    TRANSPORTINO_BUZZER,
    TRANSPORTINO_ICM20689,
    TRANSPORTINO_MOTORDRV,
    TRANSPORTINO_MICRO_ROS
} tcategory;

typedef enum {
    TRANSPORTINO_OK = 1,
    TRANSPORTINO_INVALID_CONFIG,
    TRANSPORTINO_WRONG_PARAMS,
    TRANSPORTINO_INIT_ERROR,
    TRANSPORTINO_INTERNAL_ERROR
} tstatus;

typedef struct _terror {
    tcategory category;
    tstatus status;
    bool crash;
} terror;

const static terror NULL_ERROR = {.category = 0, .status = 0, .crash = false};

bool transportino_is_error_null(terror);

terror terror_make(tcategory category, tstatus status, bool crash);

void transportino_panic(void* tboard, terror error);
void transportino_error(void* tboard, terror error);
