#pragma once

#include <stdio.h>
#include "transportino_error.h"
#include "hardware/led.h"
#include "hardware/buzzer.h"
#include "hardware/motordrv.h"
#include "software/micro_ros.h"

#include <icm20689pico/icm20689pico.h>

typedef struct _board {
    bool restart;
    bool prog_mode;
    // Hardware components
    led* led;
    buzzer* buzz;
    motordrv* motordrv;
    icm20689* icm20689;
    // Software components
    micro_ros* micro_ros;
} tboard;

void transportino_init(tboard* tboard);
void transportino_restart(tboard* tboard, bool prog_mode);
void transportino_update(tboard* tboard);