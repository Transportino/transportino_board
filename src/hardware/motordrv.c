#include "hardware/motordrv.h"
#include "hardware/motor.h"

#include "pico/stdlib.h"

#include <stdlib.h>

#include "transportino_config.h"

terror motordrv_init(motordrv* motor_driver, uint8_t enable_pin, motorcfg motors[], void* tboard)
{
    motor_driver->enable_pin = enable_pin;
    
    gpio_init(enable_pin);
    gpio_set_dir(enable_pin, GPIO_OUT);

    for (size_t i = 0; i < MOTORS_NUM; i++)
    {
         motor_init(
            &motor_driver->motors[i],
            &motors[i],
            tboard
        );
    }
    
    motordrv_power(motor_driver, true);

    return NULL_ERROR;
}

terror motordrv_power(motordrv* motordrv, bool power)
{
    motordrv->power = power;
    gpio_put(motordrv->enable_pin, power);
    return NULL_ERROR;
}

terror motordrv_get(motordrv* motordrv, uint8_t motor_index, motor** motor)
{
    if(motor_index >= MOTORS_NUM) {
        printf("TransportinoBoard > Motor index out of bounds.");
        return terror_make(TRANSPORTINO_MOTORDRV, TRANSPORTINO_WRONG_PARAMS, false);
    }

    *motor = &motordrv->motors[motor_index];
    return NULL_ERROR;
}

terror motordrv_get_by_enc_pin(motordrv* motordrv, uint8_t enc_pin, motor** mtr)
{
    bool found = false;

    for (size_t i = 0; i < MOTORS_NUM; i++)
    {
        motor* _motor = &motordrv->motors[i];
        
        if(_motor->enc_pin_a == enc_pin)
        {
            *mtr = _motor;
            found = true;
            break;
        }
    }

    if(!found)
    {
        printf("TransportinoBoard > Motor not found.");
        return terror_make(TRANSPORTINO_MOTORDRV, TRANSPORTINO_WRONG_PARAMS, false);
    }

    return NULL_ERROR;
}