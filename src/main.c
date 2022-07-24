#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include <icm20689pico/icm20689pico.h>
#include "transportino_board.h"


int main()
{

    stdio_init_all();

    tboard board;

    transportino_init(&board);

    while (1) {
        transportino_update(&board);
    }

    return 0;
}
