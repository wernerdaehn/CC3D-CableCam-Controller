#include "stm32f4xx_hal.h"

uint16_t mod16(int16_t number, int16_t base)
{
    number = number % base;
    if (number < 0)
    {
        return base + number;
    }
    else
    {
        return number;
    }
}


