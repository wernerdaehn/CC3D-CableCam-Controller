
#include "clock_50Hz.h"

static uint32_t clockcounter = 0;

uint32_t getCounter()
{
    return clockcounter;
}

/** \brief Whenever 20ms have passed, the main loop should invoke this function
 *
 * This function is called by the main loop every 20 ms and provides information to
 * all sub routines if a certain amount of time has passed.
 * This can be used for all slow timers that do not need precise timing values, e.g.
 * To write information once every second and the such.
 *
 * \return void
 *
 */
void tickCounter()
{
    clockcounter++;
}

/** \brief Returns True once every second
 *
 * \return uint8_t
 *
 */
uint8_t is1Hz()
{
    return ((clockcounter % 50) == 0);
}

/** \brief Returns True every 0.2 seconds
 *
 * \return uint8_t
 *
 */
uint8_t is5Hz()
{
    return ((clockcounter % 10) == 0);
}

/** \brief Returns True every five seconds
 *
 * \return uint8_t
 *
 */
uint8_t is5s()
{
    return ((clockcounter % 500) == 0);
}
