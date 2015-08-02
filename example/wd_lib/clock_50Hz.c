
#include "clock_50Hz.h"

static uint32_t clockcounter = 0;

uint32_t getCounter() {
	return clockcounter;
}

void tickCounter() {
	clockcounter++;
}

uint8_t is1Hz() {
	return ((clockcounter % 50) == 0);
}

uint8_t is5Hz() {
	return ((clockcounter % 10) == 0);
}

uint8_t is5s() {
	return ((clockcounter % 500) == 0);
}
