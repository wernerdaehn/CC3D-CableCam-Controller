#include "serial_print.h"
#include "debug.h"
#include <stdio.h>
#include <stdarg.h>
#include "debug.h"
#include "protocol.h"

Endpoints debug_endpoint = EndPoint_USB;

void setDebugLevel(uint8_t level, Endpoints endpoint) {
	activesettings.debuglevel = level;
	debug_endpoint = endpoint;
}

uint8_t getDebugLevel() {
	return activesettings.debuglevel;
}

void printDebug_int(int v, uint8_t debuglevel) {
	if (activesettings.debuglevel >= debuglevel) {
		PrintSerial_int(v, debug_endpoint);
	}
}

void printDebug_char(char v, uint8_t debuglevel){
	if (activesettings.debuglevel >= debuglevel) {
		PrintSerial_char(v, debug_endpoint);
	}
}
void printDebug_string(char * v, uint8_t debuglevel){
	if (activesettings.debuglevel >= debuglevel) {
		PrintSerial_string(v, debug_endpoint);
	}
}
void printDebug_long(long v, uint8_t debuglevel){
	if (activesettings.debuglevel >= debuglevel) {
		PrintSerial_long(v, debug_endpoint);
	}
}
void printDebug_double(double v, uint8_t debuglevel){
	if (activesettings.debuglevel >= debuglevel) {
		PrintSerial_double(v, debug_endpoint);
	}
}


void printlnDebug_int(int v, uint8_t debuglevel) {
	if (activesettings.debuglevel >= debuglevel) {
		PrintlnSerial_int(v, debug_endpoint);
	}
}
void printlnDebug_char(char v, uint8_t debuglevel){
	if (activesettings.debuglevel >= debuglevel) {
		PrintlnSerial_char(v, debug_endpoint);
	}
}
void printlnDebug_string(char * v, uint8_t debuglevel){
	if (activesettings.debuglevel >= debuglevel) {
		PrintlnSerial_string(v, debug_endpoint);
	}
}
void printlnDebug_long(long v, uint8_t debuglevel){
	if (activesettings.debuglevel >= debuglevel) {
		PrintlnSerial_long(v, debug_endpoint);
	}
}
void printlnDebug_double(double v, uint8_t debuglevel){
	if (activesettings.debuglevel >= debuglevel) {
		PrintlnSerial_double(v, debug_endpoint);
	}
}
