/*
 * ArmsFirmware.h
 *
 * Created: 19/03/2015 02:51:41 p.m.
 *  Author: Yisus
 */ 


#define F_CPU 16000000UL // this is actualy not necesary

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <util/delay.h>

#include "dynamixel.h"

#ifndef CMDMAN_H
#define CMDMAN_H
#include "cmdMan.h"
#endif
