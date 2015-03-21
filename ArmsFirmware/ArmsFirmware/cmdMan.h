/*
 * cmdMan.h
 *
 * Created: 18/03/2015 07:08:11 p.m.
 *  Author: Yisus
 */
#ifndef _SERIAL_HEADER
#define _SERIAL_HEADER

#ifdef __cplusplus
extern "C" {
	#endif


	void serial_initialize(long ubrr);
	void serial_write( unsigned char *pData, int numbyte );
	unsigned char serial_read( unsigned char *pData, int numbyte );
	int serial_get_qstate(void);

	#ifdef __cplusplus
}
#endif

#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef ARMSFIMRWARE_H
#define ARMSFIMRWARE_H
#include "ArmsFirmware.h"
#endif

void serial_initialize(long ubrr);
int fetchCommand();
int fetchParam(unsigned char *cc, float *param);

int execCommand();
void ActualizeActualCommand(char cmd, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8);

void SendCmdResponse( char cmdId, bool success, char* msg);
void PrintDebugMsg( char* msg);

void showHelp();
int is_digit(unsigned char c);
int is_alpha(unsigned char  c);

void serial_write( unsigned char *pData, int numbyte );
unsigned char serial_read( unsigned char *pData, int numbyte );
int serial_get_qstate(void);
void serial_put_queue( unsigned char data );
unsigned char serial_get_queue(void);
int std_putchar(char c, FILE* file);
int std_getchar(FILE* file);