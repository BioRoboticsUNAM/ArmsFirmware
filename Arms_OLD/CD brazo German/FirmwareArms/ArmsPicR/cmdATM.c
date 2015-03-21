#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "cmdATM.h"
//#include "dynamixel.h"
//#include "servoControl.c"
#include "Calc.c"
//#include "kinematics.c"

#define BUFFER_SIZE	128
#define DEFAULT_BAUDRATE	34 // 57132(57600)bps

#define DIR_RXD 	PORTE &= ~0x04, PORTE |= 0x08

volatile unsigned char buffer[BUFFER_SIZE] = {0};
volatile unsigned char bufferHead = 0;
volatile unsigned char bufferTail = 0;
static FILE *device;
int cmdReady = 0;
float Voltage = 0;
//int object = 0;

typedef struct sCommand
{
	char cmd;
	float param1;
	float param2;
	float param3;
	float param4;
	float param5;
	float param6;
	float param7;
	float param8;
} command;

command comando;


void serial_put_queue( unsigned char data );
unsigned char serial_get_queue(void);
int std_putchar(char c);
int std_getchar(void);

int fetchCommand();

int fetchParam(char *cc, float *param);
void showHelp();
int execCommand();
void cmdX(char cmd, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8);
int is_digit(int c);
int is_alpha(int c);
int is_alnum(int c);


void serial_initialize(long ubrr)
{
	// Serial communication using UART1
	int baud = (unsigned short)(2000000.0 / ubrr) - 1;

	// set UART register A
	//Bit 7: USART Receive Complete
	//Bit 6: USART Transmit Complete
	//Bit 5: USART Data Resigter Empty 
	//Bit 4: Frame Error
	//Bit 3: Data OverRun
	//Bit 2: Parity Error
	//Bit 1: Double The USART Transmission Speed
	//Bit 0: Multi-Processor Communication Mode
	UCSR1A = 0b01000010;
	
	// set UART register B
	// bit7: enable rx interrupt
    // bit6: enable tx interrupt
    // bit4: enable rx
    // bit3: enable tx
    // bit2: set sendding size(0 = 8bit)
	UCSR1B = 0b10011000;
	
	// set UART register C
	// bit6: communication mode (1 = synchronize, 0 = asynchronize)
    // bit5,bit4: parity bit(00 = no parity) 
    // bit3: stop bit(0 = stop bit 1, 1 = stop bit 2)
    // bit2,bit1: data size(11 = 8bit)
	UCSR1C = 0b00000110;

	// initialize
	UDR1 = 0xFF;
	bufferHead = 0;
	bufferTail = 0;

	// set baudrate
	UBRR1H = (unsigned char)(baud>>8);
	UBRR1L = (unsigned char)(baud & 0xFF);
	DIR_RXD;
	device = fdevopen( std_putchar, std_getchar );
}


//
//Interrupt service routine
//Gets a char from serial port and stores it in a buffer
//If char == ENTER, puts a string end char and sets the cmdReady flag
//
SIGNAL(USART1_RX_vect)
{

static char next = 0;
	
	//buffer[next] = getc();
	buffer[next] = UDR1;
		
	if(
		((buffer[next] >= 'a') && (buffer[next] <= 'z')) ||
		((buffer[next] >= '0') && (buffer[next] <= '9')) ||
		(buffer[next] == '-') || (buffer[next] == ' ') ||
		(buffer[next] == '\t') || (buffer[next] == '.')
		)
	{
		++next;
		if (next >= BUFFER_SIZE) next = 0;
		return;
	}
	else if((buffer[next] >= 'A') && (buffer[next] <= 'Z'))
	{
	//	buffer[next]+= 32;		//Conversion a minuscula
		++next;
		if (next >= BUFFER_SIZE) next = 0;
		return;
	}
	else if((buffer[next] == '\r') || (buffer[next] == '\n'))//0x0D)
	{
		//putc(0x0A);
		buffer[next] = 0;
		//next++;
		//buffer[next] = 0;
		if(next != 0)
		{
			cmdReady = 1;
			next = 0;
		}
	}
	

	//serial_put_queue( UDR1 );
	//std_putchar('i');
	//printf("\n\rinterrupcion\n\r");
	//unsigned char a = serial_get_queue();
	//printf("dato: %c \n\r",a);
	

	
}





void showHelp()
{
	printf("\r\n\nRIGHT ARM FIRMWARE. Last modification: March 11th, 2014\r\n");
	printf("\r\nCommands:\r\n");
	printf("\t c value\tCloses gripper. Value represents a percentage of the maximum torque\r\n");
	printf("\t d ID value\tSets desired position [value in bits] of servo ID\r\n");
	printf("\t g \t\tGets the current arm position in articular coordinates [rads]\r\n");
	printf("\t G 1/0\tEnables/disables continuous cartesian position printing\r\n");
	printf("\t h \t\tShows help (If you are seeing this, I assume you knew this command\r\n");
	printf("\t k ID\tGets the current position [bits] of servo ID\r\n");
	printf("\t o value\tOpens gripper. Value represents a percentage of maximun opening angle.\n\t\t\t\t100 = maximum angle, 0 = completely closed\r\n");
	printf("\t q value[7] time\tSets desired position [rads] and time [seconds] (8 values needed)\r\n");
	printf("\t S \t\t Returns the main error states of all servos\r\n");
	printf("\t t 1/0 \tEnables/disables torque in all servos\r\n");
	printf("\t v \tGet Voltage on Servo 0\r\n");
	printf("\r\nDON'T SEND ANY OTHER COMMAND BECAUSE THIS IS A BETA VERSION\r\n");
	printf("NOT ATTENDING THIS WARNING MAY RESULT IN ROBOT DAMAGE OR MALFUNCTION\r\n");
	printf("THERE IS NO WARRANTY IN ELECTRICAL PARTS\n\n");
}

int execCommand()
{ 	
	float pos,f1;
	int d1,d2;
	char str[10];
	float servoPos[7];

	float test=0;
	int succes;
	succes = 0;
	//float f1, f2, f3;
	if(!cmdReady) return 0;
	cmdReady = 0;
	
	if(!fetchCommand()) return 0;

	//printf("\n Comando: %s  , cmdReady:%i",buffer,cmdReady);
	
	switch(comando.cmd)
	{
		case 'C':	// Control Reading ON/OFF
			
			object = ObjectInHand();
			printf("C %d\r",object);
			//if(comando.param1 == 1)
			//{
			//	if(doReading == 1)
			//	{
			//		doControl = 1;
			//		printf("C 1\r");
			//		//PORTC &= 0b1101111;
			//	}else
			//	{
			//		printf("C 0\r");
			//	}

				
			//}else
			//{
			//	doControl = 0;
			//	printf("C 0\r");
			//	//PORTC |= 0b0010000;
			//}
			break;
		case 'c':	//  Open gripper
			gripperCounter = 0;
			CloseGripper(comando.param1);
			
			break;
		
		case 'd':	// Set Arm Position
			/*servoPos[0] = comando.param1;
			servoPos[1] = comando.param2;
			servoPos[2] = comando.param3;
			servoPos[3] = comando.param4;
			servoPos[4] = comando.param5;
			servoPos[5] = comando.param6;
			servoPos[6] = comando.param7;
			ArmPosition(servoPos);*/
			printf("Id %f = %d \r",comando.param1,(int)comando.param2);
			dxl_write_word( comando.param1, SERVO_GOAL_POSITION_L, (int)comando.param2);
			break;
		
		case 'f':	// Set Arm Torque 
			//servoPos[0] = comando.param1;
			//servoPos[1] = comando.param2;
			//servoPos[2] = comando.param3;
			//ArmTorque(servoPos);
			SetTorque2(comando.param1,comando.param2);
			break;

			
		
		case 'g':	
			//Gets the arm position in cartesian coordinates
			if(doReading == 1 && doLoop ==1)
			{
				updateQPosition();
				printf("g 1 %f %f %f %f %f %f %f \r",currentQPosition[0],currentQPosition[1],currentQPosition[2],currentQPosition[3],currentQPosition[4],currentQPosition[5],currentQPosition[6]);
			}else
			{
				printf("g 0\r");
			}
			break;
			
		case 'G':	// 
			if(comando.param1 ==1)
			{
				printPosLoop = comando.param1;
			}
			else
			{
				printPosLoop = comando.param1;
				printf("G %d\r",printPosLoop);
			}
			break;
			
		case 'h':	//Shows help;
			showHelp();
			//doControl = 0;
			break;
			
		case 'I':	// Sets Desired Position of Control Loop
			if(comando.param1==1)
			{
				printStatus = 1;
				printf("I 1\r");
			}else{
				printStatus = 0;
				printf("I 0\r");
			}
			break;
			
		case 'i':	// Sets Desired Position of Control Loop
			ArmPing();
			break;
			
		case 'k':	// Sets Desired Position of Control Loop
			//ForwardKinematics();
			//CalculateHT();
			//TestMult();
			test = dxl_read_word( (int)comando.param1, SERVO_PRESENT_POSITION_L );
			printf("pos %f = %f \r",comando.param1,test);
			break;
			
		case 'L':	// Control Loop ON/OFF
			if(comando.param1 == 1)
			{
				doLoop = 1;
				printf("L 1\r");
				//PORTC &= 0b0111111;
			}else
			{
				doLoop = 0;
				printf("L 0\r");
				//PORTC |= 0b1000000;
			}
			break;
		
		case 'o':	//  Open gripper
			OpenGripper(comando.param1);
			printf("o 1\r");
			break;
		
			
		case 'p':	// Sets Desired Position of Control Loop
			//instructionSet = 1;
			//succes = SetdesiredPosition(comando.param1,comando.param2,comando.param3,comando.param4,comando.param5,comando.param6,comando.param7,comando.param8);
			//if(!succes)
			//{
			//	printf("p 0 \r");
			//	instructionSet = 0;
			//}
			printf("Unimplemented command\r\n");
			break;
	
		case 'q':	// Sets Desired Position of Control Loop
			instructionSet = 2;
			succes = SetdesiredPosition(comando.param1,comando.param2,comando.param3,comando.param4,comando.param5,comando.param6,comando.param7,comando.param8);
			if(!succes)
			{
				printf("q 0 \r");
				instructionSet = 0;
			}
			break;
			
		case 'R':	// Control Reading ON/OFF
			if(comando.param1 == 1)
			{
				doReading = 1;
				printf("R 1\r");
				//PORTC &= 0b1011111;
			}else
			{
				if(doControl == 0)
				{
					doReading = 0;
					printf("R 0\r");
					//PORTC |= 0b0100000;
				}else
				{
					printf("R 1\r");
				}
			}
			break;

		case 's':	// Set Speed of desired Servo
			SetSpeedPerc(comando.param1,comando.param2);
			break;
		

		
		case 'S':	// Get Status Report of every servo
			StatusReport();
			break;
		
				
		case 't':	// Enables Arm Torque Control 
			if(comando.param1 ==1)
			{
				
				ArmTorqueEnabled(comando.param1);
				printf("t 1\r");

			}else
			{
				//SetdesiredQPosition(0,0,0,0,0,0,0);
				ArmTorqueEnabled(0);
				printf("t 1\r");
			}
			break;
		
		case 'v':	// Gets Voltage
			Voltage=GetVoltage(0);
			printf("v 1 %4.1f\r",Voltage);
			break;

		
		case 'y':	// Enables Arm Torque Control 
			if(ServoTorqueEnabled(comando.param1,comando.param2,comando.param3,comando.param4,comando.param5,comando.param6,comando.param7)==1)
			{
				printf("y 1\r");
			}
			else
			{
				printf("y 0\r");
			}
			break;
        case 'x':
		    ChangeBaud();
			break;
		
		
		default:
			return 0;
	}
	return 1;
}


int fetchCommand()
{
	char cc;
	float p1 = 0;
	float p2 = 0;
	float p3 = 0;
	float p4 = 0;
	float p5 = 0;
	float p6 = 0;
	float p7 = 0;
	float p8 = 0;
	
	//Sin parámetros
	if((buffer[0] == 'x')||(buffer[0] == 'g')||(buffer[0] == 's')||(buffer[0] == 'h')||(buffer[0] == 'i') || (buffer[0] == 'S') || (buffer[0] == 'v'))
	{
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(buffer[1] != 0) return 0; //Parametros no requeridos
		cmdX(buffer[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
	}
	//1 Parámetro
	else if((buffer[0] == 'k')||(buffer[0] == 'C')||(buffer[0] == 'L')||(buffer[0] == 'R')||(buffer[0] == 'G')||(buffer[0] == 't') ||(buffer[0] == 'o')||(buffer[0] == 'c')||(buffer[0] == 'I'))	//////// 1 PARÁMETRO
	{
		// Obtener primer parametro
		
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		cmdX(buffer[0], p1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
	}
	//2 Parámetros
	else if((buffer[0] == 'f')||(buffer[0] == 'd'))						/////////// 2 PARÁMETROS
	{
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;			// Obtengo segundo parametro
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		cmdX(buffer[0], p1, p2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
		
	}
	//3 Parámetros
	else if((buffer[0] == 'n'))		
	{
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;			// Obtengo segundo parametro
		if(!fetchParam(&cc, &p3)) p3 = 0.0f;		// Obtengo tercer parametro si existe
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		cmdX(buffer[0], p1, p2, p3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
		
	}
	//4 Parámetros
	else if((buffer[0] == 'z'))		
	{
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;
		if(!fetchParam(&cc, &p3)) return 0;			// Obtengo segundo parametro
		if(!fetchParam(&cc, &p4)) p4 = 0.0f;		// Obtengo tercer parametro si existe
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		cmdX(buffer[0], p1, p2, p3, p4, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
		
	}
	//7Parámetros
	else if((buffer[0] == 'x'))
	{
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;			// Obtengo segundo parametro
		if(!fetchParam(&cc, &p3)) return 0;			// Obtengo tercer parametro
		if(!fetchParam(&cc, &p4)) return 0;			// Obtengo cuarto parametro
		if(!fetchParam(&cc, &p5)) return 0;			// Obtengo quinto parametro
		if(!fetchParam(&cc, &p6)) return 0;			// Obtengo sexto parametro
		if(!fetchParam(&cc, &p7)) p7 = 0.0f;		// Obtengo séptimo parametro si existe
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		cmdX(buffer[0], p1, p2, p3, p4, p5, p6, p7, 0.0f);
		return 1;
		
	}
	//8 Parámetros
	else if((buffer[0] == 'p')||(buffer[0] == 'q'))
	{
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;			// Obtengo segundo parametro
		if(!fetchParam(&cc, &p3)) return 0;			// Obtengo tercer parametro
		if(!fetchParam(&cc, &p4)) return 0;			// Obtengo cuarto parametro
		if(!fetchParam(&cc, &p5)) return 0;			// Obtengo quinto parametro
		if(!fetchParam(&cc, &p6)) return 0;			// Obtengo sexto parametro
		if(!fetchParam(&cc, &p7)) return 0;			// Obtengo sexto parametro
		if(!fetchParam(&cc, &p8)) p8 = 0.0f;		// Obtengo séptimo parametro si existe
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		cmdX(buffer[0], p1, p2, p3, p4, p5, p6, p7,p8);
		return 1;
		
	}
	return 0;
}


int is_alpha(int c)
{
	if(((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z'))) return 1;
	return 0;
}

void cmdX(char cmd, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8)
{
	comando.cmd = cmd;
	comando.param1 = p1;
	comando.param2 = p2;
	comando.param3 = p3;
	comando.param4 = p4;
	comando.param5 = p5;
	comando.param6 = p6;
	comando.param7 = p7;
	comando.param8 = p8;

}

int fetchParam(char *cc, float *param)
{
	char *bcc, c;
	int pointCount = 0;
	
	
	if((buffer[*cc] != ' ') && (buffer[*cc] != '\t')) return 0;		// Se espera espacio

	while((buffer[*cc] == ' ') || (buffer[*cc] == '\t')) ++(*cc);	// Como espacios
	bcc = buffer + *cc;												// Guardo posicion de inicio de primer parametro
	if((buffer[*cc] != '-') && !is_digit(buffer[*cc])) return 0;		// Se espera - o numero
	++(*cc);
	while(is_digit(buffer[*cc]) || (buffer[*cc] == '.'))				// Como numeros y punto decimal
	{
		if(buffer[*cc] == '.') ++pointCount;						// Cuento los puntos decimales
		 ++(*cc);
	}
	if(pointCount > 1) return 0;									// Solo se acepta un punto decimal
	c = buffer[*cc];												// Respaldo Caracter 
	buffer[*cc] = 0;												// Simulo fin de cadena
	//*param = (float)atof(bcc);												// Obtengo el parametro
	//sscanf(bcc, "%f", param);
	//printf("FLOAT: %f    ,bcc: %c",param, bcc);
	*param = (float)strtod(bcc,0);
	buffer[*cc] = c;												// Recupero caracter
	return 1;
}

int is_digit(int c)
{
	if((c >= '0') && (c <= '9')) return 1;
	return 0;
}

void serial_write( unsigned char *pData, int numbyte )
{
	int count;

	for( count=0; count<numbyte; count++ )
	{
		while(!bit_is_set(UCSR1A,5));
		UDR1 = pData[count];
	}
}

unsigned char serial_read( unsigned char *pData, int numbyte )
{
	int count, numgetbyte;
	
	if( bufferHead == bufferTail )
		return 0;
	
	numgetbyte = serial_get_qstate();
	if( numgetbyte > numbyte )
		numgetbyte = numbyte;
	
	for( count=0; count<numgetbyte; count++ )
		pData[count] = serial_get_queue();
	
	return numgetbyte;
}

int serial_get_qstate(void)
{
	short NumByte;
	
	if( bufferHead == bufferTail )
		NumByte = 0;
	else if( bufferHead < bufferTail )
		NumByte = bufferTail - bufferHead;
	else
		NumByte = BUFFER_SIZE - (bufferHead - bufferTail);
	
	return (int)NumByte;
}
void serial_put_queue( unsigned char data )
{

	if( serial_get_qstate() == (BUFFER_SIZE-1) )
		return;
	
	buffer[bufferTail] = data;

	if( bufferTail == (BUFFER_SIZE-1) )
		bufferTail = 0;
	else
		bufferTail++;

		
}

unsigned char serial_get_queue(void)
{
	unsigned char data;
	
	if( bufferHead == bufferTail )
		return 0xff;
		
	data = buffer[bufferHead];
		
	if( bufferHead == (BUFFER_SIZE-1) )
		bufferHead = 0;
	else
		bufferHead++;
		
	return data;
}

int std_putchar(char c)
{
	char tx[2];
	
    if( c == '\n' )
	{
        tx[0] = '\r';
		tx[1] = '\n';
		serial_write( (unsigned char*)tx, 2 );
	}
	else
	{
		tx[0] = c;
		serial_write( (unsigned char*)tx, 1 );
	}
 
    return 0;
}

int std_getchar(void)
{
    char rx;
	
	while( serial_get_qstate() == 0 );
	rx = serial_get_queue();
	
	if( rx == '\r' )
		rx = '\n';

    return rx;
}
