/*
* cmdMan.c
*
* Created: 18/03/2015 02:56:22 p.m.
*  Author: Yisus & Maurishito
*/

#include "cmdMan.h"

#define BUFFER_SIZE	128
#define DEFAULT_BAUDRATE 34						// 57132(57600)bps
#define DIR_RXD PORTE &= ~0x04, PORTE |= 0x08
#define ENDOFLINE "\r\n"
typedef struct sCommand
{
	char commandID;
	float param1;
	float param2;
	float param3;
	float param4;
	float param5;
	float param6;
	float param7;
	float param8;
} command;

volatile unsigned char buffer[BUFFER_SIZE] = {0};
volatile unsigned char bufferHead = 0;
volatile unsigned char bufferTail = 0;
static FILE *device;


bool debugMode = true;
int cmdReady = 0;
command actualCommand;

//Interrupt service routine
//Gets a char from serial port and stores it in a buffer
//If char == ENTER, puts a string end char and sets the cmdReady flag
SIGNAL(USART1_RX_vect)
{
	static unsigned char next = 0;
	
	buffer[next] = UDR1;
	
	if(	is_alpha(buffer[next]) || is_digit(buffer[next]) || buffer[next]=='-' ||
		buffer[next]==' ' || buffer[next]=='\t' || buffer[next]=='.' )
	{
		next++;
		if( next >= BUFFER_SIZE)
			next = 0;

		return;
	}
	else if((buffer[next] == '\r') || (buffer[next] == '\n'))
	{
		buffer[next] = 0;

		if(next != 0)
			cmdReady = 1;

		next = 0;
	}
}

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
	UBRR1H = (unsigned char)(baud >> 8);
	UBRR1L = (unsigned char)(baud & 0xFF);
	DIR_RXD;
	device = fdevopen( std_putchar, std_getchar );
}

bool fetchCommand()
{
	
}

// expects a command un buffer of the form [cmdId ... par1 ... par2 ... parN ... ]
int fetchCommand_OLD()
{
	unsigned int cc;
	float p1 = 0;
	float p2 = 0;
	float p3 = 0;
	float p4 = 0;
	float p5 = 0;
	float p6 = 0;
	float p7 = 0;
	float p8 = 0;
	
	
	// Considering that command ID is buffer[0]
	cc = 1;
	while(is_alpha(buffer[cc]))
		++cc;
	
	//Sin parámetros
	if((buffer[0] == 'z')||(buffer[0] == 'g')||(buffer[0] == 's')||(buffer[0] == 'h')||(buffer[0] == 'i') || (buffer[0] == 'S') || (buffer[0] == 'v'))
	{
		cc = 1;
		while(is_alpha(buffer[cc]))
			++cc;
			
		if(buffer[1] != 0)
			return 0;
		
		ActualizeActualCommand(buffer[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		
		return 1;
	}
	//1 Parámetro
	else if((buffer[0] == 'k')||(buffer[0] == 'C')||(buffer[0] == 'L')||(buffer[0] == 'R')||(buffer[0] == 'G')||(buffer[0] == 't') ||(buffer[0] == 'o')||(buffer[0] == 'c')||(buffer[0] == 'I'))	//////// 1 PARÁMETRO
	{
		// Obtener primer parametro
				
		if(!fetchParam(&cc, &p1)) 
			return 0;
		
		if(buffer[cc] != 0) 
			return 0;				// Se espera fin de cadena
			
		ActualizeActualCommand(buffer[0], p1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
	}
	//2 Parámetros
	else if((buffer[0] == 'f')||(buffer[0] == 'd'))						/////////// 2 PARÁMETROS
	{
		if(!fetchParam(&cc, &p1)) 
			return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) 
			return 0;			// Obtengo segundo parametro
		if(buffer[cc] != 0) 
			return 0;				// Se espera fin de cadena
			
		ActualizeActualCommand(buffer[0], p1, p2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
		
	}
	//3 Parámetros
	else if((buffer[0] == 'n'))
	{		
		if(!fetchParam(&cc, &p1))
			return 0;			// Obtengo primer parametro
		
		if(!fetchParam(&cc, &p2))
			return 0;			// Obtengo segundo parametro
		
		if(!fetchParam(&cc, &p3))
			p3 = 0.0f;		// Obtengo tercer parametro si existe
		
		// Se espera fin de cadena
		if(buffer[cc] != 0) 
			return 0;				
		ActualizeActualCommand(buffer[0], p1, p2, p3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
		
	}
	//4 Parámetros
	else if((buffer[0] == 'x'))
	{
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;			// Obtengo segundo parámetro
		if(!fetchParam(&cc, &p3)) return 0;			// Obtengo tercer parametro
		if(!fetchParam(&cc, &p4)) p4 = 0.0f;		// Obtengo cuarto parametro si existe
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		ActualizeActualCommand(buffer[0], p1, p2, p3, p4, 0.0f, 0.0f, 0.0f, 0.0f);
		return 1;
		
	}
	//7Parámetros
	else if((buffer[0] == 'x'))
	{
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;			// Obtengo segundo parametro
		if(!fetchParam(&cc, &p3)) return 0;			// Obtengo tercer parametro
		if(!fetchParam(&cc, &p4)) return 0;			// Obtengo cuarto parametro
		if(!fetchParam(&cc, &p5)) return 0;			// Obtengo quinto parametro
		if(!fetchParam(&cc, &p6)) return 0;			// Obtengo sexto parametro
		if(!fetchParam(&cc, &p7)) p7 = 0.0f;		// Obtengo séptimo parametro si existe
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		ActualizeActualCommand(buffer[0], p1, p2, p3, p4, p5, p6, p7, 0.0f);
		return 1;
		
	}
	//8 Parámetros
	else if((buffer[0] == 'p')||(buffer[0] == 'q'))
	{
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;			// Obtengo segundo parametro
		if(!fetchParam(&cc, &p3)) return 0;			// Obtengo tercer parametro
		if(!fetchParam(&cc, &p4)) return 0;			// Obtengo cuarto parametro
		if(!fetchParam(&cc, &p5)) return 0;			// Obtengo quinto parametro
		if(!fetchParam(&cc, &p6)) return 0;			// Obtengo sexto parametro
		if(!fetchParam(&cc, &p7)) return 0;			// Obtengo sexto parametro
		if(!fetchParam(&cc, &p8)) p8 = 0.0f;		// Obtengo séptimo parametro si existe
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		ActualizeActualCommand(buffer[0], p1, p2, p3, p4, p5, p6, p7,p8);
		return 1;
		
	}
	return 0;
}

int fetchParam_OLD(unsigned char *cc, float *param)
{
	char *bcc, c;
	int pointCount = 0;	
	
	// Se espera espacio
	if((buffer[*cc] != ' ') && (buffer[*cc] != '\t'))				
		return 0;		

	// Quitar todos los espacios
	while((buffer[*cc] == ' ') || (buffer[*cc] == '\t'))			
		++(*cc);
	
	// Guardo posicion de inicio de primer parametro
	bcc = (char*)(buffer + *cc);												
	
	// Se espera - o numero
	if((buffer[*cc] != '-') && !is_digit(buffer[*cc]))				
		return 0;	
	
	++(*cc);
	
	// Quitar numeros y punto decimal
	while(is_digit(buffer[*cc]) || (buffer[*cc] == '.'))			
	{
		if(buffer[*cc] == '.')
			++pointCount;											// Cuento los puntos decimales
		++(*cc);
	}

	if(pointCount > 1)												// Solo se acepta un punto decimal
		return 0;							
	
	// Respaldo Caracter
	c = buffer[*cc];
	
	// Simulo fin de cadena
	buffer[*cc] = 0;												
	*param = (float)strtod(bcc,0);
	buffer[*cc] = c;												// Recupero caracter
	return 1;
}

void ActualizeActualCommand(char cmd, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8)
{
	actualCommand.commandID = cmd;
	actualCommand.param1 = p1;
	actualCommand.param2 = p2;
	actualCommand.param3 = p3;
	actualCommand.param4 = p4;
	actualCommand.param5 = p5;
	actualCommand.param6 = p6;
	actualCommand.param7 = p7;
	actualCommand.param8 = p8;
}

void SendCmdResponse( char cmdId, bool success, char* msg)
{
	printf("%c %c %s %s", cmdId, success, msg, ENDOFLINE);
}

void PrintDebugMsg( char* msg)
{
	if( debugMode )
		printf("%s%s", msg, ENDOFLINE ); 
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
	printf("\t x value[4]\t\t Sets desired OPEN position of the robot hand motors [percentage of max opening]\r\n");
	printf("\t t 1/0 \tEnables/disables torque in all servos\r\n");
	printf("\t v \tGet Voltage on Servo 0\r\n");
	printf("\r\nDON'T SEND ANY OTHER COMMAND BECAUSE THIS IS A BETA VERSION\r\n");
	printf("NOT ATTENDING THIS WARNING MAY RESULT IN ROBOT DAMAGE OR MALFUNCTION\r\n");
	printf("THERE IS NO WARRANTY IN ELECTRICAL PARTS\n\n");
}

int is_alpha(unsigned char c)
{
	if(((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')))
		return 1;
	else 
		return 0;
}

int is_digit(unsigned char c)
{
	if((c >= '0') && (c <= '9')) 
		return 1;
	else
		return 0;
}

////////////////////////////////////////////////////////

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

int std_putchar(char c, FILE* file)
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

int std_getchar(FILE* file)
{
	char rx;
	
	while( serial_get_qstate() == 0 );
	rx = serial_get_queue();
	
	if( rx == '\r' )
	rx = '\n';

	return rx;
}





