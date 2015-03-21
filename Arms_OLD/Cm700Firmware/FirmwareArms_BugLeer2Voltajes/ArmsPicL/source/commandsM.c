#ifndef _COMMANDS
#define _COMMANDS

#ifndef RS232BAUD
#define RS232BAUD 19200
#endif
#ifndef RS232XMIT
#define RS232XMIT PIN_C6
#endif
#ifndef RS232RCV
#define RS232RCV PIN_C7
#endif

#priority rda

//#include "mov.c"
#include "mov60.c"
#include "accel.c"
#include <stdlib.h>
#include <stdio.h>

#use delay(clock=CLK)
// Conexión a puerto serie a 19200bps
#use rs232(baud=RS232BAUD, xmit=RS232XMIT, rcv=RS232RCV)

typedef struct sCommand
{
	char cmd;
	float param1;
	float param2;
	float param3;
} command;

#define BUFFER_SIZE 30
char buffer[BUFFER_SIZE];
command comando;
int1 cmdReady = 0;
int1 autoreport = 0;
int1 reported = 0;
int32 arTime;

void setupCommands();
int1 fetchCommand();
void commandsHelp();
int1 fetchParam(char *cc, float *param);
int1 execCommand();
void cmdX(char cmd, float p1, float p2, float p3);
int1 is_digit(int c);
int1 is_alpha(char c);
int1 is_alnum(char c);

void commandsHelp()
{
	printf("\fCommands \r\n");
	printf("\th\t\tShow help\r\n");
	printf("\ta\t\tShow Accelerometer values in G\r\n");
	printf("\tcl\t\tGet left  speed-control constants\r\n");
	printf("\tcr\t\tGet right speed-control constants\r\n");
	printf("\tcl val\t\tTest left  speed-control constants\r\n");
	printf("\tcr val\t\tTest right speed-control constants\r\n");
	printf("\tcl min ctr max\tSet left  speed-control constants\r\n");
	printf("\tcr min ctr max\tSet right speed-control constants\r\n");
	printf("\te\t\tShow encoders values used in last PID\r\n");
	printf("\tf\t\tShow encoders count since last f-command request\r\n");
	printf("\tm dst ang [t]\tSame as mv angle distance [time]\r\n");
	printf("\t\t\tangle in rads, distance in dm time ins secs\r\n");
	//printf("\tp\t\t\tGet Estimated position since last \r\n");
	printf("\tr\t\tReset Driver\r\n");
	printf("\ts\t\tStop motors\r\n");
	printf("\tu\t\tAutomatic report mode: automatic f 250ms after w\r\n");
	printf("\tw speedL speedR\tw leftAngularSpeed rightAngularSpeed\r\n");
	
}

int1 execCommand()
{
	float f1, f2, f3;
	if(!cmdReady) return 0;
	cmdReady = 0;
	if(!fetchCommand()) return 0;
	switch(comando.cmd)
	{
		case 'a':	//accelerometer
			readAccelf(&f1, &f2, &f3);
			printf("x%1.4fy%1.4fz%1.4f", f1, f2, f3);
			break;
		
		case 'e':	//encoders
			printf("l%4LXr%4LX", getLastEncoderL(), getLastEncoderR());		
			break;
			
		case 'f':	//encoders count
			//printf("l%8LXr%8LX", getEncoderCountL(), getEncoderCountR());
			printf("l%Ldr%Ld", getEncoderCountL(), getEncoderCountR());
			//printf("PWM:%dLR:%ld,%ld", speedL, getEncoderCountL(), getEncoderCountR());
			break;
		
		case 'h':	//detener
			commandsHelp();
			break;
			
		case 'm':	//comando mv
			setup_mv(comando.param1, comando.param2, comando.param3);
			//printf("mv%f,%f,%f", comando.param1, comando.param2,comando.param3);
			break;
			
		case 's':	//detener
			putc('s');
			stop();
			break;
		
		case 'o':	//posicion estimada
			setSD(comando.param1,comando.param2);
			printf("o%lu,%lu", shutDownL, shutDownR);
			break;
		
		case 'p':	//posicion estimada
			putc('p');
			break;
			
		case 'r':	//posicion estimada
			putc('r');
			setSpeed(0);
			disable_interrupts( GLOBAL );
			CCP_1 = CCP_RESET;
			CCP_2 = CCP_RESET;
			reset_cpu();
			break;
			
		case 'u':	//posicion estimada
			autoreport = !autoreport;
			reported = 0;
			printf("u %c", autoreport ? '1' : '0');
			break;
			
		case 'w':	//Velocidad
			//printf("w%2X,%2X", comando.param1, comando.param2);
			setLeftSpeed(comando.param1);
			setRightSpeed(comando.param2);
			arTime = globalCounter;
			reported = 0;
			//printf("w%f,%f", comando.param1, comando.param2);
			break;
		
		// Left Encoder Constants GET
		case ',':
			printf("cl %lu, %lu, %lu\r\n", CCP_MIN_L, CCP_CENTRO_L, CCP_MAX_L);
			break;
		
		// Left Encoder Constants SET
		case ';':
			//printf("LES %ld, %ld, %ld\r\n", (long)comando.param1, (long)comando.param2, (long)comando.param3);
			setConstantsL((long)comando.param1, (long)comando.param2, (long)comando.param3);
			break;			   	
			
		// Left Encoder Constants TEST
		case '<':
			printf("TEST_L %ld\r\n", (long)comando.param1);
			testConstantL((long)comando.param1);
			break;
		
		// Right Encoder Constants GET
		case '.':
			printf("cr %lu, %lu, %lu\r\n", CCP_MIN_R, CCP_CENTRO_R, CCP_MAX_R);
			break;
			
		// Right Encoder Constants SET
		case ':':
			//printf("RES %ld, %ld, %ld\r\n", (long)comando.param1, (long)comando.param2, (long)comando.param3);
			setConstantsR((long)comando.param1, (long)comando.param2, (long)comando.param3);
			break;
			
		// Right Encoder Constants TEST
		case '>':
			printf("TEST_R %ld\r\n", (long)comando.param1);
			testConstantR((long)comando.param1);
			break;
			
		default:
			return 0;
	}
	return 1;
}

int1 fetchCommand()
{
	char cc;
	float p1 = 0;
	float p2 = 0;
	float p3 = 0;
	//signed long p3 = 0;
	
	//printf("\r\nFetch string %s", cc);
	//printf("\r\nFetch %c", buffer[0]);

	if( (buffer[0] == 'a') || (buffer[0] == 'e') || (buffer[0] == 'f') || (buffer[0] == 'h') ||
		(buffer[0] == 'r') || (buffer[0] == 's') || (buffer[0] == 'p') || (buffer[0] == 'u'))
	{
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(buffer[1] != 0) return 0; //Parametros no requeridos
		cmdX(buffer[0], 0.0f, 0.0f, 0.0f);
		return 1;
	}
	else if((buffer[0] == 'w') || (buffer[0] == 'o'))
	{
		//printf("\r\nFetched %c", buffer[0]);
		//printf("\r\nFetching Param 1");
		// Obtener primer parametro
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;
		//printf("\r\nFetched Param 1: %ld", p1);
		//printf("\r\nOffset %u", cc);
		//printf("\r\nFetching Param 2");
		// Obtener segundo parametro
		if(!fetchParam(&cc, &p2)) return 0;
		//printf("\r\nFetched Param 2: %ld", p2);

		//printf("\r\nOffset %u", cc);
		//printf("\r\nFetching 0x%X", buffer[cc]);
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		//printf("\r\nFetched %c", buffer[cc]);
		cmdX(buffer[0], p1, p2, 0.0f);
		//printf("Fetch return 1");
		return 1;
	}
	else if((buffer[0] == 'm'))
	{
		cc = 1;
		while(is_alpha(buffer[cc]))++cc;
		if(!fetchParam(&cc, &p1)) return 0;			// Obtengo primer parametro
		if(!fetchParam(&cc, &p2)) return 0;			// Obtengo segundo parametro
		if(!fetchParam(&cc, &p3)) p3 = 0.0f;		// Obtengo tercer parametro si existe
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		cmdX(buffer[0], p1, p2, p3);
		return 1;
		
	}
	// Definicion de constantes de Encoders
	else if((buffer[0] == 'c'))
	{
		cc = 1;
		if((buffer[cc] != 'l') && (buffer[cc] != 'r')) return 0; 
		++cc;
		if(fetchParam(&cc, &p1))					// Obtengo primer parametro
		{
			if(fetchParam(&cc, &p2))				// Obtengo segundo parametro
			{
				if(!fetchParam(&cc, &p3))			// Obtengo tercer parametro
					return 0;
			}
			else p2 = -1;
		}
		else p1 = -1;
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
	
		if(p2 == -1)								// Modo TEST
			cmdX(buffer[1] == 'l' ? '<' : '>', p1, p2, p3);
		else if(p1 == -1)								// Modo GET
			cmdX(buffer[1] == 'l' ? ',' : '.', p1, p2, p3);
		else										// Modo SET
			cmdX(buffer[1] == 'l' ? ';' : ':', p1, p2, p3);
		return 1;
	}
	return 0;
}

int1 fetchParam(char *cc, float *param)
{
	char *bcc, c;
	int pointCount = 0;
	
	if((buffer[*cc] != ' ') && (buffer[*cc] != '\t')) return 0;		// Se espera espacio
	while((buffer[*cc] == ' ') || (buffer[*cc] == '\t')) ++(*cc);	// Como espacios
	bcc = buffer + *cc;												// Guardo posicion de inicio de primer parametro
	if((buffer[*cc] != '-') && !isDigit(buffer[*cc])) return 0;		// Se espera - o numero
	++(*cc);
	while(isDigit(buffer[*cc]) || (buffer[*cc] == '.'))				// Como numeros y punto decimal
	{
		if(buffer[*cc] == '.') ++pointCount;						// Cuento los puntos decimales
		 ++(*cc);
	}
	if(pointCount > 1) return 0;									// Solo se acepta un punto decimal
	c = buffer[*cc];												// Respaldo Caracter 
	buffer[*cc] = 0;												// Simulo fin de cadena
	*param = atof(bcc);												// Obtengo el parametro
	buffer[*cc] = c;												// Recupero caracter
	return 1;
}

void setupCommands()
{
	comando.cmd = 'h';
	comando.param1 = 0;
	comando.param2 = 0;
	comando.param3 = 0;
	
	enable_interrupts(INT_RDA);
}

void cmdX(char cmd, float p1, float p2, float p3)
{
	comando.cmd = cmd;
	comando.param1 = p1;
	comando.param2 = p2;
	comando.param3 = p3;
}

int1 is_digit(char c)
{
	if((c >= 'a') && (c <= 'z')) return 1;
	return 0;
}

int1 is_alpha(char c)
{
	if(
		((c >= 'a') && (c <= 'z')) ||
		((c >= 'A') && (c <= 'Z'))) return 1;
	return 0;
}

int1 is_alnum(char c)
{
	return isDigit(c) || isAlpha(c);
}

#INT_RDA
void rda_handle()
{
	
	static char next = 0;
	
	buffer[next] = getc();
	//putc(buffer[next]);
	
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
		buffer[next]+= 32;		//Conversion a minuscula
		++next;
		if (next >= BUFFER_SIZE) next = 0;
		return;
	}
	else if((buffer[next] == '\r') || (buffer[next] == '\n'))//0x0D)
	{
		putc(0x0A);
		buffer[next] = 0;
		//next++;
		//buffer[next] = 0;
		if(next != 0)
		{
			cmdReady = 1;
			next = 0;
		}
	}
	
}

#endif
