#include <stdlib.h>
#include <stdio.h>
#include "peripherals.c"
#include "Mathe.h"

float desiredPositionX = 0;
float desiredPositionY = 0;
float desiredAngle = 0;
float desiredDistance = 0;
float desiredLeftW = 0;
float desiredRightW = 0;
int16 counter_50ms = 0;    //Timer que lleva el tiempo de ejecución de un comando en múltiplos de 50 ms
int1 newGoalPoint=0; //Variable que indica la llegada de un nuevo comando 
int1 encodersCleared=0;
int1 goalPointReached=0;
int1 stop=0;
int1 moveBackward=0;

typedef struct sCommand
{
	char cmd;
	float param1;
	float param2;
	float param3;
	unsigned int8 paramInt1;
	unsigned int8 paramInt2;
} command;

#define BUFFER_SIZE 30
char buffer[BUFFER_SIZE];
command comando;
int1 cmdReady = 0;
int1 autoreport = 0;
int1 reported = 0;
//int32 arTime;

void setupCommands();
int1 fetchCommand();
void commandsHelp();
int1 fetchParam(char *cc, float *param);
int1 execCommand();
void cmdX(char cmd, float p1, float p2, float p3);
void cmdX(char cmd, unsigned int8 pi1, unsigned int8 pi2);
int1 is_digit(int c);
int1 is_alpha(char c);
int1 is_alnum(char c);
float charToSpeed(char c);

void commandsHelp()
{
	fprintf(PC_PORT,"THIS VERSION USES     M E T E R S    AS DISTANCE UNIT \r\n");
	fprintf(PC_PORT,"\fCommands \r\n");
	fprintf(PC_PORT,"\th\t\tShow help\r\n");
	fprintf(PC_PORT,"\ta\t\tShow Accelerometer values in G\r\n");
	fprintf(PC_PORT,"\tb byte byte\tSet angular speeds in one-byte mode\r\n");
	fprintf(PC_PORT,"\r\n");
	fprintf(PC_PORT,"\tcl\t\tGet left  speed-control constants\r\n");
	fprintf(PC_PORT,"\tcr\t\tGet right speed-control constants\r\n");
	fprintf(PC_PORT,"\tcl val\t\tTest left  speed-control constants\r\n");
	fprintf(PC_PORT,"\tcr val\t\tTest right speed-control constants\r\n");
	fprintf(PC_PORT,"\tcl min ctr max\tSet left  speed-control constants\r\n");
	fprintf(PC_PORT,"\tcr min ctr max\tSet right speed-control constants\r\n");
	fprintf(PC_PORT,"\r\n");
	
	fprintf(PC_PORT,"\tkdg\t\tGet Proportional Distance-Error Gain Constant\r\n");
	fprintf(PC_PORT,"\tkag\t\tGet Proportional Angle-Error Gain Constant\r\n");
	fprintf(PC_PORT,"\tkdt\t\tGet Proportional Distance-Error Tolerance\r\n");
	fprintf(PC_PORT,"\tkat\t\tGet Proportional Angle-Error Tolerance\r\n");
	fprintf(PC_PORT,"\tksm\t\tGet Maximum motor speed (saturation) \r\n");
	fprintf(PC_PORT,"\tkmp\t\tGet Meter-Per-Pulse Constant \r\n");
	fprintf(PC_PORT,"\tkrd\t\tGet Robot's Diameter\r\n");
	
	fprintf(PC_PORT,"\tkdg value\t\tSet Proportional Distance-Error Gain Constant\r\n");
	fprintf(PC_PORT,"\tkag value\t\tSet Proportional Angle-Error Gain Constant\r\n");
	fprintf(PC_PORT,"\tkdt value\t\tSet Proportional Distance-Error Tolerance\r\n");
	fprintf(PC_PORT,"\tkat value\t\tSet Proportional Angle-Error Tolerance\r\n");
	fprintf(PC_PORT,"\tksm value\t\tSet Maximum motor speed (saturation) \r\n");
	fprintf(PC_PORT,"\tkmp value\t\tSet Meter-Per-Pulse Constant \r\n");
	fprintf(PC_PORT,"\tkrd value\t\tSet Robot's Diameter\r\n");
	
	fprintf(PC_PORT,"\r\n");
	fprintf(PC_PORT,"\te\t\tShow encoders values used in last PID\r\n");
	fprintf(PC_PORT,"\tf\t\tShow encoders count since last f-command request\r\n");
	fprintf(PC_PORT,"\tm dst ang [t]\tSame as mv angle distance [time]\r\n");
	fprintf(PC_PORT,"\t\t\tangle in rads, distance in dm time ins secs\r\n");
	//printf("\tp\t\t\tGet Estimated position since last \r\n");
	fprintf(PC_PORT,"\tr\t\tReset Driver\r\n");
	fprintf(PC_PORT,"\ts\t\tStop motors\r\n");
	fprintf(PC_PORT,"\tu\t\tAutomatic report mode: automatic f 250ms after w\r\n");
	fprintf(PC_PORT,"\tw speedL speedR\tw leftAngularSpeed rightAngularSpeed\r\n");
	fprintf(PC_PORT,"\tp\tShow relative position since p-command request\r\n");
}

int1 execCommand()
{
	float f1=0, f2=0, f3=0;
	//int8 l1, l2;
	if(!cmdReady) return 0;
	cmdReady = 0;
	if(!fetchCommand()) return 0;
	//calib = 0;
	switch(comando.cmd)
	{
		case 'a':	//accelerometer
			//readAccelf(&f1, &f2, &f3);
			fprintf(PC_PORT,"x%1.4f", f1);
			fprintf(PC_PORT,"y%1.4f", f2);
			fprintf(PC_PORT,"z%1.4f", f3);
			break;
			
		case 'b':
			stop=1;
			counter_50ms = 0;
			newGoalPoint=1;
			//Recordar que no acepta velocidades mayores a 192 ni menores a 64
			setLeftSpeed(comando.paramInt1);
			setRightSpeed(comando.paramInt2);
			//fprintf(PC_PORT,"w %X %X\r", comando.paramInt1, comando.paramInt2);
			//return;
			//arTime = globalCounter;
			//autoStopTimeLeft = 1000;
			//reported = 0;
			//autoreport = 1;
			break;
		
		case 'e':	//encoders
			fprintf(PC_PORT,"left: %ld\tright: %ld", getLastEncoderL(), getLastEncoderR());		
			break;
			
		case 'f':	//encoders count
			fprintf(PC_PORT,"left: %ld\tright: %ld", getEncoderL(), getEncoderR());
			//fprintf(PC_PORT,"PWM:%dLR:%ld,%ld", speedL, getEncoderCountL(), getEncoderCountR());
			break;
		
		case 'h':	//detener
			commandsHelp();
			break;
			
		case 'k': //La k minúscula es para Get
			if(comando.param2 == 'd' && comando.param3 == 'g')
				fprintf(PC_PORT,"kdg: %f\r", DISTANCE_KP);
			else if(comando.param2 == 'a' && comando.param3 == 'g')
				fprintf(PC_PORT,"kag: %f\r", ANGLE_KP);
			else if(comando.param2 == 'd' && comando.param3 == 't')
				fprintf(PC_PORT,"kdt: %f\r", DIST_TOLERANCE);
			else if(comando.param2 == 'a' && comando.param3 == 't')
				fprintf(PC_PORT,"kat: %f\r", ANGLE_TOLERANCE);
			else if(comando.param2 == 's' && comando.param3 == 'm')
				fprintf(PC_PORT,"ksm: %f\r", SAT_MOTOR_SPEED);
			else if(comando.param2 == 'c' && comando.param3 == 'l')
				fprintf(PC_PORT,"kcl: %f\r", METERS_PER_PULSE_L * 8000);
			else if(comando.param2 == 'c' && comando.param3 == 'r')
				fprintf(PC_PORT,"kcr: %f\r", METERS_PER_PULSE_R * 8000);
			else if(comando.param2 == 'r' && comando.param3 == 'd')
				fprintf(PC_PORT,"krd: %f\r", ROBOT_DIAMETER);
			else return 0;
			break;
			
		case 'K'://La K mayúscula es para Set
			if(comando.param2 == 'd' && comando.param3 == 'g'){
				DISTANCE_KP = comando.param1;
				writeConstantToEEPROM(D_KP,DISTANCE_KP);
			}
			else if(comando.param2 == 'a' && comando.param3 == 'g'){
				ANGLE_KP = comando.param1;
				writeConstantToEEPROM(A_KP,ANGLE_KP);
			}
			else if(comando.param2 == 'd' && comando.param3 == 't'){
				DIST_TOLERANCE = comando.param1;
				writeConstantToEEPROM(D_TOL,DIST_TOLERANCE);
			}
			else if(comando.param2 == 'a' && comando.param3 == 't'){
				ANGLE_TOLERANCE = comando.param1;
				writeConstantToEEPROM(A_TOL,ANGLE_TOLERANCE);
			}
			else if(comando.param2 == 's' && comando.param3 == 'm'){
				SAT_MOTOR_SPEED = comando.param1;
				writeConstantToEEPROM(SAT_M_S,SAT_MOTOR_SPEED);
			}
			else if(comando.param2 == 'c' && comando.param3 == 'l'){
				METERS_PER_PULSE_L = comando.param1/8000;
				writeConstantToEEPROM(M_P_P_L,METERS_PER_PULSE_L);
			}
			else if(comando.param2 == 'c' && comando.param3 == 'r'){
				METERS_PER_PULSE_R = comando.param1/8000;
				writeConstantToEEPROM(M_P_P_R,METERS_PER_PULSE_R);
			}
			else if(comando.param2 == 'r' && comando.param3 == 'd'){
				ROBOT_DIAMETER = comando.param1;
				writeConstantToEEPROM(R_DIAM,ROBOT_DIAMETER);
			}
			else return 0;
			break;
			
		case 'm':	//comando mv
			//setup_mv(comando.param1, comando.param2, comando.param3);
			//Distancia y angulo en dm y rad, se multiplica por 0.1 para manejarlo en metros
			//fprintf(PC_PORT,"\r\nmv");
			desiredPositionX = comando.param1*cos(comando.param2);
			desiredPositionY = comando.param1*sin(comando.param2);
			desiredDistance = comando.param1;
			if(comando.param1 < 0)
			{
				desiredAngle = comando.param2 + PI; //si la distancia es negativa, el angulo es pal otro lado
				if(desiredAngle >  PI) desiredAngle-= 2*PI; //Si la distancia no es cero
				if(desiredAngle < -PI) desiredAngle+= 2*PI; //entonces el ángulo debe ser -PI <= a <= PI
			   	moveBackward = 1;   
			}												//Si la distancia es cero, entonces sólo va a girar
			else if(comando.param1 > 0)						//y el ángulo puede ser cualquiera
			{
				desiredAngle = comando.param2;
				if(desiredAngle >  PI) desiredAngle-= 2*PI;
				if(desiredAngle < -PI) desiredAngle+= 2*PI;
				moveBackward = 0;
			}
			else
			{
				desiredAngle = comando.param2;
				moveBackward = 0;
			}
			newGoalPoint=1;
			counter_50ms = 0;
			break;
			
		case 's':	//detener
			//fputc('s',PC_PORT);
			stop=1;
			setRightSpeed(127);
			setLeftSpeed(127);
			//stop();
			break;
		
		case 'o':	//posicion estimada
			//setSD(comando.param1,comando.param2);
			//printf("o%lu,%lu", shutDownL, shutDownR);
			break;
		
		case 'p':	//posicion estimada
			fprintf(PC_PORT,"\r\nPostion\tx: 0.00\ty: 0.00\t angle: 0.00");
			break;
			
		case 'r':	//resetea micro
			fputc('r',PC_PORT);
			//setSpeed(0);
			//disable_interrupts(GLOBAL);
			//CCP_1 = CCP_RESET;
			//CCP_2 = CCP_RESET;
			reset_cpu();
			break;
			
		case 'u':	//posicion estimada
			autoreport = !autoreport;
			reported = 0;
			fprintf(PC_PORT,"u %c", autoreport ? '1' : '0');
			break;
			
		case 'w':	//Velocidad
			desiredLeftW = comando.param1; //Velocidades angulares deseadas
			desiredRightW = comando.param2;//en Rad/seg
			fprintf(PC_PORT,"w %0.2f, %0.2f", comando.param1, comando.param2);
			//setLeftSpeed(comando.param1);
			//setRightSpeed(comando.param2);
			//arTime = globalCounter;
			//reported = 0;
			//printf("w %1.2f %1.2f", speedL, speedR);
			break;
		
		// Left Encoder Constants GET
		case ',':
			//printf("cl %lu, %lu, %lu\r\n", CCP_MIN_L, CCP_CENTRO_L, CCP_MAX_L);
			break;
		
		// Left Encoder Constants SET
		case ';':
			//printf("LES %ld, %ld, %ld\r\n", (long)comando.param1, (long)comando.param2, (long)comando.param3);
			//setConstantsL((long)comando.param1, (long)comando.param2, (long)comando.param3);
			break;			   	
			
		// Left Encoder Constants TEST
		case '<':
			fprintf(PC_PORT,"TEST_L %ld\r\n", (long)comando.param1);
			//testConstantL((long)comando.param1);
			break;
		
		// Right Encoder Constants GET
		case '.':
			//printf("cr %lu, %lu, %lu\r\n", CCP_MIN_R, CCP_CENTRO_R, CCP_MAX_R);
			break;
			
		// Right Encoder Constants SET
		case ':':
			//printf("RES %ld, %ld, %ld\r\n", (long)comando.param1, (long)comando.param2, (long)comando.param3);
			//setConstantsR((long)comando.param1, (long)comando.param2, (long)comando.param3);
			break;
			
		// Right Encoder Constants TEST
		case '>':
			fprintf(PC_PORT,"TEST_R %ld\r\n", (long)comando.param1);
			//testConstantR((long)comando.param1);
			break;
			
		default:
			fprintf(PC_PORT,"\r\nUnknown Command");
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
	unsigned int8 pi1;
	unsigned int8 pi2;
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
	else if((buffer[0] == 'k'))
	{
		cc = 1;
		if((buffer[cc] != 'd') && (buffer[cc] != 'a') && (buffer[cc] != 's') && 
		   (buffer[cc] != 'c') && (buffer[cc] != 'r')) return 0;
		++cc;
		if((buffer[cc] != 'g') && (buffer[cc] != 't') && (buffer[cc] != 'm') && 
		   (buffer[cc] != 'l') && (buffer[cc] != 'r') && (buffer[cc] != 'd')) return 0;
		++cc;
		if(!fetchParam(&cc, &p1))					// Obtengo primer parametro (si lo hay)
			p1 = -1;
		if(buffer[cc] != 0) return 0;				// Se espera fin de cadena
		
		
		p2 = buffer[1];
		p3 = buffer[2];
		if(p1 == -1)								// Modo GET
			cmdX('k', p1, p2, p3);
		else										// Modo SET
			cmdX('K', p1, p2, p3);
		return 1;
	}
	else if(buffer[0] == 'b')
	{
	
		//p1 = charToSpeed(buffer[1]);
		//p2 = charToSpeed(buffer[2]);
		//No acepta velocidades mayores a 192 ni menores que 64
		//es decir, ni mucho padelante ni mucho patrás
		pi1 = buffer[1];
		if(pi1 > 165) pi1 = 165;
		if(pi1 < 89)  pi1 = 89;
		pi2 = buffer[2];
		if(pi2 > 165) pi2 = 165;
		if(pi2 < 89)  pi2 = 89;
		cmdX(buffer[0], pi1, pi2);
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

void cmdX(char cmd, unsigned int8 pi1, unsigned int8 pi2)
{
	comando.cmd = cmd;
	comando.paramInt1 = pi1;
	comando.paramInt2 = pi2;
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

float charToSpeed(char c)
{
	// y = 6+(x-128)*12/256
	//float f;
	//f = (int)c;
	return 6.0 + (float)((signed int)c - 128.0) * 0.046875;
}


