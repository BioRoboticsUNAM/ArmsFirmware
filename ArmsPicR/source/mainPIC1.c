#include <30f4011.h>
#use delay (clock=96000000)
#include <stdlib.h>
#fuses XT_PLL8,NOPROTECT,NOWDT
#use rs232(baud=57600, xmit=PIN_F3,rcv=PIN_F2, stream=PIC_COM)
#use rs232(baud=115200, xmit=PIN_F5,rcv=PIN_F4, stream=PC_PORT)
#include <commands.c>

//VARIABLES GLOBALES
//Variables para el control
float currentPositionX = 0;
float currentPositionY = 0;
float currentAngle=0;
float errorPositionX=0;
float errorPositionY=0;
float errorAngle=0;
float errorFinalAngle=0; //Este es el error de angulo para el caso en que sólo se quiere girar sin avanzar
float errorDistance=0;
int1 correctingFinalAngle=0; //Indica si esta alcanzando un punto o está corrigiendo orientación
int1 isMoving=0;
int1 doControl=0;
int16 timeOut = 0;

//Variables para medir posición con los encoders
float leftS;  //Distancia avanzada por la llanta izquierda en metros
float rightS; //Distancia avanzada por la llanta derecha en metros
float delta_teta;
float radio_giro;
float delta_x;
float delta_y;
float speedL;
float speedR;

//Variables para reportar la distancia y el ángulo avanzado
float measuredDistance;
float pathAngle;

//Variables para fijar velocidades
unsigned int8 intSpeedL;
unsigned int8 intSpeedR;

void parseCommand(char c);
void calculatePosition(float Si, float Sd);
void executeControl();

#INT_TIMER3 //Interrupción de tiempo cada 50 ms
void timer3_controlAction()
{
	doControl=1;
}

void main()
{
	char nextChar;
	
	delay_ms(2000);
	//commandsHelp();
	readConstantsFromEEPROM();
	setup_peripherals();
	//fprintf(PC_PORT,"\r\nSYSTEM READY\r\nPress 'h' for help");
	
	while(true)
	{//Estas son las que cachan el comando por serial   
		if(kbhit(PC_PORT))
		{
			nextChar = fgetc(PC_PORT);
			parseCommand(nextChar);
			if(cmdReady)
			{
				if(!execCommand()) fprintf(PC_PORT,"Unknown Command\r");
			}
		}
		
		if(doControl)
		{
			doControl=0;
			executeControl();
		}
	}
}

void parseCommand(char c)
{
	static char next = 0;
	
	buffer[next] = 	c;
	//fputc(buffer[next],PC_PORT); //Esta línea es para que haga eco
	
	// Parseo del comando b
	if(buffer[0] == 'b')
	{
		if(next < 3)
		{
			++next;
			return;
		}
		if(buffer[3] != '\r')
		{
			next = 0;
			return;
		}	
		buffer[3] = 0;
		cmdReady = 1;
		next = 0;
		return;
	}
		// Parseo del resto de los comandos
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
		fputc(0x0A,PC_PORT);
		buffer[next] = 0; //El cero se coloca al final para indicar fin de linea
		//next++;
		//buffer[next] = 0;
		if(next != 0)
		{
			cmdReady = 1;
			next = 0;
		}
	}
}

void calculatePosition(float Si, float Sd)
{
	if (Si != Sd)
			{
				Si = Si * METERS_PER_PULSE_L;
				Sd = Sd * METERS_PER_PULSE_R;
				delta_teta = (Sd - Si) / ROBOT_DIAMETER;
				radio_giro = (Si + Sd) / (2 * delta_teta);
				delta_x = radio_giro * sin(delta_teta);
				delta_y = radio_giro * (1 - cos(delta_teta));
				//imprimir aquí el valor del timer en %X e imprimir valores de delta_X y delta_y
				//timer3Value = get_timer3();
				//fprintf(PC_PORT,"\r\n %lu \t %0.6f  %0.6f  delta_x,delta_Y",timer3Value,delta_x,delta_y);
			}
			else
			{
				leftS = leftS * METERS_PER_PULSE_L;
				rightS = rightS * METERS_PER_PULSE_R;
				delta_x = leftS;
				delta_y = 0;
				delta_teta = 0;
				//Misma funcion que en el bloque if anterior
				//timer3Value = get_timer3();
				//fprintf(PC_PORT,"\r\n %lu \t %0.6f  %0.6f   delta_x, delta_y",timer3Value,delta_x,delta_y);
			}

			currentPositionX += delta_x * cos(currentAngle) - delta_y * sin(currentAngle);
			currentPositionY += delta_x * sin(currentAngle) + delta_y * cos(currentAngle);
			currentAngle += delta_teta;
			if(currentAngle < -PI) errorAngle+=2*PI;
			if(currentAngle >  PI) errorAngle-=2*PI;
}

void executeControl()
{
	counter_50ms++;
	//fprintf(PC_PORT,"\r\nNew Timer");
	if(comando.cmd == 'w')
	{//Acción de control para velocidades angulares
		//fprintf(PC_PORT,"\r\nTrying to reach angular speeds");
	}
	else if(comando.cmd == 'm')
	{//Acción de control para el comando mv
	//LAS DISTANCIAS SE MANEJAN EN METROS Y LOS ÁNGULOS EN RADIANES
		//fprintf(PC_PORT,"\r\nTrying to reach goal point");
		if(newGoalPoint)
		{
			newGoalPoint=0;
			currentPositionX=0;
			currentPositionY=0;
			currentAngle=0;
			encodersCleared=0;
			goalPointReached = 0;
			correctingFinalAngle=0;
			stop=0;
			if(moveBackward) currentAngle+=PI;
			timeOut = (int16)(100*abs(desiredDistance) + 40*abs(desiredAngle)+ 20);
			//timeOut = desiredDistance > 5 ? (int16)(100*abs(desiredDistance) + 40*abs(desiredAngle)) : 20;
			//primero probar poniendo aquí un print para verificar que los valores deseados y actuales son correctos
			//Imprimir también el valor del timer en %X
			//timer3value = get_timer3();
			//fprintf(PC_PORT,"\r\n %lu \t %2.4f  %2.4f newGoalPoint",timer3Value,desiredPositionX,desiredPositionY);
		}
		else if(!encodersCleared)
		{
			if(getEncoderL()==0 && getEncoderR()==0) encodersCleared=1;
			//fprintf(PC_PORT,"\r\n limpieza de Encoders");
		}
		else if(!goalPointReached && !stop)
		{
			if(!moveBackward)
			{
				leftS = (float)getEncoderL();
				rightS = (float)getEncoderR();
			}
			else
			{
				rightS = -(float)getEncoderL(); //Si va hacia atrás los lados y signos
				leftS = -(float)getEncoderR(); //de las llantas y los encoders se cambian
			}
			calculatePosition(leftS,rightS);   
			//Misma funcion linea de prueba
			//timer3Value = get_timer3();
			//fprintf(PC_PORT,"\r\n %lu \t %0.6f  %0.6f   currentPositionX",timer3Value,currentPositionX,currentPositionY);
			
			errorPositionX = desiredPositionX - currentPositionX;
			errorPositionY = desiredPositionY - currentPositionY;
			errorDistance = sqrt(errorPositionX*errorPositionX + errorPositionY*errorPositionY);
			errorAngle = atan2(errorPositionY,errorPositionX) - currentAngle;
			errorFinalAngle = desiredAngle - currentAngle;
			if(errorAngle < -PI) errorAngle+=2*PI;
			if(errorAngle >  PI) errorAngle-=2*PI;
			
			//Misma funcion linea de prueba
			//timer3Value = get_timer3();
			//fprintf(PC_PORT,"\r\n %lu \t %0.2f  %0.2f",timer3Value,errorDistance,errorAngle);
			
			//Ésta es la parte que hace la acción de control
			if(errorAngle >  SAT_ERROR_ANGLE) errorAngle =  SAT_ERROR_ANGLE;
			if(errorAngle < -SAT_ERROR_ANGLE) errorAngle = -SAT_ERROR_ANGLE;
			if(errorFinalAngle >  SAT_ERROR_ANGLE) errorFinalAngle =  SAT_ERROR_ANGLE;
			if(errorFinalAngle < -SAT_ERROR_ANGLE) errorFinalAngle = -SAT_ERROR_ANGLE;
			
			if(errorDistance > DIST_TOLERANCE && counter_50ms < timeOut && !correctingFinalAngle) 
			{
				if(!moveBackward)
				{
					speedL = -ANGLE_KP*errorAngle + DISTANCE_KP*errorDistance*(SAT_ERROR_ANGLE - abs(errorAngle));
					speedR =  ANGLE_KP*errorAngle + DISTANCE_KP*errorDistance*(SAT_ERROR_ANGLE - abs(errorAngle));
				}
				else
				{
					speedR =  ANGLE_KP*errorAngle - DISTANCE_KP*errorDistance*(SAT_ERROR_ANGLE - abs(errorAngle));
					speedL = -ANGLE_KP*errorAngle - DISTANCE_KP*errorDistance*(SAT_ERROR_ANGLE - abs(errorAngle));
				}
			}
			else if(abs(errorFinalAngle) > ANGLE_TOLERANCE && counter_50ms < timeOut)
			{ //Una vez que ya recorrió la distancia hace la corrección del ángulo
				correctingFinalAngle=1;
				if(moveBackward)
				{
					speedL = -ANGLE_KP*errorFinalAngle;
					speedR =  ANGLE_KP*errorFinalAngle;
				}
				else
				{
					speedR =  ANGLE_KP*errorFinalAngle;
					speedL = -ANGLE_KP*errorFinalAngle;
				}
			}
			else
			{
				speedL = 0;
				speedR = 0;
				goalPointReached = 1;
			}
			if(speedL >  SAT_MOTOR_SPEED) speedL =  SAT_MOTOR_SPEED;
			if(speedL < -SAT_MOTOR_SPEED) speedL = -SAT_MOTOR_SPEED;
			if(speedR >  SAT_MOTOR_SPEED) speedR =  SAT_MOTOR_SPEED;
			if(speedR < -SAT_MOTOR_SPEED) speedR = -SAT_MOTOR_SPEED;
			
			//Misma funcion linea de prueba
			//timer3Value = get_timer3();
			//fprintf(PC_PORT,"\r\n %lu \t %0.6f  %0.6f  float speeds",timer3Value,speedL,speedR);
			
			//El 69 es el valor que se le envía al speed (127+- 69) con el que se alcanza
			//la velocidad de 1 m/s
			speedL = 127 + 69*speedL; //Esto es para tener las speeds en unidades 
			speedR = 127 + 69*speedR; //que acepta el speed control
			
			intSpeedL = (unsigned int8)speedL;
			intSpeedR = (unsigned int8)speedR;
			
			setLeftSpeed(intSpeedL);
			setRightSpeed(intSpeedR);
			isMoving=1;
			
			//timer3Value = get_timer3();
			//fprintf(PC_PORT,"\r\n %lu \t %u  %u",timer3Value,intSpeedL,intSpeedR);
			
		}else if(isMoving && !stop)
		{
			if(!moveBackward)
			{
				leftS = (float)getEncoderL();
				rightS = (float)getEncoderR();
			}
			else
			{
				rightS = -(float)getEncoderL(); //Si va hacia atrás los lados y signos
				leftS = -(float)getEncoderR(); //de las llantas y los encoders se cambian
			}
			calculatePosition(leftS,rightS);   
			if(leftS == 0 && rightS == 0)
			{
				isMoving=0;
				measuredDistance = sqrt(currentPositionX*currentPositionX + currentPositionY*currentPositionY);
				pathAngle = atan2(currentPositionY,currentPositionX);
			
				if(!moveBackward) 
				{
					fprintf(PC_PORT,"mv %0.4f",measuredDistance);
					fprintf(PC_PORT," %0.4f\r",currentAngle);	
				}
				else
				{
					fprintf(PC_PORT,"mv -%0.4f",measuredDistance);
					currentAngle-=PI;
					if(currentAngle < -PI) currentAngle+=2*PI;
					if(currentAngle >  PI) currentAngle-=2*PI;
					fprintf(PC_PORT," %0.4f\r",currentAngle);	
				}
			}
		}
	}
	else if(comando.cmd == 'b')
	{
		if(counter_50ms == 2 && newGoalPoint)
		{
			newGoalPoint=0;
			fprintf(PC_PORT,"%ld.0\r",getEncoderL());
         	fprintf(PC_PORT,"%ld.0\r",getEncoderR());
		}
		if(counter_50ms == 8)
		{
			counter_50ms = 0;
			setLeftSpeed(0x7F);
			setRightSpeed(0x7F);
		}
	}
	else
	{
		setLeftSpeed(0x7F);
		setRightSpeed(0x7F);
	}
}
