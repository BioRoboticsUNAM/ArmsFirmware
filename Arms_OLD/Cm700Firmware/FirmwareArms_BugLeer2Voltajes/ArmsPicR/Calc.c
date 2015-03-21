#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "servoControl.c"
#include <math.h>


typedef struct 
{
	float value[4][4];
} matrix4;

typedef struct 
{
	float value[DOF];
} vector7;

typedef struct 
{
	float value[4];
} vector4;

typedef struct 
{
	float value[3];
} vector3;


/////// ARM CONTROL VARIABLES
float maxTol =  0.1;

int object = 0;
int doLoop = 1;
int doReading = 1;
int doControl = 0;
int printPosLoop = 0;
float executionTF = 0;
float sampleTime = 0.01;
int timeCounter = 0;
int instructionSet = 0;

//float pastPos[DOF];
float desiredQPosition[DOF];
float currentQPosition[DOF];
float initialQPosition[DOF]; // Como no leen los servos, cada muestreo se modifica currentQPosition para poder tener la posici髇 instant醤ea. Pero para caluclar la siguiente posici髇 instant醤ea, es necesaria la posici髇 inicial de la trayectoria
float desiredXPosition[DOF];
float currentXPosition[DOF];
float lastPosition[DOF];
float differentialPosition[DOF];

vector4 LSPBcoef[DOF];
vector4 PERcoef[DOF];

/////// GRIPPER CONTROL VARIABLES

int doGripper = 0;
float desiredForce = 0;
//int objectTaken = 0;

int gripperCounter = 0;

/// VARIABLES

float aDH[DOF];				// a DH parameter
float dDH[DOF];				// d DH parameter
float alphaDH[DOF];			// alpha DH parameter
float thetaDH[DOF];			// theta DH parameter
float qDH[DOF];				// theta DH parameter
//float H[DOF+1][4][4];			// Homogenean Matrices. DOF+1 Incluye la 鷏tima rotaci髇 para alinear la orientaci髇 del gripper
matrix4 H[DOF+1];
//float H07[4][4];				// Homogenean Transformation from 0 to 7. (0 a 8 incluyendo la 鷏tima rotaci髇)
matrix4 H07;
//float cPos[DOF];
//float H0[DOF+1][4][4];		// H.T. from 0 to 1, 0 to 2, 0 to 3 up to 0 to 7
matrix4 H0[DOF+1];




/////// ARM METHODS
void executeControl();
void updateQPosition();
int SetdesiredPosition(float p0,float p1,float p2,float p3,float p4,float p5,float p6, float tf);
void SetConstants();

//void mult(float **a,float **b,float **c);
matrix4 mult(matrix4 a, matrix4 b);
float multVMV(vector4 a, matrix4 b, vector4 c);
vector4 multMV(matrix4 b, vector4 c);
void TestMult();
matrix4 InvH(matrix4 matrix);
vector4 coefLSPB (float q0, float qf, float tf);
float evalLSPB(float q0,float qf,vector4 data,float t);
vector4 coefQuintico (float q0, float qf, float tf);
float perQuintico(float q0,float qf,vector4 data,float t);


/////// GRIPPER METHODS
void executeGripper();
void OpenGripper(float perc);
void CloseGripper(float perc);
int ObjectInHand();

/////// KINEMATICS METHODS
void SetKConstants();
void CalculateDH(int desired);
void CalculateHT(int desired);
void ForwardKinematics();
int InverseKinematics();


//////// ARM METHODS

void SetConstants()
{


	for(int i = 0;i<DOF;i++)
	{

	desiredQPosition[i] = 0;
	currentQPosition[i] = 0;
	initialQPosition[i] = 0;
	desiredXPosition[i] = 0;
	currentXPosition[i] = 0;
	lastPosition[i] = 0;
	differentialPosition[i] = 0;
		for(int j = 0;j<4;j++)
		{
			PERcoef[i].value[j] = 0;
			//LSPBcoef[i].value[j] = 0;
		}	
	}
	
}
void executeControl()
{
	
	// READING
	if(doReading == 1 && doControl == 0)
	{
		updateQPosition();
		//for(int i = 0; i<DOF;i++)
		//{
		//	currentQPosition[i] = GetPosition(i);
		
			
			//if(abs(currentQPosition[i] - lastPosition[i])>0.5)
			//{
		 	//currentQPosition[i] = (lastPosition[i] + differentialPosition[i]);
			//}
		// ESTIMATE NEXT STEP
		//lastPosition[i] = currentQPosition[i];
		//differentialPosition[i] = currentQPosition[i] - lastPosition[i];	
		//_delay_us(500);

		//_delay_ms(1);
		
		//}
		

	//printf("Test g 1 %f %f %f %f %f %f %f \r\n",currentQPosition[0],currentQPosition[1],currentQPosition[2],currentQPosition[3],currentQPosition[4],currentQPosition[5],currentQPosition[6]);
	ForwardKinematics();
	
	if(instructionSet == 1)
	{

		printf("p 1 %f %f %f %f %f %f %f \r", currentXPosition[0],currentXPosition[1],currentXPosition[2],currentXPosition[3],currentXPosition[4],currentXPosition[5],currentXPosition[6]);	

		instructionSet = 0;
	}

	if(instructionSet == 2)
	{
		printf("q 1 %f %f %f %f %f %f %f \r", currentQPosition[0],currentQPosition[1],currentQPosition[2],currentQPosition[3],currentQPosition[4],currentQPosition[5],currentQPosition[6]);	
		instructionSet = 0;
	}

	}
	
	if(doControl)
	{
		float nextPos[DOF];
		float t;
		//Aqui va ir la evaluaci髇 de la trayectoria y el env韔 de datos. El prec醠culo de las constantes de la planeaci髇 se hacen en cmdATM al recibir los datos en la instrucci髇 p

		t = timeCounter * sampleTime;
		//printf("ExecutionTime = %f \n",executionTF);
		//printf("time = %f \n",t);

		for(int i = 0; i<DOF;i++)
		{
			
				nextPos[i] = perQuintico(initialQPosition[i],desiredQPosition[i],PERcoef[i],t);
				//nextPos[i] = evalLSPB(initialQPosition[i],desiredQPosition[i],LSPBcoef[i],t);
				//currentQPosition[i]	 = nextPos[i];
				
				currentQPosition[i] = GetPosition(i);
				
				//if(abs(currentQPosition[i] - nextPos[i])>maxTol)
				//currentQPosition[i] = nextPos[i];
		}
		
		ArmPosition(nextPos);
		
		//printf("%f , %f , %f , %f , %f  , %f , %f , %f \n", t, nextPos[0],nextPos[1],nextPos[2],nextPos[3],nextPos[4],nextPos[5],nextPos[6]);
		_delay_ms(5);
		timeCounter++;
		
		if(t >= executionTF)
		{
			doControl = 0;
		}

	}
	
}


void updateQPosition()
{
	for(int i = 0; i<DOF;i++)
	{
		currentQPosition[i] = GetPosition(i);
	}
}

int SetdesiredPosition(float p0,float p1,float p2,float p3,float p4,float p5,float p6, float tf)
{

	int succes;
	succes = 0;
    
	SetMaxSpeeds(20);
	updateQPosition();

	if(tf<0.5)
	{
		return 0;
	}

	// 1) Escribe Dato. Posici髇 Cartesiana Deseada.

	if(instructionSet==1)
	{
	
		desiredXPosition[0] = p0;
		desiredXPosition[1] = p1;
		desiredXPosition[2] = p2;
		desiredXPosition[3] = p3;
		desiredXPosition[4] = p4;
		desiredXPosition[5] = p5;
		desiredXPosition[6] = p6;

	
		// 2) Cinem醫ica Inversa

		//printf("Inverse Kinematics \n\r");
		succes = InverseKinematics();
	}
	else if(instructionSet==2)
	{
		desiredQPosition[0] = p0;
		desiredQPosition[1] = p1;
		desiredQPosition[2] = p2;
		desiredQPosition[3] = p3;
		desiredQPosition[4] = p4;
		desiredQPosition[5] = p5;
		desiredQPosition[6] = p6;
	//	printf("Angular Pos \n\r");
		succes = 1;
	}

	timeCounter = 0;
	executionTF = tf;

	printf("CurrentQPos: %f , %f , %f , %f  , %f , %f , %f \r", currentQPosition[0],currentQPosition[1],currentQPosition[2],currentQPosition[3],currentQPosition[4],currentQPosition[5],currentQPosition[6]);
	printf("DesiredQPos: %f , %f , %f , %f  , %f , %f , %f \r", desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);

	// 2) Constantes de Planeaci髇-
	for(int i = 0;i<DOF;i++)
	{
		currentQPosition[i] = GetPosition(i);
		initialQPosition[i] = currentQPosition[i];
		
	}
	for(int i = 0; i<DOF;i++)
	{
		PERcoef[i] = coefQuintico (initialQPosition[i],desiredQPosition[i], executionTF);
		//LSPBcoef[i] = coefLSPB (initialQPosition[i],desiredQPosition[i], executionTF);
	}
	
	if(succes)
	{
		doControl = 1;
		return 1;
	}
	else{
		doControl = 0;
		return 0;
	}
}








////// GRIPPER METHODS

void executeGripper()
{
	/// CHECK LOAD ON GIPPER
	float s1,pos;
	float tolerance = 0.1;

	//s1 = -GetLoad(17);
	//printf("Load: %f \n\r",s1);
	//s2 = GetLoad(17);

	//av = ((s1+s2)/2.0);
	//if(s1>= desiredForce)
	//{
		//printf("Force Limit Passed");

		//pos = GetPosition(7);
		gripperCounter++;
		if(gripperCounter>= 100)
		{
			doGripper = 0;
			object = ObjectInHand();
			printf("c 1 %d\r",object);
		}
		//printf("Pos: %f  counter: %d \n\r",pos,gripperCounter);

		//if(abs(pos) > tolerance)
		//{
		//objectTaken = 1;
		//	doGripper = 0;
		//	printf("c 1\r");
		//}else
		//{
		//	objectTaken = 0;
		//	doGripper = 0;
		//	printf("c 0\r");
		//}
	//}
	//else
	//{
	//	doGripper = 1;
		
	//}



		
}

int OpenHand(float perc1, float perc2, float perc3, float perc4)
{
	//Base del pulgar
	MovePerc(1,perc2);
	_delay_ms(100);
	
	//Pulgar
	MovePerc(0,perc1);
	//Indice
	MovePerc(2,perc3);
	//Los otros
	MovePerc(3,perc4);

	return 1;

}

void OpenGripper(float perc)
{
	float openedPosition = 1.1;
	
	SetTorqueControl(7,0);
	SetSpeedPerc(7,20);
	int bits1 = SetPosition(7,(openedPosition * perc / 100.0) );
	dxl_write_word( 7, SERVO_GOAL_POSITION_L,  bits1);
	
	_delay_ms(10);

	SetTorqueControl(8,0);
	SetSpeedPerc(17,20);
	int bits2 = SetPosition(8,(openedPosition * perc / 100.0) );
	dxl_write_word( 17, SERVO_GOAL_POSITION_L, bits2);

}

void CloseGripper(float perc)
{
	SetTorqueControl(7,1);
	int bits1 = SetTorque( 7,  perc);
	dxl_write_word( 7, SERVO_PROG_SPEED ,  bits1 );
	
	_delay_ms(10);

	SetTorqueControl(8,1);
	int bits2 = SetTorque( 8,  perc);
	dxl_write_word( 17, SERVO_PROG_SPEED , bits2 );

	//printf("bits7: %d ",bits1);	
	//printf("bits17: %d",bits2);	
	
	desiredForce = perc;
	doGripper = 1;


}

float Absolute(float numero)
{
	if(numero <0) return -numero;
	else return numero;
}

int ObjectInHand()
{
	//If position is less than 10% of opened position there's no object holding
	float openedPosition = 1.5;	

	float posG1,posG2;
	posG1 = Absolute(GetPosition(7))-0.1;
	_delay_ms(10);
	posG2 = Absolute(GetPosition(8));
	
	//Debug line
	//printf("%3.3f : %3.3f ; %3.3f  \r",posG1,posG2,openedPosition*0.2);
	
	if((posG1<(openedPosition*0.1))&&(posG2<(openedPosition*0.1)))
	{
		return 0;
	}
	return 1;

}

float multVMV(vector4 a, matrix4 b, vector4 c)
{


	//float temp[4];
	vector4 partial;
	float temp, result;

		for(int k = 0;k<4;k++)
			{	
				temp = 0;
				for(int l = 0;l<4;l++)
				{	
					temp += (b.value[k][l] * c.value[l]);
				}
				partial.value[k] = temp;
			}
			
		result = 0;
		for(int l = 0;l<4;l++)
			{	
				result += (a.value[l] * partial.value[l]);
			}
		return result;
}


vector4 multMV(matrix4 b, vector4 c)
{

	//float temp[4];
	vector4 partial;
	float temp, result;

		for(int k = 0;k<4;k++)
			{	
				temp = 0;
				for(int l = 0;l<4;l++)
				{	
					temp += (b.value[k][l] * c.value[l]);
				}
				partial.value[k] = temp;
			}
			
		//result = 0;
		//for(int l = 0;l<4;l++)
		//	{	
		//		result += (a.value[l] * partial.value[l]);
		//	}
		//return result;
		return partial;

}

matrix4 mult(matrix4 a, matrix4 b)
{
	float temp[4];
	matrix4 response;
	
	for(int j = 0; j<4;j++)
		{
		temp[j] = 0;
			for(int k = 0;k<4;k++)
			{
				response.value[j][k] = 0.0f;
				//printf("%i %i %f \n ",j,k,response.value[j][k]);
			}

		}

	for(int j = 0;j<4;j++)
		{	
			for(int h = 0; h <4 ;h++)
			{
				temp[h] = 0;
			}
			for(int k = 0;k<4;k++)
			{
				for(int l = 0;l<4;l++)
				{	
					temp[k] += (a.value[j][l] * b.value[l][k]);
				}
			}
			for(int h = 0; h <4 ;h++)
			{
				response.value[j][h] = temp[h];
				
			}		
		}	

		return response;

}






















/////////////////////////////////////////////////////////////////  KINEMATICS  ///////////////////////////////////////////////


//////// METHODS

void SetKConstants()
{
for(int i = 0;i<DOF;i++)
	{
		aDH[i] = 0;				// a DH parameter
		dDH[i] = 0;				// d DH parameter
		alphaDH[i] = 0;			// alpha DH parameter
		thetaDH[i] = 0;			// theta DH parameter
		qDH[i] = 0;				// theta DH parameter
		//cPos[i] = 0;
		for(int j = 0;j<4;j++)
		{
			for(int k = 0;k<4;k++)
			{
				H[i].value[j][k] = 0;	
				//H07.value[j][k] = 0;
				H0[i].value[j][k] = 0;	
			}
		}
	}

	for(int j = 0;j<4;j++)
	{
		for(int k = 0;k<4;k++)
		{
			H07.value[j][k] = 0;
			H[7].value[j][k] = 0;	
			H0[7].value[j][k] = 0;	
		}
	}
	
	dDH[0] = 0.0;
	dDH[2] = 0.3084;
	dDH[4] = 0.2126;
	dDH[6] = 0.185;
	
	aDH[0] = 0.0603;

	alphaDH[0]= M_PI_2;
	alphaDH[1]= M_PI_2;
	alphaDH[2]= -M_PI_2;
	alphaDH[3]= M_PI_2;
	alphaDH[4]= -M_PI_2;
	alphaDH[5]= M_PI_2;

	thetaDH[1] = M_PI_2;
	thetaDH[2] = -M_PI_2;

	H[7].value[2][0] = 1;
	H[7].value[0][1] = 1;
	H[7].value[1][2] = 1;
	H[7].value[3][3] = 1;


}

void CalculateDH(int desired) // Current 0, Desired 1
{
	if(desired==1)
	{
		for(int i = 0; i < DOF;i++)
		{
			qDH[i] = desiredQPosition[i] + thetaDH[i];
		}
	}
	else
	{
		for(int i = 0; i < DOF;i++)
		{
			qDH[i] = currentQPosition[i] + thetaDH[i];
		}
	}
	

	for (int i = 0; i < DOF ; i++)
	{
	
		H[i].value[0][0] = cos(qDH[i]);
		H[i].value[0][1] = -sin(qDH[i]) * cos(alphaDH[i]);
		H[i].value[0][2] = sin(qDH[i]) * sin(alphaDH[i]);
		H[i].value[0][3] = aDH[i] * cos(qDH[i]);

		H[i].value[1][0] = sin(qDH[i]);
		H[i].value[1][1] = cos(qDH[i]) * cos(alphaDH[i]);
		H[i].value[1][2] = -cos(qDH[i]) * sin(alphaDH[i]);
		H[i].value[1][3] = aDH[i] * sin(qDH[i]);

		H[i].value[2][1] = sin(alphaDH[i]);
		H[i].value[2][2] = cos(alphaDH[i]);
		H[i].value[2][3] = dDH[i];

		H[i].value[3][3] = 1;
 
	}
}


void CalculateHT(int desired) // Current 0, Desired 1
{
	CalculateDH(desired);

	float temp[4];

	for(int j = 0;j<4;j++)
	{
		for(int k = 0;k<4;k++)
		{
			H07.value[j][k] = 0;
		}
	}

	for(int i = 0;i<4;i++)	// Matriz Identidad
	{
		H07.value[i][i] = 1;
	}

	for(int i = 0;i<DOF+1;i++)
	{
		for(int j = 0;j<4;j++)
		{	
			for(int h = 0; h <4 ;h++)
			{
				temp[h] = 0;
			}
			for(int k = 0;k<4;k++)
			{
				for(int l = 0;l<4;l++)
				{
					temp[k] += (H07.value[j][l] * H[i].value[l][k]);
				}
			}
			for(int h = 0; h <4 ;h++)
			{
				H07.value[j][h] = temp[h];
			}		
		}
		for(int k = 0;k<4;k++)
			{
				for(int l = 0;l<4;l++)
				{
					H0[i].value[k][l] = H07.value[k][l];
				}
			}
		//printf("%f \t %f \t %f \t %f \t \n",H0[i][0][0],H0[i][0][1],H0[i][0][2],H0[i][0][3]);
		//printf("%f \t %f \t %f \t %f \t \n",H0[i][1][0],H0[i][1][1],H0[i][1][2],H0[i][1][3]);
		//printf("%f \t %f \t %f \t %f \t \n",H0[i][2][0],H0[i][2][1],H0[i][2][2],H0[i][2][3]);
		//printf("%f \t %f \t %f \t %f \t \n",H0[i][3][0],H0[i][3][1],H0[i][3][2],H0[i][3][3]);
		//printf(" \n ");
	}
}


void ForwardKinematics()
{
	
	CalculateHT(0);
	currentXPosition[0] = H07.value[0][3];
	currentXPosition[1] = H07.value[1][3];
	currentXPosition[2] = H07.value[2][3];

	currentXPosition[3] = atan2(H07.value[1][0],H07.value[0][0]);
	currentXPosition[4] = atan2(-H07.value[2][0],sqrt(1 - (H07.value[2][0]*H07.value[2][0]) ));
	currentXPosition[5] = atan2(H07.value[2][1],H07.value[2][2]);

	currentXPosition[6] = currentQPosition[2];

	if(printPosLoop ==1)
	{
	printf("G 1 %f %f %f %f %f %f %f \r",currentXPosition[0],currentXPosition[1],currentXPosition[2],currentXPosition[3],currentXPosition[4],currentXPosition[5],currentXPosition[6]);
	}

}

int InverseKinematics()
{
	
	matrix4 R07;
	double r,alpha,beta,gamma, tunningRadiusElbow;

	matrix4 R47;
	matrix4 R40;
	matrix4 oRe;

	vector4 WristPosition;
	vector4 Pelbow;
	

	for(int j = 0;j<4;j++)
		{
			WristPosition.value[j] = 0;
			Pelbow.value[j] = 0;
			for(int k = 0;k<4;k++)
			{
				R07.value[j][k] = 0;
				R47.value[j][k] = 0;
				R40.value[j][k] = 0;
				oRe.value[j][k] = 0;
				
			}
		}

	R07.value[0][0] = cos(desiredXPosition[3]) * cos(desiredXPosition[4]);
	R07.value[0][1] = -sin(desiredXPosition[3]) * cos(desiredXPosition[5]) + cos(desiredXPosition[3]) * sin(desiredXPosition[4]) * sin(desiredXPosition[5]);
	R07.value[0][2] = sin(desiredXPosition[3]) * sin(desiredXPosition[5]) + cos(desiredXPosition[3]) * sin(desiredXPosition[4]) * cos(desiredXPosition[5]);

	R07.value[1][0] = sin(desiredXPosition[3]) * cos(desiredXPosition[4]);
	R07.value[1][1] = cos(desiredXPosition[3]) * cos(desiredXPosition[5]) + sin(desiredXPosition[3]) * sin(desiredXPosition[4]) * sin(desiredXPosition[5]);
	R07.value[1][2] = -cos(desiredXPosition[3]) * sin(desiredXPosition[5]) + sin(desiredXPosition[3]) * sin(desiredXPosition[4]) * cos(desiredXPosition[5]);

	R07.value[2][0] = -sin(desiredXPosition[4]);
	R07.value[2][1] = cos(desiredXPosition[4]) * sin(desiredXPosition[5]);
	R07.value[2][2] = cos(desiredXPosition[4]) * cos(desiredXPosition[5]);

	R07.value[3][3] = 1;

	
	//printf(" \n ");
	//printf(" R07 ");
	//printf("%f \t %f \t %f \t %f \t \n",R07.value[0][0],R07.value[0][1],R07.value[0][2],R07.value[0][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R07.value[1][0],R07.value[1][1],R07.value[1][2],R07.value[1][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R07.value[2][0],R07.value[2][1],R07.value[2][2],R07.value[2][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R07.value[3][0],R07.value[3][1],R07.value[3][2],R07.value[3][3]);
	//printf(" \n ");




	if(desiredXPosition[6] > M_PI) desiredXPosition[6] -= 2*M_PI;
	if(desiredXPosition[6] < -M_PI) desiredXPosition[6] += 2*M_PI;

	//printf("  DesiredXPosition6 \n ");
	//printf(" %f ",desiredXPosition[6]);

	WristPosition.value[0] = dDH[6];
	WristPosition.value[3] = 1;

	WristPosition = multMV(R07,WristPosition);

	

	//printf("WP  %f %f %f %f \n\r",WristPosition.value[0],WristPosition.value[1],WristPosition.value[2],WristPosition.value[3]);

	desiredXPosition[0] = desiredXPosition[0] - WristPosition.value[0];
	desiredXPosition[1] = desiredXPosition[1] - WristPosition.value[1];
	desiredXPosition[2] = desiredXPosition[2] - WristPosition.value[2];

	//printf("d x %f %f %f %f %f %f %f \n\r",desiredXPosition[0],desiredXPosition[1],desiredXPosition[2],desiredXPosition[3],desiredXPosition[4],desiredXPosition[5],desiredXPosition[6]);

	WristPosition.value[0] = desiredXPosition[0];
	WristPosition.value[1] = desiredXPosition[1];
	WristPosition.value[2] = desiredXPosition[2];
	WristPosition.value[3] = 1;

	

	desiredQPosition[0] = atan2(desiredXPosition[1],desiredXPosition[0]);

	//CORRECCION
	desiredXPosition[0] = desiredXPosition[0] -  aDH[0] * cos(desiredQPosition[0]);
	desiredXPosition[1] = desiredXPosition[1] -  aDH[0] * sin(desiredQPosition[0]);

	// FIN CORRECCION
	r = sqrt( desiredXPosition[0]*desiredXPosition[0] + desiredXPosition[1]*desiredXPosition[1] + (desiredXPosition[2] - dDH[0])*(desiredXPosition[2] - dDH[0]));

	
	//printf("d q %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);

	//printf("r= %f \n\f",r);

	if (r >= (dDH[2] + dDH[4]))
	{
	
		for(int i = 0;i<DOF;i++)
		{
			desiredQPosition[i] = currentQPosition[i];
			
		}

		//printf("regreso %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);
		return 0;
	}
	

	alpha = atan2((desiredXPosition[2] - dDH[0]), sqrt(pow(desiredXPosition[0],2) + pow(desiredXPosition[1],2))); 
	gamma = acos((-dDH[2] * dDH[2] - dDH[4] * dDH[4] + r * r) / (-2 * dDH[2] * dDH[4]));
	beta = asin(dDH[4] * sin(gamma) / r);
	
	

	//printf("agb: %f %f %f \n\f",alpha, gamma, beta);	

	//Esto siempre es considerando la soluci贸n de codo arriba, hay que checar que onda con la 
	//otra soluci贸n
	tunningRadiusElbow = dDH[2] * sin(beta);

	//printf("tunning= %f \n\f",tunningRadiusElbow);	

	//Posici贸n del codo con respecto al sistema Oelbow
	Pelbow.value[0] = 0;
	Pelbow.value[1] = -tunningRadiusElbow * cos(desiredXPosition[6]);
	Pelbow.value[2] = -tunningRadiusElbow * sin(desiredXPosition[6]);
	Pelbow.value[3] = 1;

	
	//printf("Pelbow  %f %f %f %f \n\r",Pelbow.value[0],Pelbow.value[1],Pelbow.value[2],Pelbow.value[3]);

	//Transformaci贸n del sistema sobre el que gira el codo al sistema base
	oRe.value[0][0] = cos(desiredQPosition[0]) * cos(-alpha);
	oRe.value[1][0] = sin(desiredQPosition[0]) * cos(-alpha);
	oRe.value[2][0] = -sin(-alpha);

	oRe.value[0][1] = -sin(desiredQPosition[0]);
	oRe.value[1][1] = cos(desiredQPosition[0]);
	oRe.value[2][1] = 0;

	oRe.value[0][2] = cos(desiredQPosition[0]) * sin(-alpha);
	oRe.value[1][2] = sin(desiredQPosition[0]) * sin(-alpha);
	oRe.value[2][2] = cos(-alpha);

	oRe.value[0][3] = dDH[2] * cos(beta) * cos(alpha) * cos(desiredQPosition[0]);
	oRe.value[1][3] = dDH[2] * cos(beta) * cos(alpha) * sin(desiredQPosition[0]);
	oRe.value[2][3] = dDH[2] * cos(beta) * sin(alpha) + dDH[0];
	oRe.value[3][3] = 1;


	//printf(" \n ");
	//printf(" oRe ");
	//printf("%f \t %f \t %f \t %f \t \n",oRe.value[0][0],oRe.value[0][1],oRe.value[0][2],oRe.value[0][3]);
	//printf("%f \t %f \t %f \t %f \t \n",oRe.value[1][0],oRe.value[1][1],oRe.value[1][2],oRe.value[1][3]);
	//printf("%f \t %f \t %f \t %f \t \n",oRe.value[2][0],oRe.value[2][1],oRe.value[2][2],oRe.value[2][3]);
	//printf("%f \t %f \t %f \t %f \t \n",oRe.value[3][0],oRe.value[3][1],oRe.value[3][2],oRe.value[3][3]);
	//printf(" \n ");


	Pelbow =  multMV(oRe,Pelbow); //Transformo coordenadas de posici贸n del codo con respecto al sistema base

	
	//printf("Pelbow  %f %f %f %f \n\r",Pelbow.value[0],Pelbow.value[1],Pelbow.value[2],Pelbow.value[3]);


	desiredQPosition[0] = atan2(Pelbow.value[1] + aDH[0] * sin(desiredQPosition[0]), Pelbow.value[0] + aDH[0] * cos(desiredQPosition[0]));
	desiredQPosition[1] = atan2(Pelbow.value[2] - dDH[0], sqrt(Pelbow.value[0] * Pelbow.value[0] + Pelbow.value[1] * Pelbow.value[1]));
	desiredQPosition[2] = 0;
	desiredQPosition[3] = 0;

	//printf("d q %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);
	
	CalculateHT(1);
	
	
	//R40 = calculateHomogenMatrix(4, 0);

	// De cero a tres por culpa de los 韓dices. En Calculate HT, al guardar las transformaciones de 0 a los sitemas 1-7, H0[0] no es identidad es H01
	R40 = InvH(H0[3]);
	


	//printf(" \n ");
	//printf(" R40 ");
	//printf("%f \t %f \t %f \t %f \t \n",R40.value[0][0],R40.value[0][1],R40.value[0][2],R40.value[0][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R40.value[1][0],R40.value[1][1],R40.value[1][2],R40.value[1][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R40.value[2][0],R40.value[2][1],R40.value[2][2],R40.value[2][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R40.value[3][0],R40.value[3][1],R40.value[3][2],R40.value[3][3]);
	//printf(" \n ");
	
	//Esta matriz tambi茅n se utiliza para obtener la posici贸n del centro de la mu帽eca con respecto al codo
	vector4 wp;
	
	//WristPosition = multMV(R40 , WristPosition);
	wp = multMV(R40 , WristPosition);
	
	
	//printf("WP  %f %f %f %f \n\r",wp.value[0],wp.value[1],wp.value[2],wp.value[3]);

	
	desiredQPosition[2] = atan2(wp.value[1],wp.value[0]);
	//desiredQPosition[3] = M_PI_2 - atan2(wp.value[2],
	//sqrt(wp.value[0] * wp.value[0] + wp.value[1]* wp.value[1]));
	
	desiredQPosition[3] = M_PI_2 - atan2(wp.value[2],sqrt(wp.value[0] * wp.value[0] + wp.value[1]* wp.value[1]));

	

	//printf("d q %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);



	CalculateHT(1);
	
	//A partir de aqu铆 se calculan los 谩ngulos de orientaci贸n
	//R40 = calculateHomogenMatrix(4, 0);
	R40 = InvH(H0[3]);

	
	
	//printf(" \n ");
	//printf(" R40 ");
	//printf("%f \t %f \t %f \t %f \t \n",R40.value[0][0],R40.value[0][1],R40.value[0][2],R40.value[0][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R40.value[1][0],R40.value[1][1],R40.value[1][2],R40.value[1][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R40.value[2][0],R40.value[2][1],R40.value[2][2],R40.value[2][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R40.value[3][0],R40.value[3][1],R40.value[3][2],R40.value[3][3]);
	//printf(" \n ");


	//R47 = R40 * R07;
	R47 = mult(R40 , R07);



	
	//printf(" \n ");
	//printf(" R47 ");
	//printf("%f \t %f \t %f \t %f \t \n",R47.value[0][0],R47.value[0][1],R47.value[0][2],R47.value[0][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R47.value[1][0],R47.value[1][1],R47.value[1][2],R47.value[1][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R47.value[2][0],R47.value[2][1],R47.value[2][2],R47.value[2][3]);
	//printf("%f \t %f \t %f \t %f \t \n",R47.value[3][0],R47.value[3][1],R47.value[3][2],R47.value[3][3]);
	//printf(" \n ");


	if ((R47.value[0][0] < 0.0001)&&(R47.value[0][0] > -0.0001))
	{
		desiredQPosition[4] = 0;
		desiredQPosition[5] = 0;
		desiredQPosition[6] = atan2(R47.value[1][1], R47.value[1][2]);
		//printf("d q1 %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);
	}
	else
	{
		desiredQPosition[5] = atan2(sqrt(1 - pow(R47.value[2][0], 2)), (R47.value[2][0]));
		desiredQPosition[4] = atan2(R47.value[1][0], R47.value[0][0]);
		desiredQPosition[6] = atan2(R47.value[2][2], -R47.value[2][1]);
		//printf("d q2 %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);
	}
				
	if (desiredQPosition[4] > 2.4)
	{
		desiredQPosition[4] -= M_PI;
		desiredQPosition[5] *= -1;
		if (desiredQPosition[6] > 0) desiredQPosition[6] -= M_PI;
		else desiredQPosition[6] += M_PI;

		//printf("d q3 %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);

	}
		if (desiredQPosition[4] < -2.4)
		{
			desiredQPosition[4] += M_PI;
			desiredQPosition[5] *= -1;
			if (desiredQPosition[6] > 0) desiredQPosition[6] -= M_PI;
			else desiredQPosition[6] += M_PI;

			//printf("d q4 %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);

		}

		CalculateHT(1);
		printf("dINV qF %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);
}


void TestMult()
{


	//printf("d 1 %f %f %f %f %f %f %f \n\r",desiredXPosition[0],desiredXPosition[1],desiredXPosition[2],desiredXPosition[3],desiredXPosition[4],desiredXPosition[5],desiredXPosition[6]);
	InverseKinematics();
	

	printf("d F %f %f %f %f %f %f %f \n\r",desiredQPosition[0],desiredQPosition[1],desiredQPosition[2],desiredQPosition[3],desiredQPosition[4],desiredQPosition[5],desiredQPosition[6]);
	

	
}

matrix4 InvH(matrix4 x)
{
	matrix4 hi;
	vector3 d;
	float suma;

	for(int i = 0; i<4; i++)
	{
		for(int j = 0; j<4; j++)
		{
			hi.value[i][j] = x.value[i][j];
		}
	}

	for(int i = 0; i<3; i++)
	{
		for(int j = 0; j<3; j++)
		{
			hi.value[i][j] = x.value[j][i];
		}
	}

	for(int i = 0; i<3; i++)
	{
		d.value[i] = x.value[i][3];

	}

	for(int i = 0; i<3; i++)
	{
		suma = 0;
		for(int j= 0; j<3; j++)
		{
			suma = suma + hi.value[i][j] * d.value[j];
		}
		hi.value[i][3] = -suma;
	}
	return hi;
}

///////////////////////////////// TRAYECTORIAS


vector4 coefLSPB (float q0, float qf, float tf)
{

	
	float Vmin = (qf-q0)/tf;
	float Vmax = (2*(qf-q0))/tf;

	float V = (Vmin + Vmax)/2;
	float tb = tf/3;
	float a = V/tb;

	vector4 data;

	data.value[0]= V;
	data.value[1]= a;
	data.value[2]= tb;
	data.value[3]= tf;
	
	return data;
	

} 

float evalLSPB(float q0,float qf,vector4 data,float t)

{
	float V,a,tb,tf;
	float q;

	V = data.value[0];
	a = data.value[1];
	tb = data.value[2];
	tf = data.value[3];

    if(t <= tb)
        q = q0 + (0.5*a*pow(t,2));

    else if((t>tb) && (t<=(tf-tb)))
        q = ((qf-q0-V*tf)/2)+V*t + q0;

    else
        q = qf -((a*pow(tf,2))/2) + a*tf*t - (a/2)*pow(t,2);

	return q;

}
vector4 coefQuintico (float q0, float qf, float tf)
{
	float a0 = q0;
	float a3 = 10*(qf-q0)/(pow(tf,3));
	float a4 = -15*(qf-q0)/(pow(tf,4));
	float a5 = 6*(qf-q0)/(pow(tf,5));
	vector4 data;
	data.value[0]= a0;
	data.value[1]= a3;
	data.value[2]= a4;
	data.value[3]= a5;
	return data;
}

float perQuintico(float q0,float qf,vector4 data,float t)
{
	float a0,a3,a4,a5;
	float q;
	a0 = data.value[0];
	a3 = data.value[1];
	a4 = data.value[2];
	a5 = data.value[3];
	q=a0+a3*pow(t,3)+a4*pow(t,4)+a5*pow(t,5);
	return q;
}
