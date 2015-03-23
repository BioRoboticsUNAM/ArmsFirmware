#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "servoControl.c"
//#include "kinematics.c"


/////// ARM CONTROL VARIABLES
int doLoop = 0;
int doReading = 1;
int doControl = 0;

float desiredPosition[DOF];
float currentPosition[DOF];
float lastPosition[DOF];
float differentialPosition[DOF];
float kp[DOF];
float ki[DOF];
float tau[DOF];
static float ei[DOF];
float e[DOF];
float Qj[4][4];
float U0[7][4][4];
float U1[6][4][4];
float U2[5][4][4];
float U3[4][4][4];
float U4[3][4][4];
float U5[2][4][4];
float U6[1][4][4];

/////// GRIPPER CONTROL VARIABLES

int doGripper = 0;

/////// ARM METHODS
void executeControl();
void SetDesiredPosition(float position0,float position1,float position2);
void SetConstants();
void ConstructU();
float* mult(float* a,float* b);
void TestMult();

/////// GRIPPER METHODS
void executeGripper();

//////// ARM METHODS

void SetConstants()
{
	kp[0] = 20;
	kp[1] = 20;
	kp[2] = 20;

	ki[0] = 0.01;
	ki[1] = 0.01;
	ki[2] = 0.01;

	for(int i = 0;i<DOF;i++)
	{
	e[i] = 0;
	ei[i] = 0;
	desiredPosition[i] = 0;
	currentPosition[i] = 0;
	lastPosition[i] = 0;
	differentialPosition[i] = 0;
	}

	for(int i = 0;i<4;i++)
	{
		for(int j = 0; j<4; j++)
		{
			Qj[i][j] = 0;
		}
	}

	Qj[0][1] = -1;
	Qj[1][0] = 1;

	for(int i = 0; i<DOF;i++)
	{
		for(int j = 0; j<4;j++)
		{
			for(int k = 0;k<4;k++)
			{
				U0[i][j][k] = 0;
			}
		}
	}
	for(int i = 0; i<6;i++)
	{
		for(int j = 0; j<4;j++)
		{
			for(int k = 0;k<4;k++)
			{
				U1[i][j][k] = 0;
			}
		}
	}
	for(int i = 0; i<5;i++)
	{
		for(int j = 0; j<4;j++)
		{
			for(int k = 0;k<4;k++)
			{
				U2[i][j][k] = 0;
			}
		}
	}
	for(int i = 0; i<4;i++)
	{
		for(int j = 0; j<4;j++)
		{
			for(int k = 0;k<4;k++)
			{
				U3[i][j][k] = 0;
			}
		}
	}
	for(int i = 0; i<3;i++)
	{
		for(int j = 0; j<4;j++)
		{
			for(int k = 0;k<4;k++)
			{
				U4[i][j][k] = 0;
			}
		}
	}
	for(int i = 0; i<2;i++)
	{
		for(int j = 0; j<4;j++)
		{
			for(int k = 0;k<4;k++)
			{
				U5[i][j][k] = 0;
			}
		}
	}
	
	for(int j = 0; j<4;j++)
	{
		for(int k = 0;k<4;k++)
		{
			U6[1][j][k] = 0;
		}
	}
	
}
void executeControl()
{
	//currentPosition = GetPosition(1);
	//currentPosition = ArmGetPosition();

	for(int i = 0; i<DOF;i++)
	{
		if(doReading)
		{
			currentPosition[i] = GetPosition(i);

			if(abs(currentPosition[i] - lastPosition[i])>0.5)
			{
		 	currentPosition[i] = (lastPosition[i] + differentialPosition[i]);
			
		 	//printf("Current = %f  %f  %f \n",currentPosition[0],currentPosition[1],currentPosition[2]);
		 	//PORTC ^= 0b0100000;
		 	
		 	}
			ForwardKinematics();
		}		

		if(doControl && doReading)
		{
		e[i] = desiredPosition[i] - currentPosition[i];
		ei[i] = ei[i] + e[i];
		tau[i] = (kp[i] * (e[i])) + (ki[i] * ei[i]);
		//_delay_ms(1);
		
		}
		lastPosition[i] = currentPosition[i];
		differentialPosition[i] = currentPosition[i] - lastPosition[i];
	}
		
	if(doControl && doReading)
	{	
		ArmTorque(tau);
	}
}

void SetDesiredPosition(float position0,float position1,float position2)
{
	//desiredPosition = position;
	desiredPosition[0] = position0;
	desiredPosition[1] = position1;
	desiredPosition[2] = position2;

	//for(int i = 0;i<DOF;i++)
	//{
	//	printf("Desired %i = %f ",i,desiredPosition[i]);
	//}
}

////// GRIPPER METHODS

void executeGripper()
{

}

///// Gravity Compensator

void ConstructU()
{

float temp[4];

for(int i = 0;i<DOF;i++)
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
					temp[k] += (Qj[j][l] * H0[i][l][k]);
				}
			}
			for(int h = 0; h <4 ;h++)
			{
				U0[i][j][h] = temp[h];
			}		
		}
	}

for(int i = 0;i<6;i++)
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
					temp[k] += (Qj[j][l] * H0[i][l][k]);
				}
			}
			for(int h = 0; h <4 ;h++)
			{
				U1[i][j][h] = temp[h];
			}		
		}
	}
}

float* mult(float* a,float* b)
{
	float temp[4];
	float response[4][4];
	
	for(int j = 0; j<4;j++)
		{
		temp[j] = 0;
			for(int k = 0;k<4;k++)
			{
				response[j][k] = 0;
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
					temp[k] += (a[j][l] * b[l][k]);
				}
			}
			for(int h = 0; h <4 ;h++)
			{
				response[j][h] = temp[h];
			}		
		}	
return response;

}


void TestMult()
{	
	float response[4][4];

	
	float temp1[4][4] = 
	{
	{1,2,3,4},
	{1,2,3,4},
	{1,2,3,4},
	{1,2,3,4}
	};

	float temp2[4][4] = 
	{
	{1,2,3,4},
	{1,2,3,4},
	{1,2,3,4},
	{1,2,3,4}
	};
response = mult(temp1,temp2);

		printf("%f \t %f \t %f \t %f \t \n",response[0][0],response[0][1],response[0][2],response[0][3]);
		printf("%f \t %f \t %f \t %f \t \n",response[1][0],response[1][1],response[1][2],response[1][3]);
		printf("%f \t %f \t %f \t %f \t \n",response[2][0],response[2][1],response[2][2],response[2][3]);
		printf("%f \t %f \t %f \t %f \t \n",response[3][0],response[3][1],response[3][2],response[3][3]);
		printf(" \n ");

}
