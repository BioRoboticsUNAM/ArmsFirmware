#include <avr/io.h>
#include <stdio.h>
#include <math.h>
//#include "control.c"


/////// KINEMATICS

/// VARIABLES

float aDH[DOF];				// a DH parameter
float dDH[DOF];				// d DH parameter
float alphaDH[DOF];			// alpha DH parameter
float thetaDH[DOF];			// theta DH parameter
float qDH[DOF];				// theta DH parameter
float H[DOF+1][4][4];			// Homogenean Matrices. DOF+1 Incluye la última rotación para alinear la orientación del gripper
float H07[4][4];				// Homogenean Transformation from 0 to 7. (0 a 8 incluyendo la última rotación)
float cPos[DOF];
float H0[DOF+1][4][4];		// H.T. from 0 to 1, 0 to 2, 0 to 3 up to 0 to 7

void SetKConstants();
void CalculateDH();
void CalculateHT();
void ForwardKinematics();



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
		cPos[i] = 0;
		for(int j = 0;j<4;j++)
		{
			for(int k = 0;k<4;k++)
			{
				H[i][j][k] = 0;	
				H07[j][k] = 0;
				H0[i][j][k] = 0;	
			}
		}
	}

	for(int j = 0;j<4;j++)
	{
		for(int k = 0;k<4;k++)
		{
			H07[j][k] = 0;
			H[7][j][k] = 0;	
			H0[7][j][k] = 0;	
		}
	}
	
	dDH[0] = 0.13;
	dDH[2] = 0.3084;
	dDH[4] = 0.2126;
	dDH[6] = 0.19;
	
	alphaDH[0]= M_PI_2;
	alphaDH[1]= M_PI_2;
	alphaDH[2]= -M_PI_2;
	alphaDH[3]= M_PI_2;
	alphaDH[4]= -M_PI_2;
	alphaDH[5]= M_PI_2;

	thetaDH[1] = M_PI_2;
	thetaDH[2] = -M_PI_2;

	H[7][2][0] = 1;
	H[7][0][1] = 1;
	H[7][1][2] = 1;
	H[7][3][3] = 1;


}

void CalculateDH()
{
	for(int i = 0; i < DOF;i++)
	{
		qDH[i] = currentPosition[i] + thetaDH[i];
	}

	for (int i = 0; i < DOF ; i++)
	{
	
		H[i][0][0] = cos(qDH[i]);
		H[i][0][1] = -sin(qDH[i]) * cos(alphaDH[i]);
		H[i][0][2] = sin(qDH[i]) * sin(alphaDH[i]);
		H[i][0][3] = aDH[i] * cos(qDH[i]);

		H[i][1][0] = sin(qDH[i]);
		H[i][1][1] = cos(qDH[i]) * cos(alphaDH[i]);
		H[i][1][2] = -cos(qDH[i]) * sin(alphaDH[i]);
		H[i][1][3] = aDH[i] * sin(qDH[i]);

		H[i][2][1] = sin(alphaDH[i]);
		H[i][2][2] = cos(alphaDH[i]);
		H[i][2][3] = dDH[i];

		H[i][3][3] = 1;

	}
}


void CalculateHT()
{
	CalculateDH();

	float temp[4];

	for(int j = 0;j<4;j++)
	{
		for(int k = 0;k<4;k++)
		{
			H07[j][k] = 0;
		}
	}

	for(int i = 0;i<4;i++)	// Matriz Identidad
	{
		H07[i][i] = 1;
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
					temp[k] += (H07[j][l] * H[i][l][k]);
				}
			}
			for(int h = 0; h <4 ;h++)
			{
				H07[j][h] = temp[h];
			}		
		}
		for(int k = 0;k<4;k++)
			{
				for(int l = 0;l<4;l++)
				{
					H0[i][k][l] = H07[k][l];
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
	
	CalculateHT();
	cPos[0] = H07[0][3];
	cPos[1] = H07[1][3];
	cPos[2] = H07[2][3];

	cPos[3] = atan2(H07[1][0],H07[0][0]);
	cPos[4] = atan2(-H07[2][0],sqrt(1 - (H07[2][0]*H07[2][0]) ));
	cPos[5] = atan2(H07[2][1],H07[2][2]);

	cPos[6] = 0;

	printf("%f \t %f \t %f \t %f \t %f \t %f \t %f \n",cPos[0],cPos[1],cPos[2],cPos[3],cPos[4],cPos[5],cPos[6]);


}
