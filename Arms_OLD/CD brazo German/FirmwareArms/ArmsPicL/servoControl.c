#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include "cmdATM.h"
#include <util/delay.h>
#include "dynamixel.h"

// Servo Control

#define SERVO_GOAL_POSITION_L		30
#define SERVO_PROG_SPEED			32
#define SERVO_PRESENT_POSITION_L	36
#define SERVO_PRESENT_LOAD_L		40
#define SERVO_VOLTAGE				42
#define SERVO_MAX_CW				6
#define SERVO_MAX_CCW				8
//#define DOF							7
#define DOF							7




/////// CONTSTNTS

float ceroPosition[DOF+2];// = {90,90,90};
int cw[DOF+2];// = {1, 1, 1};
float maxBits[DOF+2];// = {4095.0,1023.0,1023.0};
float maxDegrees[DOF+2];// = {250.92, 300.0, 300.0};
int ids[DOF+2];

/////// VARIABLES
int printStatus = 0;
float maxTorque[DOF+2];
int CommStatus;



/////// 
void  SetUpServos();
int   SetPosition(int id, float position);
void  SetSpeedPerc(int id, float speed);
void  SetTorqueControl(int id, int enable);
int   SetTorque(int id, float torque);
void  SetTorque2(int id, float torque);
void  TorqueEnabled(int id, int enable);
int   ServoTorqueEnabled(int e1,int e2,int e3,int e4,int e5,int e6,int e7);
void  ArmTorqueEnabled(int enable);
float GetVoltage(int id);
void  ArmPosition(float* servoPos);
void  ArmTorque(float* servoTorque);
void  ArmTorqueControl(int enable);
float* ArmGetPosition();
float GetPosition(int id);
float GetLoad(int id);
int   PrintErrorCode();
void  PrintCommStatus(int CommStatus);

///////
void SetUpServos()
{	
	float voltage12 = 0;
	voltage12 = GetVoltage(7);
	printf("Voltage12: %f \r",voltage12);
	_delay_ms(50);
	float voltage18 = 0;
	voltage18 = GetVoltage(0);
	printf("Voltage18: %f \r",voltage18);
	

	ids[0] = 0;
	ids[1] = 1;
	ids[2] = 2;
	ids[3] = 3;
	ids[4] = 4;
	ids[5] = 5;
	ids[6] = 6;

	ids[7] = 7;
	ids[8] = 17;

	// Arm
	ceroPosition[0] = 81;   // 33
	ceroPosition[1] = 0;	//180
	ceroPosition[2] = 90;
	ceroPosition[3] = 90; //
	ceroPosition[4] = 90;
	ceroPosition[5] = 90;
	ceroPosition[6] = 90;

	ceroPosition[7] = 120;
	ceroPosition[8] = 238;
	
	//1: CW 0: CCW
	cw[0] = 1;
	cw[1] = 0;//1
	cw[2] = 0;
	cw[3] = 1;
	cw[4] = 0;
	cw[5] = 1;
	cw[6] = 0;

	cw[7] = 0;
	cw[8] = 1;

	maxBits[0] = 4095.0;
	maxBits[1] = 4095.0;   ////// CAMBIAR a 64
	maxBits[2] = 1023.0;
	maxBits[3] = 4095.0;
	maxBits[4] = 1023.0;
	maxBits[5] = 1023.0;
	maxBits[6] = 1023.0;
	maxBits[7] = 1023.0;
	maxBits[8] = 1023.0;

	maxDegrees[0] = 250.92;
	maxDegrees[1] = 250.92; ////// CAMBIAR a 64
	maxDegrees[2] = 300.0;
	maxDegrees[3] = 359.999;
	maxDegrees[4] = 300.0;
	maxDegrees[5] = 300.0;
	maxDegrees[6] = 300.0;

	maxDegrees[7] = 300.0;
	maxDegrees[8] = 300.0;

	float maxTorque106 = (voltage18 * 5.945945946) - 4.0;		// EX-106
	float maxTorque64  = (voltage18 * 4.266666667) + 0.4;		// RX-64
	float maxTorque28  = (voltage18 * 1.446153846) + 10.9461;
	float maxTorque12  = (voltage12 * 1.5) + 1.5;


	maxTorque[0] = maxTorque106;
	maxTorque[1] = maxTorque106;			///// CAMBIAR A 64
	maxTorque[2] = maxTorque28;
	maxTorque[3] = maxTorque106;
	maxTorque[4] = maxTorque64;
	maxTorque[5] = maxTorque64;
	maxTorque[6] = maxTorque28;

	maxTorque[7] = maxTorque12;
	maxTorque[8] = maxTorque12;
	
	printf("MAXTORQUE106: %f \r",maxTorque106);
	printf("MAXTORQUE64: %f \r",maxTorque64);
	printf("MAXTORQUE28: %f \r",maxTorque28);
	printf("MAXTORQUE12: %f \r",maxTorque12);

}



///// METHODS
int SetPosition(int id, float position)
{
	int bits;
	if(cw[id]==1)
	{
	bits = ((maxDegrees[id] -  ceroPosition[id] - ((maxDegrees[id] - 180.0) / 2.0) + (position * 57.2957)) * (maxBits[id] / maxDegrees[id]));
	bits = maxBits[id] - bits;
	//printf("CW ");
	}
	else if(cw[id] == 0){
	bits = ((position * 57.2957) + ceroPosition[id] + ((maxDegrees[id] - 180.0) / 2.0)) * (maxBits[id] / maxDegrees[id]);
	//printf("CCW ");
	}
	//if(bits>1023) bits = 1023;
	//else if(bits<0) bits = 0;
	//printf("BITS: %i", bits);
	//dxl_write_word( id, SERVO_GOAL_POSITION_L, bits );
	//printf("Id %d, Position: %f, PosBits = %d \n\f",id,position,bits);
	return bits;
}

void ArmPosition(float* servoPos)
{
		int i;
		int GoalPos;
		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, SERVO_GOAL_POSITION_L);
		dxl_set_txpacket_parameter(1, 2);
		for( i=0; i<DOF; i++ )
		{
			dxl_set_txpacket_parameter(2+3*i, i);
			GoalPos = (int)(SetPosition(i,servoPos[i]));
			//printf( "%i  ", GoalPos );
			dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos));
			dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos));
		}
		dxl_set_txpacket_length((2+1)*DOF+4);

		//printf("\n");
		dxl_txrx_packet();


		CommStatus = dxl_get_result();
		if( CommStatus == COMM_RXSUCCESS )
			PrintErrorCode();
		else
			PrintCommStatus(CommStatus);



}

int GetBits(int id)
{
	return dxl_read_word( id, SERVO_PRESENT_POSITION_L );
}

float GetPosition(int id)
{
	int bits;
	float position;
 	bits = dxl_read_word( ids[id], SERVO_PRESENT_POSITION_L );
	
	if(cw[id]==1)
	{
	    position = ( ( ceroPosition[id] +  ((maxDegrees[id] - 180.0) / 2.0)) - ((bits * maxDegrees[id])/maxBits[id]) );
	}
	else
	{
		position = ( ( (bits * maxDegrees[id])/maxBits[id])-( ceroPosition[id]+((maxDegrees[id] - 180.0) / 2.0) ) );
	}

	return position * 0.0174532925199;
}

float GetLoad(int id)
{
	int bits;
	float load;
	int sign = 0;

 	bits = dxl_read_word( id, SERVO_PRESENT_LOAD_L );
	//printf("LoadBits: %d \n\r",bits);

	if( (bits & 1024) == 1024){sign = -1;}
	if( (bits & 1024) == 0   ){sign =  1;}

	if(cw[id] ==1) {sign = sign * -1;}

	bits = bits & 1023;
	
	load = sign * (bits * 100.0 / 1023.0);


	return  load;

}


float* ArmGetPosition()
{
	float positions[DOF];

	for(int i = 0; i<DOF;i++)
	{
		positions[i] = GetPosition(i);
		
	}
	return positions;
}

void SetMaxSpeeds(float speed)
{
	for(int i = 0; i<7; i++)
	{
		SetSpeedPerc(i,speed);
		_delay_ms(10);
	}
	

}

void SetSpeedPerc(int id, float speed)
{
	int bits;
	bits = speed * (1023.0 / 100.0);
	//printf("Speed %i",bits);
	if(bits>1023) bits = 1023;
	else if(bits<=1) bits = 1;
	//printf("Speed %i",bits);
	dxl_write_word( id, SERVO_PROG_SPEED , bits );
}


void ArmTorqueControl(int enable)
{

	for(int i = 0;i<DOF;i++)
	{
		SetTorqueControl(i,enable);
	}
}

void SetTorqueControl(int id, int enable)
{
	
	//dxl_write_byte(2, 16, 1);
		
	SetSpeedPerc(ids[id],0);

	if(enable==1)
	{
		
		dxl_write_word( ids[id], SERVO_MAX_CW , 0 );
		dxl_write_word( ids[id], SERVO_MAX_CCW , 0 );
	}else if(enable==0){
		
		dxl_write_word( ids[id], SERVO_MAX_CW , 1 );
		dxl_write_word( ids[id], SERVO_MAX_CCW , maxBits[id] );
	}
}



int SetTorque(int id, float torque)
{
	int direction,bits,bitsF2;
	float bitsF;
	

	if(cw[id]==1)
	{
		if(torque<=0) {direction = 1;}
		else{direction = 0;}
		
	}
	else
	{	
		if(torque<=0) {direction = 0;}
		else{direction = 1;}
	}
	
	torque = abs(torque);
	
	bits = (int)(torque*(1023.0 / 100.0));
	
	//printf("torqueBits1: %d \n\r",bits);	
	
		
	
	if(bits>1023) bits = 1023;
	else if(bits<1) bits = 1;

	if(direction ==1){ bits = bits + 1024;}

	//dxl_write_word( id, SERVO_PROG_SPEED , bits );
	return bits;
}


void SetTorque2(int id, float torque)
{
	int direction,bits;

	if(cw[id]==1)
	{
		if(torque<=0) {direction = 1;}
		else{direction = 0;}
		
	}
	else
	{	
		if(torque<=0) {direction = 0;}
		else{direction = 1;}
	}
	torque = abs(torque);

	bits = torque * (1023.0 / maxTorque[id]);
	

	if(bits>1023) bits = 1023;
	else if(bits<0) bits = 0;

	if(direction ==1){ bits = bits + 1024;}

	dxl_write_word( id, SERVO_PROG_SPEED , bits );
	//return bits;
}

void TorqueEnabled(int id, int enable)
{
	dxl_write_byte( id, 24 , enable );
}

void ArmTorqueEnabled(int enable)
{
	for(int i = 0;i<DOF;i++)
	{
		dxl_write_byte( i, 24 , enable );
	}
}

int ServoTorqueEnabled(int e1,int e2,int e3,int e4,int e5,int e6,int e7)
{
	
	dxl_write_byte( 0, 24 , e1);
	dxl_write_byte( 1, 24 , e2);
	dxl_write_byte( 2, 24 , e3);
	dxl_write_byte( 3, 24 , e4);
	dxl_write_byte( 4, 24 , e5);
	dxl_write_byte( 5, 24 , e6);
	dxl_write_byte( 6, 24 , e7);
	return 1;
}

void ArmTorque(float* servoTorque)
{
		int i;
		int GoalTorque;
		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, SERVO_PROG_SPEED);
		dxl_set_txpacket_parameter(1, 2);
		for( i=0; i<DOF; i++ )
		{
			dxl_set_txpacket_parameter(2+3*i, i);
			GoalTorque = (int)(SetTorque(i,servoTorque[i]));
			//printf( "%i  ", GoalTorque );
			dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalTorque));
			dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalTorque));
		}
		dxl_set_txpacket_length((2+1)*DOF+4);

		//printf("\n");
		dxl_txrx_packet();


		
		if(printStatus==1)
		{
			CommStatus = dxl_get_result();
			if( CommStatus == COMM_RXSUCCESS )
			{
				PrintErrorCode();
			}
		
			else
			{
				PrintCommStatus(CommStatus);
			
			//PORTC ^= 0b0010000;
			}
		}
}

float GetVoltage(int id)
{
	float voltageBits = 0;
	 voltageBits = dxl_read_byte( id, SERVO_VOLTAGE  );
	float voltage = 0;
	 voltage = voltageBits / 10.0;
	return voltage;
}


int Ping(int id)
{

	dxl_ping(id);

	CommStatus = dxl_get_result();
		if( CommStatus == COMM_RXSUCCESS )
		{
		return 	PrintErrorCode();
		}
		
		else
		{
			//PrintCommStatus(CommStatus);
			return 0;
		}
}


//Returns de exact error code
int Ping2(int id)
{
	int errT = 1;

	dxl_ping(id);
	
	if(dxl_get_rxpacket_error(1)==1) // Error de Voltaje
	errT = errT*2;
	if(dxl_get_rxpacket_error(4)==1) // Error de SobreCalentamiento
	errT = errT*3;
	if(dxl_get_rxpacket_error(32)==1) // Error de SobreCarga
	errT = errT*5;

	return errT;
}

//Returns the error code of every servo
void StatusReport()
{
	int p0,p10,p1,p2,p3,p4,p5,p6,p7;
	p0 = Ping2(0);
	p10 = Ping2(10);
	p1 = Ping2(1);
	p2 = Ping2(2);
	p3 = Ping2(3);
	p4 = Ping2(4);
	p5 = Ping2(5);
	p6 = Ping2(6);
	p7 = Ping2(7);

	printf("S 1 %d %d %d %d %d %d %d %d %d\r",p0,p1,p2,p3,p4,p5,p6,p7,p10);
}

void ArmPing()
{
	int p0,p1,p2,p3,p4,p5,p6;
	p0 = Ping(0);
	p1 = Ping(1);
	p2 = Ping(2);
	p3 = Ping(3);
	p4 = Ping(4);
	p5 = Ping(5);
	p6 = Ping(6);


	printf("i 1 %d %d %d %d %d %d %d \r",p0,p1,p2,p3,p4,p5,p6);
}
// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		//printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		printf("txf\r");
		break;

	case COMM_TXERROR:
		//printf("COMM_TXERROR: Incorrect instruction packet!\n");
		printf("txe\r");
		break;

	case COMM_RXFAIL:
		//printf("COMM_RXFAIL: Failed get status packet from device!\n");
		printf("rxf\r");
		break;

	case COMM_RXWAITING:
		//printf("COMM_RXWAITING: Now recieving status packet!\n");
		printf("rxz\r");
		break;

	case COMM_RXTIMEOUT:
		//printf("COMM_RXTIMEOUT: There is no status packet!\n");
		printf("rxo\r");
		break;

	case COMM_RXCORRUPT:
		//printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		printf("rxs\r");
		break;

	default:
		//printf("This is unknown error code!\n");
		printf("rxx\r");
		break;
	}
}

// Print error bit of status packet
int PrintErrorCode()
{
	
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
	{
		PORTC ^= 0b0010000;
		// Voltage Error
		//printf("vo\r");
		return 0;
	}
	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
	{
		PORTC ^= 0b0010000;
		// Angle Limit error
		//printf("al\r");
		return 0;
	}
	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
	{
		PORTC ^= 0b0010000;
		// Overheat error
		//printf("oh\r");
		return 0;
	}
	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
	{
		PORTC ^= 0b0010000;
		// Range error
		//printf("or\r");
		return 0;
	}
	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
	{
		PORTC ^= 0b0010000;
		//Checksum error
		//printf("ch\r");
		return 0;
	}
	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
	{
		PORTC ^= 0b0010000;
		//OverLoad error
		//printf("ol\r");
		return 0;
	}
	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
	{
		PORTC ^= 0b0010000;
		// Instruction error
		//printf("in\r");
		return 0;
	}
	return 1;
}


