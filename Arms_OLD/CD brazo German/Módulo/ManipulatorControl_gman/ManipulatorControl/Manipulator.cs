using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
using System.Threading;
using System.IO.Ports;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public enum ArmType {LeftArm, RightArm};
    public delegate void ArmPositionChangedEH(Manipulator senderArm);
    public class Manipulator
    {
        private ArmType armType;
		private int dof;
        private ServoControl[] servos;
        private double[] DHd;
        private double[] DHa;
        private double[] DHalpha;
        private double[] DHtheta;
        private Matrix4[] DH;

        private SerialPortManager spm;
        private double[] constants;
        private double[] senses;
        private double[] centers;
		private double[] maxBits;
		private double[] maxDegrees;

        private Vector currentPosCartesian;
        private Vector currentPosArticular;
        private Vector desPosCartesian;
        private Vector desPosArticular;
        private Vector lastCartesianPosition;
        private double[] lowerServoLimits;
        private double[] upperServoLimits;

        private SerialPort serialPort;

        public Manipulator(SerialPort serialPort, ArmType armType)
        {
            this.armType = armType;
            this.serialPort = serialPort;
			this.dof = 7;
			this.spm = new SerialPortManager();
			this.InitializeServos();
            this.InitializeDH();
            this.currentPosCartesian = new Vector(7);
            this.currentPosArticular = new Vector(7);
            this.desPosCartesian = new Vector(7);
            this.desPosArticular = new Vector(7);
            this.lastCartesianPosition = new Vector(7);
        }

        private void InitializeDH()
        {
            this.DHd = new double[7];
            this.DHa = new double[7];
            this.DHalpha = new double[7];
            this.DHtheta = new double[7];
            this.DH = new Matrix4[7];

            for (int i = 0; i < 7; i++)
            {
                this.DHd[i] = 0;
                this.DHa[i] = 0;
                this.DHalpha[i] = 0;
                this.DHtheta[i] = 0;
                this.DH[i] = Matrix4.Zero;
            }

            this.DHd[2] = 0.285;
            this.DHd[4] = 0.2126;
            this.DHd[6] = 0.12;

            this.DHa[0] = 0.0603;

            this.DHalpha[0] = Math.PI / 2;
            this.DHalpha[1] = Math.PI / 2;
            this.DHalpha[2] = -Math.PI / 2;
            this.DHalpha[3] = Math.PI / 2;
            this.DHalpha[4] = -Math.PI / 2;
            this.DHalpha[5] = Math.PI / 2;

            this.DHtheta[1] = Math.PI / 2;
            this.DHtheta[2] = -Math.PI / 2;
        }

		private void InitializeServos()
		{
            this.constants = new double[7];
            this.constants[0] = (4095) / (251 * Math.PI / 180);
            this.constants[1] = (4095) / (251 * Math.PI / 180);
            this.constants[2] = (1023) / (300 * Math.PI / 180);
            this.constants[3] = (4095) / (251 * Math.PI / 180);
            this.constants[4] = (1023) / (300 * Math.PI / 180);
            this.constants[5] = (1023) / (300 * Math.PI / 180);
            this.constants[6] = (1023) / (300 * Math.PI / 180);

            this.senses = new double[7];
            if (this.armType == ArmType.RightArm)
            {
                this.senses[0] = 1;
                this.senses[1] = 1;
                this.senses[2] = 1; // -1
                this.senses[3] = -1;
                this.senses[4] = 1; // -1
                this.senses[5] = -1;//  1
                this.senses[6] = 1; // -1
            }
            else
            {
                this.senses[0] = -1;
                this.senses[1] = 1; //-1
                this.senses[2] = 1;
                this.senses[3] = -1;
                this.senses[4] = 1;
                this.senses[5] = -1;
                this.senses[6] = 1;
            }

            this.centers = new double[7];
            if (this.armType == ArmType.RightArm)
            {
                this.centers[0] = 1588;
                this.centers[1] = 580;
                this.centers[2] = 509;
                this.centers[3] = 2372;
				this.centers[4] = 589;
                this.centers[5] = 512; 
                this.centers[6] = 509;
            }
            else
            {
                this.centers[0] = 1906;
                this.centers[1] = 585;
                this.centers[2] = 525;
                this.centers[3] = 2523;
                this.centers[4] = 512;
                this.centers[5] = 512;
                this.centers[6] = 512;
            }

			this.maxBits = new double[7];
			this.maxBits[0] = 4095; //Estaban mal, tenian 4096
			this.maxBits[1] = 4095;
			this.maxBits[2] = 1023;
			this.maxBits[3] = 4095;
			this.maxBits[4] = 1023;
			this.maxBits[5] = 1023;
			this.maxBits[6] = 1023;

			this.maxDegrees = new double[7];
			this.maxDegrees[0] = 250.92;
			this.maxDegrees[1] = 250.92;
			this.maxDegrees[2] = 300;
			this.maxDegrees[3] = 250.92;
			this.maxDegrees[4] = 300;
			this.maxDegrees[5] = 300;
			this.maxDegrees[6] = 300;


			this.servos = new ServoControl[8];

            this.servos[0] = new ServoControl(ServoType.EX106, 0, 57, false, spm);
            this.servos[1] = new ServoControl(ServoType.EX106, 1, 180, true, spm);
            this.servos[2] = new ServoControl(ServoType.RX64, 2, 90, false, spm);
            this.servos[3] = new ServoControl(ServoType.EX106, 3, 115, true, spm);
            this.servos[4] = new ServoControl(ServoType.RX64, 4, 90, false, spm);
            this.servos[5] = new ServoControl(ServoType.RX64, 5, 90, false, spm);
            this.servos[6] = new ServoControl(ServoType.AX12, 6, 90, false, spm); //antes 28
            this.servos[7] = new DualServoControl(ServoType.AX12, 7, 25, false, false, spm);

            this.lowerServoLimits = new double[7];
            this.upperServoLimits = new double[7];

			for (int i = 0; i < 7; i++)
			{
				if (this.senses[i] > 0)
				{
					this.lowerServoLimits[i] = -this.centers[i] / this.maxBits[i] * (this.maxDegrees[i] * Math.PI / 180);
					this.upperServoLimits[i] = (this.maxBits[i] - this.centers[i]) / this.maxBits[i] * (this.maxDegrees[i] * Math.PI / 180);
				}
				else
				{
					this.lowerServoLimits[i] = -(this.maxBits[i]- this.centers[i]) / this.maxBits[i] * (this.maxDegrees[i] * Math.PI / 180);
					this.upperServoLimits[i] = this.centers[i] / this.maxBits[i] * (this.maxDegrees[i] * Math.PI / 180);
				}
			}
			/*
			this.lowerServoLimits[1] = -125 * Math.PI / 180;
            this.lowerServoLimits[2] = -150 * Math.PI / 180;
            this.lowerServoLimits[3] = -111 * Math.PI / 180;
            this.lowerServoLimits[4] = -150 * Math.PI / 180;
            this.lowerServoLimits[5] = -150 * Math.PI / 180;
            this.lowerServoLimits[6] = -150 * Math.PI / 180;

            this.upperServoLimits[0] = 125 * Math.PI / 180;
            this.upperServoLimits[1] = 125 * Math.PI / 180;
            this.upperServoLimits[2] = 150 * Math.PI / 180;
            this.upperServoLimits[3] = 139 * Math.PI / 180;
            this.upperServoLimits[4] = 150 * Math.PI / 180;
            this.upperServoLimits[5] = 150 * Math.PI / 180;
            this.upperServoLimits[6] = 150 * Math.PI / 180;*/
        }

        private void CheckPositionChangedEvent()
        {
            if ((this.currentPosCartesian - this.lastCartesianPosition).Magnitude > 0.05)
            {
                this.lastCartesianPosition[0] = this.currentPosCartesian[0];
                this.lastCartesianPosition[1] = this.currentPosCartesian[1];
                this.lastCartesianPosition[2] = this.currentPosCartesian[2];
                this.lastCartesianPosition[3] = this.currentPosCartesian[3];
                this.lastCartesianPosition[4] = this.currentPosCartesian[4];
                this.lastCartesianPosition[5] = this.currentPosCartesian[5];
                this.lastCartesianPosition[6] = this.currentPosCartesian[6];
                this.OnArmPositionChnaged(this);
            }
        }
	
        private void CalculateDH(Vector q)
        {
            Matrix4 temp = Matrix4.Identity;
            for (int i = 0; i < 7; i++)
            {
                this.DH[i][0, 0] = Math.Cos(this.DHtheta[i] + q[i]);
                this.DH[i][0, 1] = -Math.Sin(this.DHtheta[i] + q[i]) * Math.Cos(this.DHalpha[i]);
                this.DH[i][0, 2] = Math.Sin(this.DHtheta[i] + q[i]) * Math.Sin(this.DHalpha[i]);
                this.DH[i][0, 3] = this.DHa[i] * Math.Cos(this.DHtheta[i] + q[i]);

                this.DH[i][1, 0] = Math.Sin(this.DHtheta[i] + q[i]);
                this.DH[i][1, 1] = Math.Cos(this.DHtheta[i] + q[i]) * Math.Cos(this.DHalpha[i]);
                this.DH[i][1, 2] = -Math.Cos(this.DHtheta[i] + q[i]) * Math.Sin(this.DHalpha[i]);
                this.DH[i][1, 3] = this.DHa[i] * Math.Sin(this.DHtheta[i] + q[i]);

                this.DH[i][2, 1] = Math.Sin(this.DHalpha[i]);
                this.DH[i][2, 2] = Math.Cos(this.DHalpha[i]);
                this.DH[i][2, 3] = DHd[i];

                this.DH[i][3, 3] = 1;
            }
        }

        private void CalculateDH(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
        {
            Vector q = new Vector(7);
            q[0] = q1;
            q[1] = q2;
            q[2] = q3;
            q[3] = q4;
            q[4] = q5;
            q[5] = q6;
            q[6] = q7;

            this.CalculateDH(q);
        }

        private Matrix4 CalculateHomogenMatrix(int a, int b)
        {
            Matrix4 result = Matrix4.Identity;// ('I'); //La I es para que empiece como matriz identidad
            Matrix R = new Matrix(3, 3);
            Matrix d = new Matrix(3, 1);
            for (int i = 0; i < 4; i++)
            {
                result[i, i] = 1;
            }


            if (a < b)
            {
                if (a < 0) a = 0;
                if (b > 7) b = 7;

                for (int i = a; i < b; i++)
                {
                    result *= DH[i];
                }
            }
            else
            {
                if (a > 7) a = 7;
                if (b < 0) b = 0;

                for (int i = b; i < a; i++)
                {
                    result *= DH[i];
                }

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        R[i, j] = result[i, j];
                    }
                    d[i, 0] = result[i, 3];
                }
                d = (-1 * R.Transpose) * d;

                result = result.Transpose;
                
				for (int i = 0; i < 3; i++)
                {
                    result[3, i] = 0;
                    result[i, 3] = d[i, 0];
                }
            }

            return result;
        }

        private Matrix CalculateHomogenMatrix(int a, int b, bool matrixGen)
        {
            Matrix4 temp = this.CalculateHomogenMatrix(a, b);
            Matrix result = new Matrix(4, 4);

            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    result[i, j] = temp[i, j];

            return result;
        }

        private bool InverseKinematics(double x, double y, double z, Matrix desiredR07, double elbowAngle, out Vector result)
        {
            result = new Vector(7);

            double r, alpha, beta, gamma; //Variables auxiliares para la cinemática inversa
            double D1 = DHd[0]; //Altura del piso al hombro en m
            double D2 = DHd[2]; //Distancia del hombro al codo en m
            double D3 = DHd[4]; //Distancia del codo a la muñeca en m
            double D4 = DHd[6]; //Distancia de la muñeca  al efector en m
            bool outSpaceWork = false;

            Matrix R07 = new Matrix(4, 4); // Matriz 0R7 de orientación deseada Roll Pitch Yaw
            Matrix R47 = new Matrix(4, 4); // Matriz 4R7 de orientación de la muñeca deseada implicita
            Matrix R40 = new Matrix(4, 4);
            Matrix WristPosition = new Matrix(4, 1);

            double tunningRadiusElbow;	//Radio de giro del self-motion del codo
            Matrix oRe = new Matrix(4, 4);	//Homogénea del sistema Oelbow al sistema base
            Matrix Pelbow = new Matrix(4, 1);	//Posición del codo

            if (elbowAngle > Math.PI) elbowAngle -= 2 * Math.PI;
            if (elbowAngle < -Math.PI) elbowAngle += 2 * Math.PI;
            //Recordar que para este programa, el ángulo de codo 0° corresponde a la posición de home de este programa que
            //difiere 90° del 0° que manejan la mayoría de los papers

            // Calculo Matriz Roll Pitch Yaw (matriz de orientación deseada
            R07 = desiredR07;

            //Primero wrist position se usará para calcular lo que hay que restar a la pose deseada para obtener 
            //la pose deseada del centro de la muñeca
            WristPosition[0, 0] = D4;
            WristPosition[3, 0] = 1;

            WristPosition = R07 * WristPosition;

            x = x - WristPosition[0, 0];
            y = y - WristPosition[1, 0];
            z = z - WristPosition[2, 0];
            //ahora WristPosition guarda la posición deseada del centro de la muñeca
            WristPosition[0, 0] = x;
            WristPosition[1, 0] = y;
            WristPosition[2, 0] = z;
            WristPosition[3, 0] = 1;

            //Se resta a la posición de la muñeca el defase provocado por el parámetro DHa[0]
            result[0] = Math.Atan2(y, x); //Ángulo de la cintura
            x = x - DHa[0] * Math.Cos(result[0]);
            y = y - DHa[0] * Math.Sin(result[0]);

            r = Math.Sqrt(x * x + y * y + (z - D1) * (z - D1));
            if (r <= (D2 + D3))
            {
                outSpaceWork = false;
                alpha = Math.Atan2((z - D1), Math.Sqrt(x * x + y * y));
                gamma = Math.Acos((-D2 * D2 - D3 * D3 + r * r) / (-2 * D2 * D3));
                beta = Math.Asin(D3 * Math.Sin(gamma) / r);

                //Esto siempre es considerando la solución de codo arriba, hay que checar que onda con la 
                //otra solución
                tunningRadiusElbow = D2 * Math.Sin(beta);
                //Posición del codo con respecto al sistema Oelbow
                Pelbow[0, 0] = 0;
                Pelbow[1, 0] = -tunningRadiusElbow * Math.Cos(elbowAngle);
                Pelbow[2, 0] = -tunningRadiusElbow * Math.Sin(elbowAngle);
                Pelbow[3, 0] = 1;
                //Transformación del sistema sobre el que gira el codo al sistema base
                oRe[0, 0] = Math.Cos(result[0]) * Math.Cos(-alpha);
                oRe[1, 0] = Math.Sin(result[0]) * Math.Cos(-alpha);
                oRe[2, 0] = -Math.Sin(-alpha);

                oRe[0, 1] = -Math.Sin(result[0]);
                oRe[1, 1] = Math.Cos(result[0]);
                oRe[2, 1] = 0;

                oRe[0, 2] = Math.Cos(result[0]) * Math.Sin(-alpha);
                oRe[1, 2] = Math.Sin(result[0]) * Math.Sin(-alpha);
                oRe[2, 2] = Math.Cos(-alpha);

                oRe[0, 3] = D2 * Math.Cos(beta) * Math.Cos(alpha) * Math.Cos(result[0]);
                oRe[1, 3] = D2 * Math.Cos(beta) * Math.Cos(alpha) * Math.Sin(result[0]);
                oRe[2, 3] = D2 * Math.Cos(beta) * Math.Sin(alpha) + D1;
                oRe[3, 3] = 1;

                Pelbow = oRe * Pelbow; //Transformo coordenadas de posición del codo con respecto al sistema base

                result[0] = Math.Atan2(Pelbow[1, 0] + DHa[0] * Math.Sin(result[0]), Pelbow[0, 0] + DHa[0] * Math.Cos(result[0]));
                result[1] = Math.Atan2(Pelbow[2, 0] - D1, Math.Sqrt(Pelbow[0, 0] * Pelbow[0, 0] + Pelbow[1, 0] * Pelbow[1, 0]));
                result[2] = 0;
                result[3] = 0;

                this.CalculateDH(result);

                R40 = CalculateHomogenMatrix(4, 0, true);
                //Esta matriz también se utiliza para obtener la posición del centro de la muñeca con respecto al codo
                WristPosition = R40 * WristPosition;

                result[2] = Math.Atan2(WristPosition[1, 0], WristPosition[0, 0]);
                result[3] = Math.PI / 2 - Math.Atan2(WristPosition[2, 0],
                    Math.Sqrt(WristPosition[0, 0] * WristPosition[0, 0] + WristPosition[1, 0] * WristPosition[1, 0]));

                this.CalculateDH(result);

                //A partir de aquí se calculan los ángulos de orientación
                R40 = this.CalculateHomogenMatrix(4, 0, true);
                R47 = R40 * R07;

                if (Math.Round(R47[0, 0], 3) == 0)
                {
                    result[4] = 0;
                    result[5] = 0;
                    result[6] = Math.Atan2(R47[1, 1], R47[1, 2]);
                }
                else
                {
                    result[5] = Math.Atan2(Math.Sqrt(1 - Math.Pow(R47[2, 0], 2)), (R47[2, 0]));
                    result[4] = Math.Atan2(R47[1, 0], R47[0, 0]);
                    result[6] = Math.Atan2(R47[2, 2], -R47[2, 1]);
                }
                /*for (int i = 0; i < 7; i++) {
                    q[i] = Math.Round(q[i], 8);

                    if (q[i] > (Math.PI))
                    {
                        q[i] = q[i] - (2*Math.PI);
                    }
                    if (q[i] <= (-Math.PI)) {
                        q[i] = (2 * Math.PI) + q[i];
                    }

                }*/
                if (result[4] > 2.4)
                {
                    result[4] -= Math.PI;
                    result[5] *= -1;
                    if (result[6] > 0) result[6] -= Math.PI;
                    else result[6] += Math.PI;
                }

                if (result[4] < -2.4)
                {
                    result[4] += Math.PI;
                    result[5] *= -1;
                    if (result[6] > 0) result[6] -= Math.PI;
                    else result[6] += Math.PI;
                }

                this.CalculateDH(result);

            }
            else
            {
                outSpaceWork = true;
            }

			if (result[0] < this.lowerServoLimits[0] || result[0] > this.upperServoLimits[0] ||
				result[1] < this.lowerServoLimits[1] || result[1] > this.upperServoLimits[1] ||
				result[2] < this.lowerServoLimits[2] || result[2] > this.upperServoLimits[2] ||
				result[3] < this.lowerServoLimits[3] || result[3] > this.upperServoLimits[3] ||
				result[4] < this.lowerServoLimits[4] || result[4] > this.upperServoLimits[4] ||
				result[5] < this.lowerServoLimits[5] || result[5] > this.upperServoLimits[5] ||
				result[6] < this.lowerServoLimits[6] || result[6] > this.upperServoLimits[6])
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Invalid servo angles: Out of servo range");
				return false;
			}

            if (outSpaceWork)
                result = new Vector(7);

            return !outSpaceWork;
        }

        private Vector InverseKinematics(double x, double y, double z, Matrix desiredR07, double elbowAngle)
        {
            Vector result;
            this.InverseKinematics(x, y, z, desiredR07, elbowAngle, out result);
            return result;
        }

        private Vector InverseKinematics(Vector desPos7)
        {
            if (desPos7.Length != 7) return null;

            double Roll = desPos7[3];
            double Pitch = desPos7[4];
            double Yaw = desPos7[5];

            Matrix R07 = new Matrix(4, 4);

            R07[0, 0] = Math.Cos(Roll) * Math.Cos(Pitch);
            R07[0, 1] = -Math.Sin(Roll) * Math.Cos(Yaw) + Math.Cos(Roll) * Math.Sin(Pitch) * Math.Sin(Yaw);
            R07[0, 2] = Math.Sin(Roll) * Math.Sin(Yaw) + Math.Cos(Roll) * Math.Sin(Pitch) * Math.Cos(Yaw);

            R07[1, 0] = Math.Sin(Roll) * Math.Cos(Pitch);
            R07[1, 1] = Math.Cos(Roll) * Math.Cos(Yaw) + Math.Sin(Roll) * Math.Sin(Pitch) * Math.Sin(Yaw);
            R07[1, 2] = -Math.Cos(Roll) * Math.Sin(Yaw) + Math.Sin(Roll) * Math.Sin(Pitch) * Math.Cos(Yaw);


            R07[2, 0] = -Math.Sin(Pitch);
            R07[2, 1] = Math.Cos(Pitch) * Math.Sin(Yaw);
            R07[2, 2] = Math.Cos(Pitch) * Math.Cos(Yaw);

            R07[3, 3] = 1;

            return this.InverseKinematics(desPos7[0], desPos7[1], desPos7[2], R07, desPos7[6]);
        }

        public bool InverseKinematics(Vector desPos7, out Vector result)
        {
            result = new Vector(7);
            if (desPos7 == null || desPos7.Length != 7) return false;

            double Roll = desPos7[3];
            double Pitch = desPos7[4];
            double Yaw = desPos7[5];

            Matrix R07 = new Matrix(4, 4);

            R07[0, 0] = Math.Cos(Roll) * Math.Cos(Pitch);
            R07[0, 1] = -Math.Sin(Roll) * Math.Cos(Yaw) + Math.Cos(Roll) * Math.Sin(Pitch) * Math.Sin(Yaw);
            R07[0, 2] = Math.Sin(Roll) * Math.Sin(Yaw) + Math.Cos(Roll) * Math.Sin(Pitch) * Math.Cos(Yaw);

            R07[1, 0] = Math.Sin(Roll) * Math.Cos(Pitch);
            R07[1, 1] = Math.Cos(Roll) * Math.Cos(Yaw) + Math.Sin(Roll) * Math.Sin(Pitch) * Math.Sin(Yaw);
            R07[1, 2] = -Math.Cos(Roll) * Math.Sin(Yaw) + Math.Sin(Roll) * Math.Sin(Pitch) * Math.Cos(Yaw);


            R07[2, 0] = -Math.Sin(Pitch);
            R07[2, 1] = Math.Cos(Pitch) * Math.Sin(Yaw);
            R07[2, 2] = Math.Cos(Pitch) * Math.Cos(Yaw);

            R07[3, 3] = 1;

            return this.InverseKinematics(desPos7[0], desPos7[1], desPos7[2], R07, desPos7[6], out result);
        }

        public Vector ForwardKinematics(Vector q)
        {
            Vector result = new Vector(7);

            //Matrix XYZ = new Matrix(4, 1);
            Matrix4 R70cte = Matrix4.Zero;

            R70cte[2, 0] = 1; //Esta matriz es el giro que falta para colocar el sistema del efector final
            R70cte[0, 1] = 1; //paralelo al sistema base. Es la misma matriz que se le aplica al formulazo de los
            R70cte[1, 2] = 1; //ángulos de Euler para sacar la cinemática inversa
            R70cte[3, 3] = 1;

            this.CalculateDH(q);

            Matrix4 xPosTemp = this.CalculateHomogenMatrix(0, 7);
            Matrix4 xPos = xPosTemp * R70cte;

            result[0] = xPos[0, 3];
            result[1] = xPos[1, 3];
            result[2] = xPos[2, 3];

            result[3] = Math.Atan2(xPos[1, 0], xPos[0, 0]);
            result[4] = Math.Atan2(-xPos[2, 0], Math.Sqrt(1 - xPos[2, 0] * xPos[2, 0]));
            result[5] = Math.Atan2(xPos[2, 1], xPos[2, 2]);
            result[6] = 0;

            return result;
        }

        /*public bool GoToArticularPosition(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
        {
			bool setSuccess = true;
            bool[] success = new bool[7];
            bool globalSuccess;
            double error = 3 * Math.PI / 180;
            Stopwatch clock = new Stopwatch();
            Stopwatch timeout = new Stopwatch();
            int timeoutTime = 10000;

            TextBoxStreamWriter.DefaultLog.WriteLine("RightArm: Trying to reach " + q1.ToString("0.0000") + " " +
                q2.ToString("0.0000") + " " + q3.ToString("0.0000") + " " + q4.ToString("0.0000") + " " +
                q5.ToString("0.0000") + " " + q6.ToString("0.0000") + " " + q7.ToString("0.0000"));

            this.desPosArticular[0] = q1;
            this.desPosArticular[1] = q2;
            this.desPosArticular[2] = q3;
            this.desPosArticular[3] = q4;
            this.desPosArticular[4] = q5;
            this.desPosArticular[5] = q6;
            this.desPosArticular[6] = q7;

            this.desPosCartesian = this.ForwardKinematics(this.desPosArticular);

			for (int i = 0; i < dof; i++)
			{
				setSuccess &= SetServoPosition(i, desPosArticular[i], 0);
				if (!setSuccess)
				{
					TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + servos[i] + " set failure");
					return false;
				}
			}

			//for (int i = 0; i < dof; i++)
			//{
			//    this.servos[i].SetSpeedPerc(5);
			//    this.servos[i].SetPosition(this.desPosArticular[i]);
			//    Thread.Sleep(5);
			//}

			//for (int i = 0; i < dof; i++)
			//{
			//    this.servos[i].SetSpeedPerc(5);
			//    this.servos[i].SetPosition(this.desPosArticular[i]);
			//    Thread.Sleep(5);
			//}

            timeout.Reset();
            timeout.Start();
            do
            {
                globalSuccess = true;
                this.GetPositionArticular();
                for (int i = 0; i < dof; i++)
                {
                    success[i] &= (this.currentPosArticular[i] <= this.desPosArticular[i] + error && this.currentPosArticular[i] >= this.desPosArticular[i] - error);
                    if (!success[i])
                    {
                        globalSuccess = false;
                        success[i] = true;
                    }
                }
                clock.Reset();
                clock.Start();
                do
                {
                    Thread.Sleep(20);
                } while (clock.ElapsedMilliseconds <= 100);
                clock.Stop();
            } while (!globalSuccess && timeout.ElapsedMilliseconds <= timeoutTime);
            timeout.Stop();

            //ESTO ES TEMPORAL, DEBERÍA CHECAR LA POSICIÓN REAL DEL ROBOT
			//this.currentPosCartesian = this.ForwardKinematics(this.desPosArticular);
            this.CheckPositionChangedEvent();

            //return globalSuccess;
            return globalSuccess;
        }*/

        public bool GetVoltage(out double volt)
        {
            string cmd = "v\r";
            int resp=0;
            string[] respParts;
            volt = 0;
            
            if (!SendReceiveCommandCM700(cmd, out resp, out respParts)) return false;

            if (resp == 0)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() + ": CM700 couldn't retrieve voltage from servo ID 0");
                return false;
            }

            if (!double.TryParse(respParts[2], out volt))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() + ": Can't parse voltage answer from CM700");
                return false;
            }

            return true;

        }


		public void GoToArticularPositionNoAnswer(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
		{
			if (q1 < this.lowerServoLimits[0] || q1 > this.upperServoLimits[0] ||
				q2 < this.lowerServoLimits[1] || q2 > this.upperServoLimits[1] ||
				q3 < this.lowerServoLimits[2] || q3 > this.upperServoLimits[2] ||
				q4 < this.lowerServoLimits[3] || q4 > this.upperServoLimits[3] ||
				q5 < this.lowerServoLimits[4] || q5 > this.upperServoLimits[4] ||
				q6 < this.lowerServoLimits[5] || q6 > this.upperServoLimits[5] ||
				q7 < this.lowerServoLimits[6] || q7 > this.upperServoLimits[6])
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Invalid servo angles: Out of servo range");
				return;
			}

			//Updating current position
			//UpdateArtPos();

			this.desPosArticular[0] = q1;
			this.desPosArticular[1] = q2;
			this.desPosArticular[2] = q3;
			this.desPosArticular[3] = q4;
			this.desPosArticular[4] = q5;
			this.desPosArticular[5] = q6;
			this.desPosArticular[6] = q7;

			this.desPosCartesian = this.ForwardKinematics(this.desPosArticular);

			TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() + ": Trying to reach articular position: " +
				this.desPosArticular.ToString());

			double deltaMax = 0;
			double[] deltas = new double[7];
			deltas[0] = Math.Abs(this.currentPosArticular[0] - q1);
			deltas[1] = Math.Abs(this.currentPosArticular[1] - q2);
			deltas[2] = Math.Abs(this.currentPosArticular[2] - q3);
			deltas[3] = Math.Abs(this.currentPosArticular[3] - q4);
			deltas[4] = Math.Abs(this.currentPosArticular[4] - q5);
			deltas[5] = Math.Abs(this.currentPosArticular[5] - q6);
			deltas[6] = Math.Abs(this.currentPosArticular[6] - q7);
			for (int i = 0; i < this.dof; i++) if (deltas[i] > deltaMax) deltaMax = deltas[i];

			double desTime = 1.0;

			if (desTime < 0.5) desTime = 0.5;
			if (desTime > 5) desTime = 5;

			string cmdToSend;

			cmdToSend = "q " + q1.ToString("0.000") + " " + q2.ToString("0.000") + " " + q3.ToString("0.000") + " "
				 + q4.ToString("0.000") + " " + q5.ToString("0.000") + " " + q6.ToString("0.000") + " "
				  + q7.ToString("0.000") + " " + desTime.ToString("0.00") + "\r";


			if (!this.serialPort.IsOpen)
			{
				try
				{
					this.serialPort.Open();
				}
				catch
				{
					TextBoxStreamWriter.DefaultLog.WriteLine("Can't open serial port" + this.serialPort.PortName);
					return;
				}
			}

			try
			{
				this.serialPort.DiscardInBuffer();
				this.serialPort.DiscardOutBuffer();
				this.serialPort.Write(cmdToSend);
			}
			catch (Exception e)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Can't write to serial port " + this.serialPort.PortName + " " +
					e.Message);
				return;
			}

			for (int i = 0; i < 7; i++) this.currentPosArticular[i] = this.desPosArticular[i];
			//UpdateArtPos();

		}
		public void GoToArticularPositionNoAnswer(Vector q)
		{
			if (q == null || q.Length != 7)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Invalid articular positions");
				return;
			}

			this.GoToArticularPositionNoAnswer(q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
		}

        public bool GoToArticularPosition(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
        {
            if (q1 < this.lowerServoLimits[0] || q1 > this.upperServoLimits[0] ||
                q2 < this.lowerServoLimits[1] || q2 > this.upperServoLimits[1] ||
                q3 < this.lowerServoLimits[2] || q3 > this.upperServoLimits[2] ||
                q4 < this.lowerServoLimits[3] || q4 > this.upperServoLimits[3] ||
                q5 < this.lowerServoLimits[4] || q5 > this.upperServoLimits[4] ||
                q6 < this.lowerServoLimits[5] || q6 > this.upperServoLimits[5] ||
                q7 < this.lowerServoLimits[6] || q7 > this.upperServoLimits[6])
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Invalid servo angles: Out of servo range");
                return false;
            }

            this.desPosArticular[0] = q1;
            this.desPosArticular[1] = q2;
            this.desPosArticular[2] = q3;
            this.desPosArticular[3] = q4;
            this.desPosArticular[4] = q5;
            this.desPosArticular[5] = q6;
            this.desPosArticular[6] = q7;

            this.desPosCartesian = this.ForwardKinematics(this.desPosArticular);

            TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() + ": Trying to reach articular position: " +
                this.desPosArticular.ToString());

            double deltaMax = 0;
            double[] deltas = new double[7];
            deltas[0] = Math.Abs(this.currentPosArticular[0] - q1);
            deltas[1] = Math.Abs(this.currentPosArticular[1] - q2);
            deltas[2] = Math.Abs(this.currentPosArticular[2] - q3);
            deltas[3] = Math.Abs(this.currentPosArticular[3] - q4);
            deltas[4] = Math.Abs(this.currentPosArticular[4] - q5);
            deltas[5] = Math.Abs(this.currentPosArticular[5] - q6);
            deltas[6] = Math.Abs(this.currentPosArticular[6] - q7);
            for (int i = 0; i < this.dof; i++) if (deltas[i] > deltaMax) deltaMax = deltas[i];
			
            double desTime = 1.0;//3 * deltaMax / (Math.PI / 2);
            
            if (desTime < 0.5) desTime = 0.5;
            if (desTime > 5) desTime = 5;

            string cmdToSend;

            //if (this.armType == ArmType.RightArm)
            //{
                cmdToSend = "q " + q1.ToString("0.000") + " " + q2.ToString("0.000") + " " + q3.ToString("0.000") + " "
                     + q4.ToString("0.000") + " " + q5.ToString("0.000") + " " + q6.ToString("0.000") + " "
                      + q7.ToString("0.000") + " " + desTime.ToString("0.00") + "\r";
            //}
            /*else
            {
                int[] tq = new int[7];
                for (int i = 0; i < 7; i++)
                {
                    tq[i] = (int)(this.desPosArticular[i] * this.constants[i] * this.senses[i] + this.centers[i]);
                    if (tq[i] < 0) tq[i] = 0;
                }
                cmdToSend = "q " + tq[0].ToString() + " " + tq[1].ToString() + " " + tq[2].ToString() + " " +
                    tq[3].ToString() + " " + tq[4].ToString() + " " + tq[5].ToString() + " " + tq[6].ToString() + "\r";
            }*/

            this.serialPort.ReadTimeout = 7000;
            this.serialPort.NewLine = "\r";

            if (!this.serialPort.IsOpen)
            {
                try
                {
                    this.serialPort.Open();
                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't open serial port" + this.serialPort.PortName);
                    return false;
                }
            }

            try
            {
                this.serialPort.DiscardInBuffer();
                this.serialPort.DiscardOutBuffer();
                this.serialPort.Write(cmdToSend);
            }
            catch(Exception e)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Can't write to serial port " + this.serialPort.PortName + " " + 
					e.Message);
                return false;
            }

            //if (this.armType == ArmType.RightArm)
            //{
                string response = "";

                try
                {
                    this.serialPort.ReadLine();
                    this.serialPort.ReadLine();
                    //this.serialPort.ReadLine();
                    //this.serialPort.ReadLine();
                    response += this.serialPort.ReadLine();
                }
                catch(Exception e)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't read from serial port " + this.serialPort.PortName + " " 
						+ e.Message);
                    return false;
                }

                bool succes = true;
                char[] delimiters = { ' ', '\r', ',', '\n' };
                string[] parts = response.Split(delimiters, StringSplitOptions.RemoveEmptyEntries);
                
                

                Vector tempCurrentPos = new Vector(7);

                try
                {
                    succes = parts[1] == "1";
                    for (int i = 0; i < 7; i++) tempCurrentPos[i] = double.Parse(parts[i + 2]);
                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't parse response from microcontroller");
                    return false;
                }
                //for (int i = 0; i < 7; i++) this.currentPosArticular[i] = tempCurrentPos[i];
				for (int i = 0; i < 7; i++) this.currentPosArticular[i] = this.desPosArticular[i];

                if (succes)
                    TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() + ": Position reached");
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() + ": Can't reach position");

                return succes;
            //}
            /*else
			 
            {
                string[] responses = new string[8];
                double[] tempCurrentAngles = new double[7];
                try
                {
                    for (int i = 0; i < 8; i++) responses[i] = this.serialPort.ReadLine();
                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't read from serial port " + this.serialPort.PortName);
                    return false;
                }
                try
                {
                    for (int i = 0; i < 7; i++)
                    {
                        tempCurrentAngles[i] = double.Parse(responses[i]);
                        tempCurrentAngles[i] = (tempCurrentAngles[i] - this.centers[i]) / this.constants[i] / this.senses[i];
                    }

                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't parse response from microcontroller");
                    return false;
                }
                //Esto que se ve medio truculento es porque el brazo izquierdo no lee bien las posiciones de los 
                //servos
                for (int i = 0; i < 7; i++)
                    if (Math.Abs(this.desPosArticular[i] - tempCurrentAngles[i]) < 0.2)
                        this.currentPosArticular[i] = tempCurrentAngles[i];
                    else this.currentPosArticular[i] = this.desPosArticular[i];

                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() + ": Position reached");

                return true;
            }*/
        }

        public bool GoToArticularPosition(Vector q)
        {
            if (q == null || q.Length != 7)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Invalid articular positions");
                return false;
            }
            return this.GoToArticularPosition(q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
        }

        public bool GoToCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            Vector tempArt;
            Vector tempPos = new Vector(x, y, z, roll, pitch, yaw, elbow);
            if (!this.InverseKinematics(tempPos, out tempArt))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Can't calculate inverse kinematics: Out of work space");
                return false;
            }

            if (tempArt[0] < this.lowerServoLimits[0] || tempArt[0] > this.upperServoLimits[0] ||
                tempArt[1] < this.lowerServoLimits[1] || tempArt[1] > this.upperServoLimits[1] ||
                tempArt[2] < this.lowerServoLimits[2] || tempArt[2] > this.upperServoLimits[2] ||
                tempArt[3] < this.lowerServoLimits[3] || tempArt[3] > this.upperServoLimits[3] ||
                tempArt[4] < this.lowerServoLimits[4] || tempArt[4] > this.upperServoLimits[4] ||
                tempArt[5] < this.lowerServoLimits[5] || tempArt[5] > this.upperServoLimits[5] ||
                tempArt[6] < this.lowerServoLimits[6] || tempArt[6] > this.upperServoLimits[6])
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Invalid servo angles: Out of servo range");
                return false;
            }

            for (int i = 0; i < tempArt.Length; i++) this.desPosArticular[i] = tempArt[i];
            for (int i = 0; i < tempPos.Length; i++) this.desPosCartesian[i] = tempPos[i];

            return this.GoToArticularPosition(this.desPosArticular);
        }

        public bool GoToCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw)
        {
            return this.GoToCartesianPosition(x, y, z, roll, pitch, yaw, this.desPosCartesian[6]);
        }

        public bool GoToCartesianPosition(double x, double y, double z)
        {
            return this.GoToCartesianPosition(x, y, z, this.desPosCartesian[3], this.desPosCartesian[4], this.desPosCartesian[5],
                this.desPosCartesian[6]);
        }

        public bool GoToRelativeCartesianPos(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            Vector temp = new Vector(7);
            Vector res;
            temp[0] = this.desPosCartesian[0] + x;
            temp[1] = this.desPosCartesian[1] + y;
            temp[2] = this.desPosCartesian[2] + z;
            temp[3] = this.desPosCartesian[3] + roll;
            temp[4] = this.desPosCartesian[4] + pitch;
            temp[5] = this.desPosCartesian[5] + yaw;
            temp[6] = this.desPosCartesian[6] + elbow;

            if (this.InverseKinematics(temp, out res))
            {

                this.desPosCartesian[0] += x;
                this.desPosCartesian[1] += y;
                this.desPosCartesian[2] += z;
                this.desPosCartesian[3] += roll;
                this.desPosCartesian[4] += pitch;
                this.desPosCartesian[5] += yaw;
                this.desPosCartesian[6] += elbow;
                return this.GoToCartesianPosition(this.desPosCartesian[0], this.desPosCartesian[1], this.desPosCartesian[2],
                    this.desPosCartesian[3], this.desPosCartesian[4], this.desPosCartesian[5], this.desPosCartesian[6]);
            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType + ": Cannot calculate inverse kinematics to reach relative position");
                return false;
            }
        }

        public Vector GetPositionCartesian()
        {
			this.GetPositionArticular();
            this.currentPosCartesian = this.ForwardKinematics(this.currentPosArticular);
			//TextBoxStreamWriter.DefaultLog.WriteLine("FK: " + this.currentPosCartesian[0].ToString("0.0000") + " " +
			//    this.currentPosCartesian[1].ToString("0.0000") + " " + this.currentPosCartesian[2].ToString("0.0000") + " " +
			//    this.currentPosCartesian[3].ToString("0.0000") + " " + this.currentPosCartesian[4].ToString("0.0000") + " " +
			//        this.currentPosCartesian[5].ToString("0.0000") + " " + this.currentPosCartesian[6].ToString("0.0000"));
            return this.currentPosCartesian;
        }

        

		public Vector GetPositionArticular()
		{
			Vector temp;
			double[] theta = new double[7];
            
            int resp;
            string[] respParts;

            //Instrucción para pedir las posiciones articulares de la tarjeta
            string cmd = "g\r";
            
            if (!SendReceiveCommandCM700(cmd, out resp, out respParts)) return null;

            if (resp == 0)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                    ": Error receiving articular position");
                return null;
            }

            try
            {
                theta[0] = double.Parse(respParts[2]);
                theta[1] = double.Parse(respParts[3]);
                theta[2] = double.Parse(respParts[4]);
                theta[3] = double.Parse(respParts[5]);
                theta[4] = double.Parse(respParts[6]);
                theta[5] = double.Parse(respParts[7]);
                theta[6] = double.Parse(respParts[8]);
                temp = new Vector(theta);
            }
            catch
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                    ": Error parsing articular positions");
                return null;
            }

            return temp;

            
		}

        public bool GetErrorState( out string errors)
        {
            string cmd = "S\r"; //Comando para recibir el estado de los errores
            int resp;
            string[] respParts;

            errors = "ErrorState ";
            
            if (!SendReceiveCommandCM700(cmd, out resp, out respParts)) return false;
            

            if (resp == 0 || respParts.Count() != 11)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                    ": Error receiving error state. Parts Received: (" + respParts.Count() + "/11)");
                return false;
            }

            
            for (int i = 2; i < 11; i++)
            {
                int error = 0;
                
                if (i != 10)
                    errors += (i - 2) + ": ";
                else
                    errors += i + ": ";

                if (!int.TryParse(respParts[i], out error))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() + ": Can't parse response " + respParts[i]);
                    errors += "NoParse ";
                }
                else
                {
                    if (error == 1) errors += "OK ";
                    else
                    {
                        if (error % 2 == 0) errors += "InVoltaje ";
                        if (error % 3 == 0) errors += "OverHeat ";
                        if (error % 5 == 0) errors += "OverLoad ";
                    }
                }

            }
            return true;


        }

        public bool OpenGripper(double percentage)
        {
            
            int percInt = (int)(percentage);
            string cmd = "o " + percInt.ToString() + "\r";
            
            string[] respParts;
            int resp = 0;

            if (!SendReceiveCommandCM700(cmd, out resp, out respParts)) return false;

            if (resp == 0) return false;
            else return true;
        }

        public bool CloseGripper(double force)
        {
            string cmd = "c 45\r"; //antes 25

            string[] respParts;
            int resp = 0;
            
            if (!SendReceiveCommandCM700(cmd, out resp, out respParts)) return false;

            if (resp == 0)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                ": Error closing gripper");
                return false;
            }
            else return true;

        }

        public bool EnableTorque(bool enable)
        {
            string response = "";
            string cmd = "t " + (enable ? "1" : "0") + "\r";
            char[] delimiters = { ' ' };
            string[] parts;
            int resp = 0;

            if (!this.serialPort.IsOpen)
            {
                try
                {
                    this.serialPort.Open();
                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                        ": Can't open serial port" + this.serialPort.PortName);
                    return false;
                }
            }

            try
            {
                this.serialPort.DiscardInBuffer();
                this.serialPort.DiscardOutBuffer();
                this.serialPort.Write(cmd);
            }
            catch
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                    ": Can't write to serial port " + this.serialPort.PortName);
                return false;
            }

            this.serialPort.ReadTimeout = 2500;

            try
            {
                response = this.serialPort.ReadLine();
            }
            catch (Exception e)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Can't read from serial port " + this.serialPort.PortName + " " +
                    e.Message);
                return false;
            }

            parts = response.Split(delimiters, StringSplitOptions.RemoveEmptyEntries);

            try
            {
                resp = int.Parse(parts[1]);
            }
            catch
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                    ": Can't parse response from serial port " + this.serialPort.PortName);
                return false;
            }

            if (resp == 0) return false;
            else return true;
        }

        public bool IsManipulatorReady()
        {
            //return true;
            return this.GoToArticularPosition(0, 0, 0, 0, 0, 0, 0);
        }

        public event ArmPositionChangedEH ArmPositionChanged;

        private void OnArmPositionChnaged(Manipulator senderArm)
        {
            if (this.ArmPositionChanged != null) this.ArmPositionChanged(senderArm);
        }

        public Vector CartesianPosition { get { return this.currentPosCartesian; } }

		public bool SetServoPosition(int id, double desPosArticular, int k)
		{
			//double goalPosition;
			int desBits;
			int realBits;

			if (k > 4)
				return false;

			this.servos[id].SetSpeedPerc(10);
			Thread.Sleep(1);
			//this.servos[id].SetPosition(desPosArticular, out desBits);
			this.servos[id].SetPosition(desPosArticular, out desBits);
			this.servos[id].SetPositionBits(desBits);
			//this.servos[id].SetPosition(desPosArticular);
			Thread.Sleep(2);
			//this.spm.WaitForData(5);
			this.servos[id].GetGoalPositionBits(out realBits);
			//this.servos[id].GetGoalPosition(out goalPosition);
			
			if (desBits != realBits)
			{
				SetServoPosition(id, desPosArticular, k + 1);
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id.ToString() + " " + desBits + " " + realBits);
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id.ToString() + " reached " + desBits + " " + realBits);
				return true;
			}

			return true;
		}

		public bool TorqueOnOff(bool OnOff)
		{
            
			int resp;
            string[] respParts;

			//Instrucción para encender o apagar el torque
            string cmd = OnOff ? "t 1\r" : "t 0\r";

            if (!SendReceiveCommandCM700(cmd, out resp, out respParts)) return false;

            if (resp == 0)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                    ": Error switching torque On/Off");
                return false;
            }
            else return true;

		}
        private bool SendReceiveCommandCM700(string cmd, out int resp, out string[] respParts)
        {
            string response = "";
            char[] delimiters = { ' ' };
            resp = 0;
            respParts = new string[1];

            if (!this.serialPort.IsOpen)
            {
                try
                {
                    this.serialPort.Open();
                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                        ": Can't open serial port" + this.serialPort.PortName);
                    return false;
                }
            }

            try
            {
				this.serialPort.DiscardInBuffer();
                this.serialPort.DiscardOutBuffer();
               
                this.serialPort.Write(cmd);
				Thread.Sleep(20);
            }
            catch
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                    ": Can't write to serial port " + this.serialPort.PortName);
                return false;
            }

            this.serialPort.ReadTimeout = 7000;

            try
            {
                response = this.serialPort.ReadLine();
            }
            catch (Exception e)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Can't read from serial port " + this.serialPort.PortName + " " +
                e.Message);
                return false;
            }
            
            respParts = response.Split(delimiters, StringSplitOptions.RemoveEmptyEntries);

            try
            {
                resp = int.Parse(respParts[1]);
            }
            catch
            {
                TextBoxStreamWriter.DefaultLog.WriteLine(this.armType.ToString() +
                    ": Can't parse response from serial port " + this.serialPort.PortName);
                return false;
            }
            return true;
        }
    }
}
