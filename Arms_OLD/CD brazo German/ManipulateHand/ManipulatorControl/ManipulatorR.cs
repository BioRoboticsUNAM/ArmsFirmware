using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class ManipulatorR
    {
        private ServoControl[] servos;
        private double[] DHd;
        private double[] DHa;
        private double[] DHalpha;
        private double[] DHtheta;
        private Matrix4[] DH;

        SerialPortManager spm;

        private Vector currentPosCartesian;
        private Vector currentPosArticular;
        private Vector desPosCartesian;
        private Vector desPosArticular;

        public ManipulatorR()
        {
            this.InitializeDH();
            this.currentPosCartesian = new Vector(7);
            this.currentPosArticular = new Vector(7);
            this.desPosCartesian = new Vector(7);
            this.desPosArticular = new Vector(7);
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

            this.DHd[2] = 0.3084;
            this.DHd[4] = 0.2126;
            this.DHd[6] = 0.16;

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

        private Vector ForwardKinematics(Vector q)
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

        private Vector InverseKinematics(double x, double y, double z, Matrix desiredR07, double elbowAngle)
        {
            Vector result = new Vector(7);

            double r, alpha, beta, gamma; //Variables auxiliares para la cinemática inversa
            double D1 = DHd[0]; //Altura del piso al hombro en cm
            double D2 = DHd[2]; //Distancia del hombro al codo en cm
            double D3 = DHd[4]; //Distancia del codo a la muñeca en cm
            double D4 = DHd[6]; //Distancia de la muñeca  al efector en cm
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
            if (r < (D2 + D3))
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

            if (outSpaceWork)
                result = new Vector(7);

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

        public bool GoToPosition(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("RightArm: Trying to reach " + x.ToString("0.00") + " " +
                y.ToString("0.00") + " " + z.ToString("0.00") + " " + roll.ToString("0.0000") + " " +
                pitch.ToString("0.0000") + " " + yaw.ToString("0.0000") + " " + elbow.ToString("0.0000"));

            this.desPosCartesian[0] = x;
            this.desPosCartesian[1] = y;
            this.desPosCartesian[2] = z;
            this.desPosCartesian[3] = roll;
            this.desPosCartesian[4] = pitch;
            this.desPosCartesian[5] = yaw;
            this.desPosCartesian[6] = elbow;

            this.currentPosArticular = this.InverseKinematics(this.desPosCartesian);

            this.GetPosition();
            
            return true;
        }

        public bool GoToPosition(double x, double y, double z, double roll, double pitch, double yaw)
        {
            return this.GoToPosition(x, y, z, roll, pitch, yaw, this.desPosCartesian[6]);
        }

        public bool GoToPosition(double x, double y, double z)
        {
            return this.GoToPosition(x, y, z, this.desPosCartesian[3], this.desPosCartesian[4], this.desPosCartesian[5],
                this.desPosCartesian[6]);
        }

        public Vector GetPosition()
        {
            this.currentPosCartesian = this.ForwardKinematics(this.currentPosArticular);
            TextBoxStreamWriter.DefaultLog.WriteLine("FK: " + this.currentPosCartesian[0].ToString("0.0000") + " " +
                this.currentPosCartesian[1].ToString("0.0000") + " " + this.currentPosCartesian[2].ToString("0.0000") + " " +
                this.currentPosCartesian[3].ToString("0.0000") + " " + this.currentPosCartesian[4].ToString("0.0000") + " " +
                    this.currentPosCartesian[5].ToString("0.0000") + " " + this.currentPosCartesian[6].ToString("0.0000"));
            return this.currentPosCartesian;
        }

        public bool OpenGripper(double percentage)
        {
            return true;
        }

        public bool CloseGripper(double force)
        {
            return true;
        }
    }
}
