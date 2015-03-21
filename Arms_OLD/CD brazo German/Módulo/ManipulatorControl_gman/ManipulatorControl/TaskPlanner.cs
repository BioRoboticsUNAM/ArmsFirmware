using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class TaskPlanner
    {
        private Manipulator leftArm;
        private Manipulator rightArm;
        private SortedList<string, MapObstacle> mapObstacles;
        private SortedList<string, MapNode> mapNodes;
        private SortedList<string, PredefPosition> leftPredefPos;
        private SortedList<string, PredefMovement> leftPredefMovs;
        private SortedList<string, PredefPosition> rightPredefPos;
        private SortedList<string, PredefMovement> rightPredefMovs;
        private MapOptimalPath optLeftPath;
        private MapOptimalPath optRightPath;
        private MapGoalPoint leftGoal;
        private MapGoalPoint rightGoal;
        private bool movingLeftArm;
        private bool movingRightArm;

        ManipulatorManager armMan;

        public TaskPlanner(ManipulatorManager armMan)
        {
            this.armMan = armMan;

            this.leftArm = armMan.LeftArm;
            this.rightArm = armMan.RightArm;
            this.mapObstacles = armMan.MapObstacles;
            this.mapNodes = armMan.MapNodes;
            this.leftPredefPos = armMan.LeftPredefPos;
            this.leftPredefMovs = armMan.LeftPredefMovs;
            this.rightPredefPos = armMan.RightPredefPos;
            this.rightPredefMovs = armMan.RightPredefMovs;
            this.optLeftPath = armMan.OptLeftPath;
            this.optRightPath = armMan.OptRightPath;
            this.leftGoal = armMan.LeftGoal;
            this.rightGoal = armMan.RightGoal;
		}

		#region Both arms
        public bool ArmsGetVoltage(out double LAv, out double RAv)
        {
            bool succes = true;
            int lowBat = 0;
            LAv = 0;
            RAv = 0;

            if (!this.MovingLeftArm && this.armMan.status.IsLeftArmReady)
                succes&= this.leftArm.GetVoltage(out LAv);
            else
                succes = false;
            

            if (!this.MovingRightArm && this.armMan.status.IsRightArmReady)
                succes&= this.rightArm.GetVoltage(out RAv);
            else
                succes = false;

            if (succes && (RAv < 16 || LAv < 16))
            {
				lowBat = 1;
			}

            this.armMan.UpdateLowBatSharedVar(lowBat);

            return succes;
            
                
        }

        public bool ArmsGoToPredefPos(string position)
        {
            bool succes = false;
            int n = 0;
            int maxIntentos = 10;
            double maxError = 5 * Math.PI / 180; //5°

            if (!this.leftPredefPos.ContainsKey(position))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskPlanner: Can't find predefined position for left arm : \"" +
                    position + "\"");
                return false;
            }

            if (!this.rightPredefPos.ContainsKey(position))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskPlanner: Can't find predefined position for right arm: \"" + position + "\"");
                return false;
            }

            PredefPosition rp = this.rightPredefPos[position];
            PredefPosition lp = this.leftPredefPos[position];

            Vector rv;
            Vector lv;

            movingLeftArm = true;
            movingRightArm = true;

            do
            {

                this.leftArm.GoToArticularPositionNoAnswer(lp.Q1, lp.Q2, lp.Q3, lp.Q4, lp.Q5, lp.Q6, lp.Q7);
                this.rightArm.GoToArticularPositionNoAnswer(rp.Q1, rp.Q2, rp.Q3, rp.Q4, rp.Q5, rp.Q6, rp.Q7);

                
                Thread.Sleep(3000); //Tiempo de espera para alcanzar cada posición

                rv = this.rightArm.GetPositionArticular();
                lv = this.leftArm.GetPositionArticular();

                succes = (
                    (Math.Abs(rv[0] - rp.Q1) < maxError) &&
                    (Math.Abs(rv[1] - rp.Q2) < maxError) &&
                    (Math.Abs(rv[2] - rp.Q3) < maxError) &&
                    (Math.Abs(rv[3] - rp.Q4) < maxError) &&
                    (Math.Abs(rv[4] - rp.Q5) < maxError) &&
                    (Math.Abs(rv[5] - rp.Q6) < maxError) &&
                    
                    (Math.Abs(lv[0] - lp.Q1) < maxError) &&
                    (Math.Abs(lv[1] - lp.Q2) < maxError) &&
                    (Math.Abs(lv[2] - lp.Q3) < maxError) &&
                    (Math.Abs(lv[3] - lp.Q4) < maxError) &&
                    (Math.Abs(lv[4] - lp.Q5) < maxError) &&
                    (Math.Abs(lv[5] - lp.Q6) < maxError) 
                    
                    );
				
				n++;
            } while (!succes && !(n >= maxIntentos));

            movingRightArm = false;
            movingLeftArm = false;
			
            TextBoxStreamWriter.DefaultLog.WriteLine("Both arms reached predefined potition " + succes.ToString());
            return succes;
        }
		public bool ArmsMove(string movement)
		{
			if (!this.leftPredefMovs.ContainsKey(movement))
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("TaskPlanner: Can't find predefined movement for left arm : \"" +
					movement + "\"");
				return false;
			}
			if (!this.rightPredefMovs.ContainsKey(movement))
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("TaskPlanner: Can't find predefined movement for right arm : \"" +
					movement + "\"");
				return false;
			}

			List<PredefPosition> leftPos = this.leftPredefMovs[movement].Positions;
			List<PredefPosition> rightPos = this.rightPredefMovs[movement].Positions;

			if ((leftPos.Count != rightPos.Count)|| leftPos.Count==0 || rightPos.Count==0)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("armsMove: PredefMov " + movement + " is of unequal length on left and right PredefPos lists, or cero");
				return false;
			}
			
			
			bool succes = true;

			for (int i = 0; i < leftPos.Count; i++)
			{
				succes &= ArmsGoToPredefPos(leftPos[i].Name);
			}
			TextBoxStreamWriter.DefaultLog.WriteLine("Movement performed " + succes.ToString());
			return succes;
		}
		public bool ArmsTorqueEnable(bool OnOff)
		{
			return RaTorque(OnOff) && LaTorque(OnOff);
		}
		#endregion

		#region Right arm
		public bool RaGoToCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            this.movingRightArm = true;
            // Aun no implementado
            this.optRightPath.CalculateOptimalPath();
            
            bool succes = this.rightArm.GoToCartesianPosition(x, y, z, roll, pitch, yaw, elbow);
            this.movingRightArm = false;
            return succes;
        }

        public bool RaGoToCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw)
        {
            this.movingRightArm = true;
            this.optRightPath.CalculateOptimalPath();
            bool succes = this.rightArm.GoToCartesianPosition(x, y, z, roll, pitch, yaw);
            this.movingRightArm = false;
            return succes;
        }

        public bool RaGoToCartesianPosition(double x, double y, double z)
        {
            this.movingRightArm = true;
            this.optRightPath.CalculateOptimalPath();
            bool succes = this.rightArm.GoToCartesianPosition(x, y, z);
            this.movingRightArm = false;
            return succes;
        }

        public bool RaGoToArticularPosition(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
        {
            this.movingRightArm = true;
            this.optRightPath.CalculateOptimalPath();
            bool succes = this.rightArm.GoToArticularPosition(q1, q2, q3, q4, q5, q6, q7);
            this.movingRightArm = false;
            return succes;
        }

        public bool RaGoToRelativeCartesianPos(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            this.movingRightArm = true;
            bool succes = this.rightArm.GoToRelativeCartesianPos(x, y, z, roll, pitch, yaw, elbow);
            this.movingRightArm = false;
            return succes;
        }

        public bool RaGoToPredefPos(string position)
        {
            if (!this.rightPredefPos.ContainsKey(position))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskPlanner: Can't find predefined position for right arm: \"" + position + "\"");
                return false;
            }

            PredefPosition gp = this.rightPredefPos[position];
            
            return this.RaGoToArticularPosition(gp.Q1, gp.Q2, gp.Q3, gp.Q4, gp.Q5, gp.Q6, gp.Q7);
        }

        public bool RaPerformMovement(string movement)
        {
            if (!this.rightPredefMovs.ContainsKey(movement))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskPlanner: Can't find predefined movement for right arm \"" + movement + "\"");
                return false;
            }

            bool success = true;

            foreach (PredefPosition pp in this.rightPredefMovs[movement].Positions)
                success &= this.RaGoToArticularPosition(pp.Q1, pp.Q2, pp.Q3, pp.Q4, pp.Q5, pp.Q6, pp.Q7);

            return success;
        }

        public bool RaGetCartesianPosition(out Vector currentPos)
        {
            currentPos = this.rightArm.GetPositionCartesian();
            if (currentPos == null)
            {
                currentPos = new Vector(7);
                return false;
            }
            else return true;
        }

        public bool RaGetArticularPosition(out Vector currentPos)
        {
            currentPos = this.rightArm.GetPositionArticular();
            if (currentPos == null)
            {
                currentPos = new Vector(7);
                return false;
            }
            else return true;
        }

        public bool RaOpenGripper(double percentage)
        {
            this.movingRightArm = true;
            bool succes = this.rightArm.OpenGripper(percentage);
            this.movingRightArm = false;
            return succes;
        }

        public bool RaCloseGripper(double force)
        {
            this.movingRightArm = true;
            bool succes = this.rightArm.CloseGripper(force);
            this.movingRightArm = false;
            return succes;
        }

        public bool RaTorque(bool enable)
        {
            return this.rightArm.EnableTorque(enable);
        }

        public bool RaIsReachablePos(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            Vector res;
            Vector pos = new Vector(x, y, z, roll, pitch, yaw, elbow);
            return this.rightArm.InverseKinematics(pos, out res);
		}
		#endregion

		#region Left arm
		public bool LaGoToCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            this.movingLeftArm = true;
            this.optLeftPath.CalculateOptimalPath();
            bool succes = this.leftArm.GoToCartesianPosition(x, y, z, roll, pitch, yaw, elbow);
            this.movingLeftArm = false;
            return succes;
        }

        public string LaState()
        {
            
            string state = "";
            
            if (this.movingLeftArm) return "LA: Moving";

            else 
            {
                this.movingLeftArm = true;
                if (!this.leftArm.GetErrorState(out state))
                {
                    this.movingLeftArm = false;
                    return "LA: Can't read Error State";
                }
                else
                {
                    this.movingLeftArm = false;
                    return "LA " + state;
                }

            }
        }
        
        public string RaState()
        {
            string state = "";

            if (this.movingRightArm) return "LA: Moving";

            else
            {
                this.movingRightArm = true;
                if (!this.rightArm.GetErrorState(out state))
                {
                    this.movingRightArm = false;
                    return "LA: Can't read Error State";
                }
                else
                {
                    this.movingRightArm = false;
                    return "LA " + state;
                }

            }
        }

        public bool LaGoToCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw)
        {
            this.movingLeftArm = true;
            this.optLeftPath.CalculateOptimalPath();
            bool succes = this.leftArm.GoToCartesianPosition(x, y, z, roll, pitch, yaw);
            this.movingLeftArm = false;
            return succes;
        }

        public bool LaGoToCartesianPosition(double x, double y, double z)
        {
            this.movingLeftArm = true;
            this.optLeftPath.CalculateOptimalPath();
            bool succes = this.leftArm.GoToCartesianPosition(x, y, z);
            this.movingLeftArm = false;
            return succes;
        }

        public bool LaGoToArticularPosition(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
        {
            this.movingLeftArm = true;
            this.optRightPath.CalculateOptimalPath();
            bool succes = this.leftArm.GoToArticularPosition(q1, q2, q3, q4, q5, q6, q7);
            this.movingLeftArm = false;
            return succes;
        }

        public bool LaGoToRelativeCartesianPos(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            this.movingLeftArm = true;
            bool succes = this.leftArm.GoToRelativeCartesianPos(x, y, z, roll, pitch, yaw, elbow);
            this.movingLeftArm = false;
            return succes;
        }

        public bool LaGoToPredefPos(string position)
        {
            if (!this.leftPredefPos.ContainsKey(position))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskPlanner: Can't find predefined position for left arm : \"" +
                    position + "\"");
            }

            PredefPosition gp = this.leftPredefPos[position];

            return this.LaGoToArticularPosition(gp.Q1, gp.Q2, gp.Q3, gp.Q4, gp.Q5, gp.Q6, gp.Q7);
        }

        public bool LaPerformMovement(string movement)
        {
            if (!this.leftPredefMovs.ContainsKey(movement))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskPlanner: Can't find predefined movement for left arm \"" + movement + "\"");
                return false;
            }

            bool success = true;

            foreach (PredefPosition pp in this.leftPredefMovs[movement].Positions)
                success &= this.LaGoToArticularPosition(pp.Q1, pp.Q2, pp.Q3, pp.Q4, pp.Q5, pp.Q6, pp.Q7);

            return success;
        }

        public bool LaGetCartesianPosition(out Vector currentPos)
        {
            currentPos = this.leftArm.GetPositionCartesian();
            if (currentPos == null)
            {
                currentPos = new Vector(7);
                return false;
            }
            else return true;
        }

        public bool LaGetArticularPosition(out Vector currentPos)
        {
            currentPos = this.leftArm.GetPositionArticular();
            if (currentPos == null)
            {
                currentPos = new Vector(7);
                return false;
            }
            else return true;
        }

        public bool LaOpenGripper(double percentage)
        {
            this.movingLeftArm = true;
            bool succes = this.leftArm.OpenGripper(percentage);
            this.movingLeftArm = false;
            return succes;
        }

        public bool LaCloseGripper(double force)
        {
            this.movingLeftArm = true;
            bool succes = this.leftArm.CloseGripper(force);
            this.movingLeftArm = false;
            return succes;
        }

        public bool LaTorque(bool enable)
        {
            return this.leftArm.EnableTorque(enable);
        }

        public bool LaIsReachablePos(double x, double y, double z, double roll, double pitch, double yaw, double elbow)
        {
            Vector res;
            Vector pos = new Vector(x, y, z, roll, pitch, yaw, elbow);
            return this.leftArm.InverseKinematics(pos, out res);
		}
		#endregion

		public bool AddMapObstacle()
        {
            return true;
        }

        public bool MovingLeftArm { get { return this.movingLeftArm; } }

        public bool MovingRightArm { get { return this.movingRightArm; } }

		
	}
}
