using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;


namespace ManipulatorControl
{
	class CmdArmsTorque : AsyncCommandExecuter
	{
		TaskPlanner taskPlanner;

		public CmdArmsTorque(TaskPlanner taskPlanner)
			: base("arms_torque")
		{
			this.taskPlanner = taskPlanner;
		}

		public override bool ParametersRequired
		{
			get
			{
				return true;
			}
		}

		protected override Response AsyncTask(Command command)
		{
			TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsTorque: Received: " + command.StringToSend);

			bool success = false;
			Response response = null;

			if (command.HasParams)
			{
				string paramLower = command.Parameters.ToLower();
				if (paramLower.Contains("on") || paramLower.Contains("enable") || paramLower.Contains("true"))
					success = this.taskPlanner.ArmsTorqueEnable(true);
				else if (paramLower.Contains("off") || paramLower.Contains("disable") || paramLower.Contains("false"))
					success = this.taskPlanner.ArmsTorqueEnable(false);
				else
				{
					TextBoxStreamWriter.DefaultLog.WriteLine("CmdArmsTorque: Invalid parameters");
					success = false;
				}
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("CmdArmsTorque: Parameters required");
				success = false;
			}

			response = Response.CreateFromCommand(command, success);
			TextBoxStreamWriter.DefaultLog.WriteLine("CmdArmsTorque: Sent response: " + response.StringToSend);
			return response;
		}

		public override void DefaultParameterParser(string[] parameters)
		{
			throw new NotImplementedException();
		}
	}
}
