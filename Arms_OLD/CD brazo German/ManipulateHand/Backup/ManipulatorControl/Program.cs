using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace ManipulatorControl
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main(string[] args)
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            frmArmsControl armsControl = new frmArmsControl();
            if (args.Length > 0) parseArgs(armsControl, args);
            Application.Run(armsControl);
        }

        private static void parseArgs(frmArmsControl armsControlForm, string[] args)
        {
            byte resultByte;

            for (int i = 0; i < args.Length; ++i)
            {

                switch (args[i].ToLower())
                {
                    case "-lp":
                        if (++i > args.Length) return;
                        if ((args[i].Substring(0, 3) == "COM") && Byte.TryParse(args[i].Substring(3), out resultByte))
                        {
                            armsControlForm.LeftArmPortName = args[i];
                        }
                        break;
                    case "-rp":
                        if (++i > args.Length) return;
                        if ((args[i].Substring(0, 3) == "COM") && Byte.TryParse(args[i].Substring(3), out resultByte))
                        {
                            armsControlForm.RightArmPortName = args[i];
                        }
                        break;
                }
            }
        }
    }
}
