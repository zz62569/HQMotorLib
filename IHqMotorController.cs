using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HQMotorLib
{
    public interface IHqMotorController
    {
        Task InitHub(string com, int baudrate);
        double outputangle { get; set; }
        Task MotorDInit();
        Task MotorStop();
        Task MotorInit();
        Task MotorStart();
        Task SetSpeed(int speed);
        Task SetInstance(string instanace);
        Task CalculateBuffer(byte[]? buffer1, byte[]? buffer2);
        Task DeInitHub();
        Task MotorSerZero(double offset);
        bool _readbool { get; set; }
        double outputrange { get; set; }
        double offsetmotor { get; set; }
        List<double> zAngleList { set; get; }
    }
}
