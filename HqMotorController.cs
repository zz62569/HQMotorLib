using NModbus;
using NModbus.Serial;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace HQMotorLib
{
    public class HqMotorController : IHqMotorController
    {
        private byte[] ToModbus(byte[] byteData, int byteLength)
        {
            byte[] CRC = new byte[2];

            UInt16 wCrc = 0xFFFF;
            for (int i = 0; i < byteLength; i++)
            {
                wCrc ^= Convert.ToUInt16(byteData[i]);
                for (int j = 0; j < 8; j++)
                {
                    if ((wCrc & 0x0001) == 1)
                    {
                        wCrc >>= 1;
                        wCrc ^= 0xA001;//异或多项式
                    }
                    else
                    {
                        wCrc >>= 1;
                    }
                }
            }

            CRC[0] = (byte)((wCrc & 0xFF00) >> 8);//高位在后
            CRC[1] = (byte)(wCrc & 0x00FF);       //低位在前
            return CRC;

        }
        private SerialPort? _serialPort;
        private HqMotorControllerModel MotorHubModel { set; get; } = new();
        private double _frqratio;
        public bool _readbool { get; set; } = false;
        private double _offset = 0.0;
        private double motorangle = 0.0;
        public double outputangle { get; set; } = 0.0;
        public double outputrange { get; set; } = 0.0;
        //private IModbusMaster? master { get; set; }
        private double signaloriginaldata = 0;
        private double duoriginaldata = 0;
        public long AbandonData { get; set; } = 0;
        //private string _hubtype { get; set; } = string.Empty;
        public HqMotorController()
        {
            _frqratio = 65536.0 / 360.0;
        }
        private async void ReadFunc()
        {
            await Task.Run(async () => {
                while (true)
                {
                    if (_readbool)
                    {
                        _serialPort!.Write(MotorHubModel.ReadAngleCmd_signal,
                                0,
                                MotorHubModel.ReadAngleCmd_signal.Length);
                        await Task.Delay(5);
                    }
                    await Task.Delay(5);
                }
            });
        }
        public async Task InitHub(string com, int baudrate)
        {
            try
            {
                //_hubtype = com;
                _serialPort = new SerialPort(com, baudrate, Parity.None, 8, StopBits.One)
                {
                    DtrEnable = true,
                    RtsEnable = true,
                    ReadBufferSize = 3900
                };
                _serialPort.DataReceived += SerialPort_DataReceived;
                _serialPort.Open();
                //var factory = new ModbusFactory();
                //master = factory.CreateRtuMaster(_serialPort);
                await MotorInit();
                await Task.Delay(500);
                await MotorFm1Init();
                await Task.Delay(200);
                await MotorFm2Init();
                await Task.Delay(200);
                await MotorFm3Init();
                await Task.Delay(200);
                await SpeedInit();
                await Task.Delay(200);
                await SetBeOne();
                await Task.Delay(200);
                ReadFunc();
                _readbool = true;
                //await _hubCallerClients!.All.SendAsync("LinkResult", "Ok");
            }
            catch { }
        }
        public double offsetmotor { get; set; } = 0;
        public Task DeInitHub()
        {
            _serialPort!.Close();
            _serialPort.Dispose();
            return Task.CompletedTask;
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            var port = (sender as SerialPort)!;
            byte[] data;
            var length = port.BytesToRead;
            data = new byte[port.BytesToRead];
            if (length > 12)
            {
                _serialPort!.Read(data, 0, data.Length);
                if (_readbool && data[1] == 0x03)
                {
                    var nowdata = (data[5] * 256 + data[6]) * 65536.0
                     + (data[3] * 256.0 + data[4]);
                    var nowrange = (data[9] * 256 + data[10]) * 65536.0
                     + (data[7] * 256.0 + data[8]);
                    if (nowdata != signaloriginaldata)
                    {
                        //Trace.WriteLine(((signaloriginaldata / 10000.0 * 360 - offsetmotor) / 192.0 * 10).ToString("f3") + " " + DateTime.Now);
                        signaloriginaldata = (data[5] * 256 + data[6]) * 65536.0
                         + (data[3] * 256.0 + data[4]);
                        zAngleList.Add(outputangle);
                    }
                    if (duoriginaldata != nowrange)
                    {
                        duoriginaldata = nowrange;
                    }
                    outputangle = (nowdata / 10000.0 * 360) / 192.0 * 10 - offsetmotor;
                    //Console.WriteLine("原始值：" + signaloriginaldata + "|"  + duoriginaldata);
                }
            }
        }
        public List<double> zAngleList { set; get; } = new();
        #region Just about motor
        public Task MotorDInit()
        {
            _serialPort!.Write(MotorHubModel.HQMotorDeInit,
                    0,
                    MotorHubModel.HQMotorDeInit.Length);
            return Task.CompletedTask;
        }
        public Task MotorStop()
        {
            _serialPort!.Write(MotorHubModel.HQMotorStop,
                    0,
                    MotorHubModel.HQMotorStop.Length);
            return Task.CompletedTask;
        }
        public Task MotorInit()
        {
            _serialPort!.Write(MotorHubModel.HQMotorInit,
                    0,
                    MotorHubModel.HQMotorInit.Length);
            return Task.CompletedTask;
        }
        #region Init 1.2.3
        private Task MotorFm1Init()
        {
            _serialPort!.Write(MotorHubModel.HQMotorFm1Init,
                    0,
                    MotorHubModel.HQMotorFm1Init.Length);
            return Task.CompletedTask;
        }
        private Task MotorFm2Init()
        {
            _serialPort!.Write(MotorHubModel.HQMotorFm2Init,
                    0,
                    MotorHubModel.HQMotorFm2Init.Length);
            return Task.CompletedTask;
        }
        private Task MotorFm3Init()
        {
            _serialPort!.Write(MotorHubModel.HQMotorFm3Init,
                    0,
                    MotorHubModel.HQMotorFm3Init.Length);
            return Task.CompletedTask;
        }
        #endregion
        public Task MotorStart()
        {
            _serialPort!.Write(MotorHubModel.HQMotorStart,
                    0,
                    MotorHubModel.HQMotorStart.Length);
            return Task.CompletedTask;
        }
        private Task SpeedInit()
        {
            _serialPort!.Write(MotorHubModel.HQMotorSppedInit,
                    0,
                    MotorHubModel.HQMotorSppedInit.Length);
            return Task.CompletedTask;
        }
        public Task SetSpeed(int speed)
        {
            MotorHubModel.SetSpeed[7] = (byte)(speed / 256);
            MotorHubModel.SetSpeed[8] = (byte)(speed % 256);
            byte[] data = ToModbus(MotorHubModel.SetSpeed,
                MotorHubModel.SetSpeed.Length - 2);
            MotorHubModel.SetSpeed[MotorHubModel.SetSpeed.Length - 1] = data[0];
            MotorHubModel.SetSpeed[MotorHubModel.SetSpeed.Length - 2] = data[1];
            _serialPort!.Write(MotorHubModel.SetSpeed,
                    0,
                    MotorHubModel.SetSpeed.Length);
            return Task.CompletedTask;
        }
        public Task SetInstance(string instanace)
        {
            var _instance = (int)(double.Parse(instanace) * _frqratio);
            if (_instance > 0)
            {
                int bighigh = _instance / 65536;
                int lowhigh = _instance % 65536;


                MotorHubModel.SetInstanceData[11] = (byte)(lowhigh / 256);
                MotorHubModel.SetInstanceData[12] = (byte)(lowhigh % 256);

                MotorHubModel.SetInstanceData[13] = (byte)(bighigh / 256);
                MotorHubModel.SetInstanceData[14] = (byte)(bighigh % 256);
            }
            else
            {
                var instanaced = (4294967296 + _instance);    //4174967296
                int bighigh = (int)(instanaced / 65536);
                int lowhigh = (int)(instanaced - bighigh * 65536);

                MotorHubModel.SetInstanceData[11] = (byte)(lowhigh / 256);
                MotorHubModel.SetInstanceData[12] = (byte)(lowhigh % 256);

                MotorHubModel.SetInstanceData[13] = (byte)(bighigh / 256);
                MotorHubModel.SetInstanceData[14] = (byte)(bighigh % 256);
            }

            byte[] data = ToModbus(MotorHubModel.SetInstanceData,
                MotorHubModel.SetInstanceData.Length - 2);
            MotorHubModel.SetInstanceData[MotorHubModel.SetInstanceData.Length - 1] = data[0];
            MotorHubModel.SetInstanceData[MotorHubModel.SetInstanceData.Length - 2] = data[1];
            _serialPort!.Write(MotorHubModel.SetInstanceData,
                    0,
                    MotorHubModel.SetInstanceData.Length);
            return Task.CompletedTask;
        }

        public Task CalculateBuffer(byte[]? buffer1, byte[]? buffer2)
        {
            if (buffer1 is not null && buffer2 is not null)
            {
                motorangle = (buffer1[3] * 256 + buffer1[4] + (buffer2[3] * 256 + buffer2[4]) * 65536) / 65536.0 * 360.0;
                var ag = motorangle + _offset;
                if (ag > 360)
                    outputangle = ag - 360;
                else if (ag < 0)
                    outputangle = ag + 360;
                else
                    outputangle = ag;
            }
            return Task.CompletedTask;
        }
        #endregion
        #region SetZero
        public Task MotorSerZero(double offset)
        {
            var ag = (int)(motorangle) / 360;
            _offset = -(motorangle - 360 * ag);
            //_serialPort!.DiscardInBuffer();
            //zAngleList.Clear();
            return Task.CompletedTask;
        }
        public Task SetBeOne()
        {
            _serialPort!.Write(MotorHubModel.SetbeOne,
                    0,
                    MotorHubModel.SetbeOne.Length);
            return Task.CompletedTask;
        }
        #endregion
    }
}
