/*  
    Copyright (C) 2013  Soroush Falahati - soroush@falahati.net

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see [http://www.gnu.org/licenses/].
    */

using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using System.Windows.Threading;
using Microsoft.Kinect;
using Microsoft.Win32;
using NiUI.Properties;

namespace NiUI
{
    // ReSharper disable once InconsistentNaming
    public sealed partial class frm_Main
    {
        private const int smallBlockSize = 16;
        private const int bigBlockSize = 80;
        private const int depthLimit = 8000;

        private readonly BitmapBroadcaster _broadcaster;

        private object bitmapLock = new object();

        private Rectangle _currentCropping = Rectangle.Empty;

        private KinectSensor _currentDevice;

        private int _noClientTicker;

        private bool _isIdle = true;

        private bool _softMirror;

        private object depthLock = new object();
        private short[] depthBytes;

        private byte[] background;
        private int[] depthMapping;
        private int[] smallBlockMapping;
        private int[] bigBlockMapping;

        private int counter;

        public bool IsAutoRun { get; set; }

        // ReSharper disable once FlagArgument
        private static bool HandleError(KinectStatus status)
        {
            if (status == KinectStatus.Connected)
            {
                return true;
            }

            MessageBox.Show(
                string.Format("Error: status {0}", status),
                @"Error",
                MessageBoxButtons.OK,
                MessageBoxIcon.Asterisk);

            return false;
        }

        private static void RegisterFilter()
        {
            var filterAddress = Path.Combine(Environment.CurrentDirectory, "NiVirtualCamFilter.dll");

            if (!File.Exists(filterAddress))
            {
                MessageBox.Show(
                    @"NiVirtualCamFilter.dll has not been found. Please reinstall this program.",
                    @"Fatal Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);
                Environment.Exit(1);
            }

            try
            {
                var p = new Process
                {
                    StartInfo =
                        new ProcessStartInfo("regsvr32.exe", "/s \"" + filterAddress + "\"")
                };
                p.Start();
                p.WaitForExit();
            }
            catch
            {
                // ignored
            }
        }

        private static byte[] PadLines(byte[] bytes, int rows, int columns)
        {
            int currentStride = columns; // 3
            int newStride = columns;  // 4
            byte[] newBytes = new byte[newStride * rows];
            for (int i = 0; i < rows; i++)
                Buffer.BlockCopy(bytes, currentStride * i, newBytes, newStride * i, currentStride);
            return newBytes;
        }


        private void CurrentSensorOnNewFrame(byte[] data, int bytesPerPixel)
        {
            try
            {
                Bitmap im;
                lock (bitmapLock)
                {
                    if (depthBytes != null)
                    {
                        var smallBlockQuality = new int[640 / smallBlockSize * (480 / smallBlockSize)];
                        var bigBlockQuality = new int[640 / bigBlockSize * (480 / bigBlockSize)];
                        for (int i = 0; i < 640 * 480; i++)
                        {
                            var depthIndex = depthMapping[i];
                            var depthValue = depthBytes[depthIndex];
                            if (depthValue < depthLimit && depthIndex > 0 && depthValue > 0)
                            {
                                smallBlockQuality[smallBlockMapping[i]] += 3;
                                bigBlockQuality[bigBlockMapping[i]] += 3;
                            }
                            else if (depthValue == 0)
                            {
                                smallBlockQuality[smallBlockMapping[i]] += 2;
                                bigBlockQuality[bigBlockMapping[i]] += 2;
                            }
                            else if (depthValue == -1)
                            {
                                smallBlockQuality[smallBlockMapping[i]] += 1;
                                bigBlockQuality[bigBlockMapping[i]] += 1;
                            }
                            else if (depthValue >= depthLimit)
                            {
                                smallBlockQuality[smallBlockMapping[i]] -= 3;
                                bigBlockQuality[bigBlockMapping[i]] -= 3;
                            }
                        }
                        im = new Bitmap(640, 480, PixelFormat.Format24bppRgb);
                        BitmapData bmapdata = im.LockBits(
                            new Rectangle(0, 0, 640, 480),
                            ImageLockMode.WriteOnly,
                            im.PixelFormat);
                        IntPtr ptr = bmapdata.Scan0;
                        for (int i = 0; i < 640 * 480; i++)
                        {
                            var depthIndex = depthMapping[i];
                            var depthValue = depthBytes[depthIndex];
                            var bigBlockQuali = bigBlockQuality[bigBlockMapping[i]];
                            var smalllBockQuali = smallBlockQuality[smallBlockMapping[i]];
                            if ((depthValue < depthLimit) && (depthIndex > 0 && depthValue > 0 || smalllBockQuali > 0 || (smalllBockQuali == 0 && bigBlockQuali > 0) || bigBlockQuali > bigBlockSize * bigBlockSize))
                            {
                                Marshal.Copy(data, i * 4, ptr + i * 3, 3);
                            }
                            else
                            {
                                Marshal.Copy(background, i * 3, ptr + i * 3, 3);
                            }
                        }
                        im.UnlockBits(bmapdata);
                    }
                    else
                    {
                        im = new Bitmap(640, 480, bytesPerPixel == 4 ? PixelFormat.Format32bppRgb: PixelFormat.Format24bppRgb);
                        BitmapData bmapdata = im.LockBits(
                            new Rectangle(0, 0, 640, 480),
                            ImageLockMode.WriteOnly,
                            im.PixelFormat);
                        IntPtr ptr = bmapdata.Scan0;
                        Marshal.Copy(data, 0, ptr, data.Length);
                        im.UnlockBits(bmapdata);
                    }

                    var position = new Rectangle(new Point(0, 0), im.Size);

                    if (_currentCropping == Rectangle.Empty)
                    {
                        _currentCropping = position;
                    }

                    if (_softMirror)
                    {
                        im.RotateFlip(RotateFlipType.RotateNoneFlipX);
                    }

                    if (!_isIdle)
                    {
                        if (im.PixelFormat != PixelFormat.Format24bppRgb)
                        {
                            im = im.Clone(new Rectangle(0, 0, 640, 480), PixelFormat.Format24bppRgb);
                        }

                        _broadcaster.SendBitmap(im);
                    }
                }

                BeginInvoke(
                    (Action) delegate
                    {
                        var img = im;
                        if (!_isIdle)
                        {
                            lock (bitmapLock)
                            {
                                if (img.PixelFormat != PixelFormat.DontCare)
                                {
                                    pb_image.Image?.Dispose();
                                    pb_image.Image = new Bitmap(img, pb_image.Size);
                                    pb_image.Refresh();
                                }
                            }
                        }
                    });
            }
            catch (Exception e)
            {
                var newBitmap = new Bitmap(640, 480, PixelFormat.Format24bppRgb);
                if (!_isIdle)
                {
                    _broadcaster.SendBitmap(newBitmap);
                }

                BeginInvoke(
                    (Action) delegate
                    {
                        if (!_isIdle)
                        {
                            lock (bitmapLock)
                            {
                                pb_image.Image?.Dispose();
                                pb_image.Image = newBitmap;
                                pb_image.Refresh();
                            }
                        }
                    });
            }
        }

        private void DeviceChanged()
        {
            cb_type.Items.Clear();
            gb_ir.Enabled = false;
            gb_color.Enabled = false;
            gb_depth.Enabled = false;
            cb_smart.Enabled = false;

            if (cb_device.SelectedItem is string deviceId)
            {
                var isNewDevice = _currentDevice == null || deviceId != _currentDevice.UniqueKinectId;
                _currentDevice?.Stop();
                var newDevice = isNewDevice ? KinectSensor.KinectSensors.Single(x => x.UniqueKinectId == deviceId) : _currentDevice;

                angle_slider.Minimum = newDevice?.MinElevationAngle ?? int.MinValue;
                angle_slider.Maximum = newDevice?.MaxElevationAngle ?? int.MaxValue;

                foreach (var mode in Enum.GetValues(typeof(SensorMode)))
                {
                    cb_type.Items.Add(mode);
                }
                gb_color.Enabled = true;
                gb_ir.Enabled = true;
                gb_depth.Enabled = true;
                cb_smart.Enabled = true;

                if (cb_type.Items.Count < 0)
                {
                    cb_type.SelectedIndex = 0;
                }

                _currentDevice = newDevice;
            }
        }

        private void Init()
        {
            try
            {
                var sensors = KinectSensor.KinectSensors;
                KinectSensor.KinectSensors.StatusChanged += StatusChanged;
                UpdateDevicesList();
                notify.Visible = !Settings.Default.AutoNotification;
                ReadSettings();
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                    string.Format("Fatal Error: {0}", ex.Message),
                    @"Execution Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);
                Environment.Exit(1);
            }
        }

        private void StatusChanged(object sender, StatusChangedEventArgs e)
        {
            if (e.Sensor.UniqueKinectId == _currentDevice.UniqueKinectId)
            {
                switch (e.Status)
                {
                    case KinectStatus.Undefined:
                    case KinectStatus.Disconnected:
                    case KinectStatus.Error:
                    case KinectStatus.NotPowered:
                    case KinectStatus.DeviceNotGenuine:
                    case KinectStatus.DeviceNotSupported:
                    case KinectStatus.InsufficientBandwidth:
                        HandleError(e.Status);
                        Stop(false);
                        return;
                    case KinectStatus.NotReady:
                    case KinectStatus.Initializing:
                        _broadcaster.SendBitmap(Resources.PleaseWait);
                        return;
                    case KinectStatus.Connected:
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }

                btn_stopstart.Text = @"Stop Streaming";
                _isIdle = false;
                notify.Visible = true;
            }
        }

        private void DoesNeedHalt()
        {
            if (_broadcaster != null)
            {
                if (_broadcaster.HasClient || Visible)
                {
                    _noClientTicker = 0;

                    if (_isIdle)
                    {
                        _broadcaster.SendBitmap(Resources.PleaseWait);

                        if (Start())
                        {
                            _isIdle = false;
                        }
                        else
                        {
                            _broadcaster.ClearScreen();
                        }
                    }
                }
                else
                {
                    _noClientTicker++;

                    if (_noClientTicker > 60 && !_isIdle) // 1min of no data
                    {
                        _isIdle = true;
                        Stop(false);
                    }
                }
            }
        }

        private void ReadSettings()
        {
            cb_device.SelectedIndex = -1;

            if (!Settings.Default.DeviceURI.Equals(string.Empty))
            {
                foreach (var item in cb_device.Items)
                {
                    if (item is string id && id.Equals(
                            Settings.Default.DeviceURI,
                            StringComparison.CurrentCultureIgnoreCase))
                    {
                        cb_device.SelectedItem = item;
                    }
                }

                DeviceChanged();
                cb_type.SelectedIndex = -1;

                if (Settings.Default.CameraType != -1)
                {
                    foreach (var item in cb_type.Items)
                    {
                        if (item is SensorMode casted)
                        {
                            if (Settings.Default.CameraType == (int)casted)
                            {
                                cb_type.SelectedItem = item;
                            }
                        }
                    }
                }
            }

            cb_notification.Checked = Settings.Default.AutoNotification;
            cb_hd.Checked = Settings.Default.Color_HD;
            cb_fill.Checked = Settings.Default.Depth_Fill;
            cb_equal.Checked = Settings.Default.Depth_Histogram;
            cb_invert.Checked = Settings.Default.Depth_Invert;
            cb_mirror.Checked = Settings.Default.Mirroring;
            cb_smart.Checked = Settings.Default.SmartCam;
            angle_slider.Value = Settings.Default.Angle;

            try
            {
                var registryKey =
                    Registry.CurrentUser.OpenSubKey(@"Software\Microsoft\Windows\CurrentVersion\Run");

                if (registryKey?.GetValue("OpenNI Virtual Webcam Server") != null)
                {
                    cb_startup.Checked = true;
                }
            }
            catch
            {
                // ignored
            }
        }

        private void SaveSettings()
        {
            Settings.Default.DeviceURI = "";

            if (cb_device.SelectedItem is string info)
            {
                Settings.Default.DeviceURI = info;
            }

            Settings.Default.CameraType = -1;

            if (cb_type.SelectedItem is SensorMode selectedType)
            {
                Settings.Default.CameraType = (int) selectedType;
            }

            Settings.Default.AutoNotification = cb_notification.Checked;
            Settings.Default.Color_HD = cb_hd.Checked;
            Settings.Default.Depth_Fill = cb_fill.Checked;
            Settings.Default.Depth_Histogram = cb_equal.Checked;
            Settings.Default.Depth_Invert = cb_invert.Checked;
            Settings.Default.Mirroring = cb_mirror.Checked;
            Settings.Default.SmartCam = cb_smart.Checked;
            Settings.Default.Angle = angle_slider.Value;
            Settings.Default.Save();

            try
            {
                var registryKey =
                    Registry.CurrentUser.OpenSubKey(@"Software\Microsoft\Windows\CurrentVersion\Run", true);

                if (registryKey != null)
                {
                    if (cb_startup.Checked)
                    {
                        registryKey.SetValue(
                            "OpenNI Virtual Webcam Server",
                            "\"" + Process.GetCurrentProcess().MainModule.FileName + "\" /autoRun");
                    }
                    else
                    {
                        registryKey.DeleteValue("OpenNI Virtual Webcam Server");
                    }
                }
            }
            catch
            {
                // ignored
            }
        }

        private bool Start()
        {
            RegisterFilter();

            if (_isIdle && _broadcaster.HasServer())
            {
                MessageBox.Show(
                    @"Only one server is allowed.",
                    @"Multi-Server",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);

                return false;
            }

            if (_currentDevice == null)
            {
                MessageBox.Show(
                    @"Please select a device to open and then click Apply.",
                    @"Device Open",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning);

                return false;
            }

            var isSameDevice = _currentDevice.UniqueKinectId == Settings.Default.DeviceURI;

            if (!isSameDevice)
            {
                if (Settings.Default.DeviceURI == string.Empty)
                {
                    _currentDevice = null;
                    MessageBox.Show(
                        @"Please select a device to open and then click Apply.",
                        @"Device Open",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Warning);

                    return false;
                }
            }


            _currentDevice.ColorFrameReady -= NewColorFrame;
            _currentDevice.DepthFrameReady -= NewDepthFrame;

            _currentDevice.Start();
            _currentDevice.ElevationAngle = Settings.Default.Angle;
            if (Settings.Default.CameraType == (int)SensorMode.NoBackground)
            {
                var bitmap = new Bitmap("T:\\temp\\BG.bmp");
                var data = bitmap.LockBits(
                    new Rectangle(0, 0, 640, 480),
                    ImageLockMode.ReadOnly,
                    bitmap.PixelFormat);
                IntPtr ptr = data.Scan0;
                background = new byte[data.Height * data.Stride];
                Marshal.Copy(ptr, background, 0, data.Height * data.Stride);
                bitmap.UnlockBits(data);
                counter = 100000;
                smallBlockMapping = new int[640 * 480];
                bigBlockMapping = new int[640 * 480];
                var maxSmallBlock = 640 / smallBlockSize;
                var maxBigBlock = 640 / bigBlockSize;
                for (int x = 0; x < 640; x++)
                {
                    var xSmallBlock = x / smallBlockSize;
                    var xBigBlock = x / bigBlockSize;
                     for (int y = 0; y < 480; y++)
                    {
                        var ySmallBlock = y / smallBlockSize;
                        var yBigBlock = y / bigBlockSize;
                        smallBlockMapping[y * 640 + x] = ySmallBlock * maxSmallBlock + xSmallBlock;
                        bigBlockMapping[y * 640 + x] = yBigBlock * maxBigBlock + xBigBlock;
                    }
                }
            }
            if (Settings.Default.CameraType == (int)SensorMode.Color || Settings.Default.CameraType == (int)SensorMode.NoBackground)
            {
                _currentDevice.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                _currentDevice.ColorFrameReady += NewColorFrame;
                _currentDevice.DepthStream.Disable();
            }

            if (Settings.Default.CameraType == (int)SensorMode.Ir)
            {
                _currentDevice.ColorStream.Enable(ColorImageFormat.InfraredResolution640x480Fps30);
                _currentDevice.ColorFrameReady += NewColorFrame;
            }

            if (Settings.Default.CameraType == (int)SensorMode.Depth || Settings.Default.CameraType == (int)SensorMode.NoBackground)
            {
                _currentDevice.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                _currentDevice.DepthStream.Range = DepthRange.Near;
                _currentDevice.DepthFrameReady += NewDepthFrame;
            }

            _softMirror = Settings.Default.Mirroring;

            //if (Settings.Default.SmartCam)
            //{
            //    try
            //    {
            //        if (!isSameDevice || _userTracker == null || _handTracker == null || !_userTracker.IsValid || !_handTracker.IsValid)
            //        {
            //            _userTracker = UserTracker.Create(_currentDevice);
            //            _handTracker = HandTracker.Create(_currentDevice);
            //            _handTracker.StartGestureDetection(GestureData.GestureType.HandRaise);
            //            _handTracker.OnNewData += NiTeOnNewData;
            //        }
            //    }
            //    catch
            //    {
            //        // ignored
            //    }
            //}

            return true;
        }

        private void NewDepthFrame(object sender, DepthImageFrameReadyEventArgs e)
        {
            var frame = e.OpenDepthImageFrame();
            if (frame == null)
            {
                return;
            }

            if (Settings.Default.CameraType == (int)SensorMode.NoBackground)
            {
                lock (depthLock)
                {
                    if (depthBytes == null)
                    {
                        depthBytes = new short[480 * 640];
                    }
                    frame.CopyPixelDataTo(depthBytes);
                }
                if (++counter > 10)
                {
                    counter = 0;
                    //var colorMapping = new DepthImagePoint[640 * 480];
                    //_currentDevice.CoordinateMapper.MapColorFrameToDepthFrame(
                    //    ColorImageFormat.RgbResolution640x480Fps30, DepthImageFormat.Resolution640x480Fps30,
                    //    frame.GetRawPixelData(), colorMapping);
                    var colorMapping = new ColorImagePoint[640 * 480];
                    _currentDevice.CoordinateMapper.MapDepthFrameToColorFrame(
                        DepthImageFormat.Resolution640x480Fps30, frame.GetRawPixelData(), ColorImageFormat.RgbResolution640x480Fps30, colorMapping);
                    depthMapping = new int[640 *480];
                    for (int i = 0; i < 480 * 640; i++)
                    {
                        var point = colorMapping[i];
                        if (point.X > -1 && point.Y > -1)
                        {
                            depthMapping[point.Y * 640 + point.X] = i;
                        }
                    }

                }
            }
            else
            {
                var depthData = new short[frame.PixelDataLength];
                frame.CopyPixelDataTo(depthData);
                var bitmapData = new byte[640 * 480 * 3];
                for (int i = 0; i < depthData.Length; i++)
                {
                    var bi = 3 * i;
                    switch (depthData[i])
                    {
                        case -8:
                            bitmapData[bi] = 0;
                            bitmapData[bi + 1] = 0;
                            bitmapData[bi + 2] = 255;
                            break;
                        case -1:
                            bitmapData[bi] = 255;
                            bitmapData[bi + 1] = 0;
                            bitmapData[bi + 2] = 255;
                            break;
                        case 0:
                            bitmapData[bi] = 0;
                            bitmapData[bi + 1] = 255;
                            bitmapData[bi + 2] = 255;
                            break;
                        default:
                            var msb = (byte)(depthData[i] / 255);
                            var lsb = (byte)(depthData[i] % 255);
                            bitmapData[bi] = msb;
                            bitmapData[bi + 1] = lsb;
                            bitmapData[bi + 2] = 0;
                            break;
                    }
                }
                CurrentSensorOnNewFrame(bitmapData, 3);
            }
        }

        private void NewColorFrame(object sender, ColorImageFrameReadyEventArgs e)
        {
            var frame = e.OpenColorImageFrame();
            if (frame == null)
            {
                return;
            }

            //if (Settings.Default.CameraType == (int) SensorMode.NoBackground)
            //{
            //    if (depthMapping == null)
            //    {
            //        var colormapping = _currentDevice.CoordinateMapper.MapColorFrameToDepthFrame();
            //    }
            //}

            switch (frame.Format)
            {
                case ColorImageFormat.RgbResolution640x480Fps30:
                    var image = frame.GetRawPixelData();
                    CurrentSensorOnNewFrame(image, 4);
                    break;
                case ColorImageFormat.InfraredResolution640x480Fps30:
                    var irData = frame.GetRawPixelData();
                    var bitmapData = new byte[640 * 480 * 3];
                    for (int i = 0; i < 640 * 480; i++)
                    {
                        var bi = 3 * i;
                        var iri = i * 2;
                        var value = irData[iri + 1];
                        var half = (byte)(value >> 1);
                        bitmapData[bi] = half;
                        bitmapData[bi + 1] = value;
                        bitmapData[bi + 2] = half;
                    }
                    CurrentSensorOnNewFrame(bitmapData, 3);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        private void Stop(bool shouldApply)
        {
            var isSameDevice = shouldApply &&
                               _currentDevice != null;

            if (!isSameDevice)
            {
                if (_currentDevice != null)
                {
                    if (_currentDevice.IsRunning)
                    {
                        _currentDevice.ColorFrameReady -= NewColorFrame;
                        _currentDevice.DepthFrameReady -= NewDepthFrame;
                        depthBytes = null;
                        background = null;
                        depthMapping = null;
                        smallBlockMapping = null;
                        bigBlockMapping = null;
                        _currentDevice.Stop();
                    }
                }
            }

            _isIdle = true;
            btn_stopstart.Text = @"Start Streaming";

            if (!shouldApply)
            {
                _broadcaster.ClearScreen();
                pb_image.Image = null;
                pb_image.Refresh();
            }

            if (Settings.Default.AutoNotification)
            {
                notify.Visible = false;
            }
        }

        private void UpdateDevicesList()
        {
            var devices = KinectSensor.KinectSensors;
            cb_device.Items.Clear();

            if (devices.Count == 0)
            {
                cb_device.Items.Add("None");
            }

            var inList = false;

            foreach (var device in devices)
            {
                cb_device.Items.Add(device.UniqueKinectId);

                if (device.UniqueKinectId == Settings.Default.DeviceURI)
                {
                    inList = true;
                }
            }

            if (!inList)
            {
                Settings.Default.DeviceURI = string.Empty;
            }

            if (cb_device.SelectedIndex == -1)
            {
                cb_device.SelectedIndex = 0;
            }
        }
    }
}