//--------------------------------------------------------------------------------
// Public Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------

#include    <xscontroller/xscontrol_def.h>
#include    <xscontroller/xsdevice_def.h>
#include    <xscontroller/xsscanner.h>
#include    <xstypes/xsoutputconfigurationarray.h>
#include    <xstypes/xsdatapacket.h>
#include    <xstypes/xstime.h>
#include    <xscommon/xsens_mutex.h>
#include    <iostream>
#include    <iomanip>
#include    <list> 
#include    <string>
#include    <stdio.h>
#include    <cstdlib>
#include    <cmath>
#include    <cstring>
#include    <memory.h>
#include    <conio.h>
#include    <ctime>
#include    <Windows.h>
#include    <tchar.h>
#include    <fstream>
#include    "SharedMemory.h"

using namespace std;

//=====================================================================
//=====================================================================

#define RUNTIME                       (double)             (100)      //[sec]
#define FREQ                          (double)             (10)  //[Hz]
#define N_data                        (unsigned int)       (RUNTIME*FREQ)
#define coordinate_NED                                     (XDI_CoordSysNed)

enum FLAG { COPLIETE = -1, STOP = 0, INIT = 1, START = 2 };

//=====================================================================
//==========================Array name=================================

double euler_x[N_data] = { 0.0 }; //euler angle(roll pitch yaw)
double euler_y[N_data] = { 0.0 };
double euler_z[N_data] = { 0.0 };
double acc_x[N_data]   = { 0.0 };   // Acceleration (x y z)
double acc_y[N_data]   = { 0.0 };   
double acc_z[N_data]   = { 0.0 };  
double vel_x[N_data]   = { 0.0 };  // Velocity (x y z)
double vel_y[N_data]   = { 0.0 };
double vel_z[N_data]   = { 0.0 };
double rot_x[N_data]   = { 0.0 };  // Rate of Turn (x y z)
double rot_y[N_data]   = { 0.0 };
double rot_z[N_data]   = { 0.0 };

double buftime[N_data]          = { 0.0 };
double buftime_output[N_data]   = { 0.0 };
double Deltime[N_data]          = { 0.0 };
double tasktime[N_data]         = { 0.0 };
double buf_index[N_data]        = { 0.0 };
double sm_time[N_data]          = { 0.0 };

double eulerRoll  = 0.0;
double eulerPitch = 0.0;
double eulerYaw   = 0.0;
double accX       = 0.0;
double accY       = 0.0;
double accZ       = 0.0;
double velX       = 0.0;
double velY       = 0.0;
double velZ       = 0.0;
double rotX       = 0.0;
double rotY       = 0.0;
double rotZ       = 0.0;

//=======================================================================================
double iniTime = 0.0;
double curTime = 0.0;
double delTime = 0.0;
double simTime = 0.0;
double task1   = 0.0;
double task2   = 0.0;
double task3   = 0.0;
double Ts      = 1 / FREQ;
int    simcnt  = 0;

//=======================================================================================


enum XsStatusFlag
{
    XSF_SelfTestOk = 0x01      //!< Is set when the self test result was ok
    , XSF_OrientationValid = 0x02      //!< Is set when the computed orientation is valid. The orientation may be invalid during startup or when the sensor data is clipping during violent (for the device) motion
    , XSF_GpsValid = 0x04      //!< Is set when the device has a GPS receiver and the receiver says that there is a GPS position fix.

    , XSF_NoRotationMask = 0x18      //!< If all of these flags are set, the No Rotation algorithm is running
    , XSF_NoRotationAborted = 0x10      //!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm was aborted
    , XSF_NoRotationSamplesRejected = 0x08      //!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running but has rejected samples
    , XSF_NoRotationRunningNormally = 0x18      //!< If all these flags are set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running normally

    , XSF_RepresentativeMotion = 0x20      //!< Indicates if the In-Run Compass Calibration is doing the representative motion analysis

    , XSF_ExternalClockSynced = 0x40      //!< Indicates whether the internal clock is synced with an external clock (Either GNNS or custom provided clock sync)

    , XSF_ClipAccX = 0x00000100
    , XSF_ClipAccY = 0x00000200
    , XSF_ClipAccZ = 0x00000400
    , XSF_ClipGyrX = 0x00000800
    , XSF_ClipGyrY = 0x00001000
    , XSF_ClipGyrZ = 0x00002000
    , XSF_ClipMagX = 0x00004000
    , XSF_ClipMagY = 0x00008000
    , XSF_ClipMagZ = 0x00010000

    , XSF_Retransmitted = 0x00040000   //!< When set Indicates the sample was received as a retransmission
    , XSF_ClippingDetected = 0x00080000   //!< When set Indicates clipping has occurred
    , XSF_Interpolated = 0x00100000   //!< When set Indicates the sample is an interpolation between other samples
    , XSF_SyncIn = 0x00200000   //!< When set indicates a sync-in event has been triggered
    , XSF_SyncOut = 0x00400000   //!< When set Indicates a sync-out event has been generated

    , XSF_FilterMode = 0x03800000   //!< Mask for the 3 bit filter mode field
    , XSF_HaveGnssTimePulse = 0x04000000   //!< Indicates that the 1PPS GNSS time pulse is present

    , XSF_RtkStatus = 0x18000000   //!< Mask for 2 bit RTK status field 00: No RTK; 01: RTK floating; 10: RTK fixed
};

//======================================================================
//===========================function===================================

Journaller* gJournal = 0;
//FILE* pFile;



double   GetWindowTime(void)
{
    LARGE_INTEGER   liEndCounter, liFrequency;

    QueryPerformanceCounter(&liEndCounter);
    QueryPerformanceFrequency(&liFrequency);

    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
};

using namespace std;

class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 5)
        : m_maxNumberOfPacketsInBuffer(maxBufferSize)
        , m_numberOfPacketsInBuffer(0)
    {
    }

    virtual ~CallbackHandler() throw()
    {
    }

    bool packetAvailable() const
    {
        xsens::Lock locky(&m_mutex);
        return m_numberOfPacketsInBuffer > 0;
    }

    XsDataPacket getNextPacket()
    {
        assert(packetAvailable());
        xsens::Lock locky(&m_mutex);
        XsDataPacket oldestPacket(m_packetBuffer.front());
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        return oldestPacket;
    }

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
    {
        xsens::Lock locky(&m_mutex);
        assert(packet != 0); // if false (= 0) program is aborted, packet = 0일때
        while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
            (void)getNextPacket();

        m_packetBuffer.push_back(*packet);
        ++m_numberOfPacketsInBuffer;
        assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer); // if num > max program is aborted
    }
private:
    mutable xsens::Mutex m_mutex;

    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    list<XsDataPacket> m_packetBuffer;
};

//--------------------------------------------------------------------------------

int main(void) 
{
    CreateServerXsens();
    CreateServerFlag();

    int  flag = 1;
    int  count = 0;
    int  filter_mode;
    cout << "Creating XsControl object..." << endl;

    XsControl* control = XsControl::construct(); 
    assert(control != 0);

    // Lambda function for error handling
    auto handleError = [=](string errorString)
    {
        control->destruct();
        cout << errorString << endl;
        cout << "Press [ENTER] to continue." << endl;
        cin.get();
        return -1;
    };

    cout << "Scanning for devices..." << endl;
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

    // Find an MTi device
    XsPortInfo mtPort;
    for (auto const& portInfo : portInfoArray)
    {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
        {
            mtPort = portInfo;
            break;
        }
    }

    if (mtPort.empty())
        return handleError("No MTi device found. Aborting.");

    cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

    cout << "Opening port..." << endl;
    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
        return handleError("Could not open port. Aborting.");

    // Get the device object
    XsDevice* device = control->device(mtPort.deviceId());
    assert(device != 0);

    cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

    // Create and attach callback handler to device
    CallbackHandler callback;
    device->addCallbackHandler(&callback);

    // Put the device into configuration mode before configuring the device
    cout << "Putting device into configuration mode..." << endl;
    if (!device->gotoConfig())
        return handleError("Could not put device into configuration mode. Aborting.");

    cout << "Configuring the device..." << endl;

    // Important for Public XDA!
    // Call this function if you want to record a mtb file:

    device->readEmtsAndDeviceConfiguration();

    //==========================================================================================================
    //============================================================================================================

    XsOutputConfigurationArray configArray;
    if (device->deviceId().isGnss())
    {

        configArray.push_back(XsOutputConfiguration(XDI_EulerAngles, FREQ));
        configArray.push_back(XsOutputConfiguration(XDI_Acceleration, FREQ));
        configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, FREQ));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, FREQ));

    }
    else
    {
        return handleError("Unknown device while configuring. Aborting.");
    }

    if (!device->setOutputConfiguration(configArray))
        return handleError("Could not configure MTi device. Aborting.");

    cout << "Putting device into measurement mode..." << endl;
    if (!device->gotoMeasurement())
        return handleError("Could not put device into measurement mode. Aborting.");

    XsTime::msleep(2000);

    cout << "\nMain loop" << endl;
    cout << string(79, '-') << endl;

    //============================================================================================================
    while (1)
    {
        if (FlagRead(SERV_smdat_Flag) != STOP)
            break;
    }

    iniTime = GetWindowTime() * 0.001; 

    do
    {
        task1 = GetWindowTime() * 0.001;

        if (callback.packetAvailable()) { 
            XsDataPacket packet = callback.getNextPacket();
            if (packet.containsOrientation())
            {
                XsEuler Euler = packet.orientationEuler(coordinate_NED);
                eulerRoll = Euler.roll();
                eulerPitch = Euler.pitch();
                eulerYaw = Euler.yaw();
            }

            if (packet.containsCalibratedAcceleration()) 
            {
                XsVector Acceleration = packet.calibratedAcceleration();
                accX = Acceleration[0];
                accY = Acceleration[1];
                accZ = Acceleration[2];

            }

            if (packet.containsVelocity())
            {
                XsVector Velocity = packet.velocity(coordinate_NED);
                velX = Velocity[0];
                velY = Velocity[1];
                velZ = Velocity[2];
            }

            if (packet.containsRateOfTurnHR())
            {
                XsVector ROT = packet.rateOfTurnHR();
                rotX = ROT[0];
                rotY = ROT[1];
                rotZ = ROT[2];
            }

        }
        task2 = GetWindowTime() * 0.001;

        cout << "Roll : " << eulerRoll << "\n";
        /* memory write*/
        euler_x[simcnt] = eulerRoll;
        euler_y[simcnt] = eulerPitch;
        euler_z[simcnt] = eulerYaw;
        acc_x[simcnt]   = accX;
        acc_y[simcnt]   = accY;
        acc_z[simcnt]   = accZ;
        vel_x[simcnt]   = velX;
        vel_y[simcnt]   = velY;
        vel_z[simcnt]   = velZ;
        rot_x[simcnt]   = rotX;
        rot_y[simcnt]   = rotY;
        rot_z[simcnt]   = rotZ;


        buf_index[simcnt]   = simcnt;
        sm_time[simcnt]     = task2;

        dat_buf_Xsens.SIM_count = simcnt;
        dat_buf_Xsens.SIM_Time  = simTime;

        dat_buf_Xsens.euler_x = eulerRoll;
        dat_buf_Xsens.euler_y = eulerPitch;
        dat_buf_Xsens.euler_z = eulerYaw;
        dat_buf_Xsens.acc_x   = accX;
        dat_buf_Xsens.acc_y   = accY;
        dat_buf_Xsens.acc_z   = accZ;
        dat_buf_Xsens.vel_x   = velX;
        dat_buf_Xsens.vel_y   = velY;
        dat_buf_Xsens.vel_z   = velZ;
        dat_buf_Xsens.rot_x   = rotX;
        dat_buf_Xsens.rot_y   = rotY;
        dat_buf_Xsens.rot_z   = rotZ;

        DataWrite(dat_buf_Xsens);

        while (1) {
            curTime = GetWindowTime() * 0.001;   // [ms]
            delTime = curTime - iniTime - simTime;

            if (delTime >= Ts) {
                break;
            }
        }

        simTime                 = ((double)simcnt + 1.0) * Ts;
        buftime[simcnt]         = simTime;
        buftime_output[simcnt]  = curTime - iniTime;
        Deltime[simcnt]         = delTime;                    
        tasktime[simcnt]        = task2 - task1;             
        simcnt                  = simcnt + 1;

    } while (simTime < RUNTIME);

    cout << "\n" << string(79, '-') << "\n";
    cout << endl;

    cout << "Closing port..." << endl;
    control->closePort(mtPort.portName().toStdString());

    cout << "Freeing XsControl object..." << endl;
    control->destruct();


    //============================================================================================
    // ================================data file 저장 하기========================================

    cout << string(79, '-') << endl;

    //FILE* pFile;
    //pFile = fopen("C:\\Users\\rlawl\\Desktop\\HADA_SI\\COM_2\\Xsens_DataRead\\Time.txt", "w");
    //if (pFile == NULL) {
    //    printf("안됨..\n");
    //    return 1;
    //}
    //for (int r = 0; r < N_data; r++) {

    //   fprintf(pFile, "%d, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n", (int)buf_index[r], (double)euler_x[r], (double)euler_y[r], (double)euler_z[r],(double)acc_x[r], (double)acc_y[r], (double)acc_z[r], (double)vel_x[r], (double)vel_y[r], (double)vel_z[r], (double)rot_x[r], (double)rot_y[r], (double)rot_z[r]);
    //}

    //fclose(pFile);

    cout << "File storing done" << endl;
    cout << string(79, '-') << "\n";

    cout << "Successful exit." << endl;
    cout << "Press [ENTER] to continue." << endl;
    cin.get();

    ClosedServerXsens();
    ClosedServerFlag();

    return 0;
}

void CreateServerXsens(void)
{
    hMampF_SER_Xsens = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
        0, sizeof(Xsens_smdat), SERV_NAME1);

    if (hMampF_SER_Xsens == NULL)
    {
        printf("Could not open file mapping object (%d).\n", GetLastError());
        return;
    }

    SERV_smdat_Xsens = (Xsens_smdat*)MapViewOfFile(hMampF_SER_Xsens, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(Xsens_smdat));
}

void ClosedServerXsens(void)
{
    if (SERV_smdat_Xsens == NULL) return;
    UnmapViewOfFile(SERV_smdat_Xsens);
    CloseHandle(hMampF_SER_Xsens);
}

void  DataWrite(Xsens_smdat data)
{
    if (SERV_smdat_Xsens == NULL) return;
    *SERV_smdat_Xsens = data;

    printf("Write Data->SIM_Count : % d\t SIM_Time : % f\n", data.SIM_count, data.SIM_Time);
    //printf("Write Data -> SIM_Count : %d\t SIM_Time : %f\t SIN_Wave : %f\n", data.SIM_count, data.SIM_Time, data.SIN_Wave);
}

void CreateServerFlag(void)
{
    hMampF_SER_Flag = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
        0, sizeof(ReadData_flag), SERV_NAME3);

    if (hMampF_SER_Flag == NULL)
    {
        printf("Could not open file mapping object (%d).\n", GetLastError());
        return;
    }

    SERV_smdat_Flag = (ReadData_flag*)MapViewOfFile(hMampF_SER_Flag, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ReadData_flag));
}

void ClosedServerFlag(void)
{
    if (SERV_smdat_Flag == NULL) return;
    UnmapViewOfFile(SERV_smdat_Flag);
    CloseHandle(hMampF_SER_Flag);
}

int  FlagRead(ReadData_flag* data)
{
    printf("\nMain COM flag : %d", data->flag);
    return data->flag;
}