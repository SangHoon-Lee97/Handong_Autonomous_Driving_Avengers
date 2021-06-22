#include "Include.h"
#include "ConfigureSensor.h"
#include "Callback.h"


void ConfigureSensor(void) {

    Journaller* gJournal = 0;

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
            assert(packet != 0); // if false (= 0) program is aborted, packet = 0ÀÏ¶§
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
      //return -1;
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

};
