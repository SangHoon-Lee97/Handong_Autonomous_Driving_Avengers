// CallbackHandler.cpp

#include "Include.h"
#include "Callback.h"

void Callback(void) {

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
}