#include "connections/MyFakeCan.h"

namespace {
    constexpr int DEFAULT_PORT = 114514;
}

MyFakeCan::MyFakeCan(QString port)
    : CANConnection(port, "MyFakeCan", CANCon::MY_FAKE_CAN, 0, 0, false, 0, 1, 4000, true)
{
    sendDebug("MyFakeCan()");

    if (port.length() > 0) {
        bool bIsOk = false;
        m_Port = port.toInt(&bIsOk);
        if (!bIsOk) {
            m_Port = DEFAULT_PORT;
        }
    }
    else {
        m_Port = DEFAULT_PORT;
    }

    mNumBuses = 1;
    mBusData.resize(mNumBuses);

    setStatus(CANCon::CONNECTED);
    CANConStatus stats;
    stats.conStatus = getStatus();
    stats.numHardwareBuses = mNumBuses;
    emit status(stats);

    for (int i = 0; i < mNumBuses; i++)
    {
        CANBus bus_info;

        bus_info.setActive(true);
        bus_info.setListenOnly(false);
        bus_info.setSpeed(500000);
        setBusConfig(i, bus_info);
    }
}

MyFakeCan::~MyFakeCan()
{

}

void MyFakeCan::piStarted()
{
    qDebug() << "MyFakeCon: " << "Connecting...";

    if (m_pTimer == nullptr) {
        m_pTimer = createTimer();
        connect(m_pTimer, &QTimer::timeout, this, &MyFakeCan::generateFakeFrame);
    }
    m_pTimer->start();
}

void MyFakeCan::piStop()
{
    disconnectDevice();

    if (m_pTimer) {
        m_pTimer->stop();
    }
}

void MyFakeCan::piSetBusSettings(int pBusIdx, CANBus pBus)
{

    /* sanity checks */
    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    /* copy bus config */
    setBusConfig(pBusIdx, pBus);
}

bool MyFakeCan::piGetBusSettings(int pBusIdx, CANBus &pBus)
{

    return getBusConfig(pBusIdx, pBus);
}

void MyFakeCan::piSuspend(bool pSuspend)
{

    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}

bool MyFakeCan::piSendFrame(const CommFrame& pFrame)
{
    CommFrame* frame_p = getQueue().get();
    if(frame_p)
    {
        *frame_p = pFrame;
        getQueue().queue();
    }

    return true;
}

void MyFakeCan::disconnectDevice()
{
    qDebug() << "MyFakeCon: " << "Disconnecting...";

    setStatus(CANCon::NOT_CONNECTED);
    CANConStatus stats;
    stats.conStatus = getStatus();
    stats.numHardwareBuses = mNumBuses;
    emit status(stats);
}

void MyFakeCan::generateFakeFrame()
{
    /* drop frame if capture is suspended */
    if(isCapSuspended())
        return;

    CommFrame* frame_p = getQueue().get();
    if(frame_p)
    {
        QByteArray fakePayload;
        fakePayload.append(0xF0);
        fakePayload.append(0xF0);
        fakePayload.append(0xF0);
        fakePayload.append(0xF0);

        frame_p->setPayload(fakePayload);
        frame_p->setBus(0);
        frame_p->setExtendedFrameFormat(0);
        frame_p->setFrameId(0x0CF004FE);
        frame_p->setFrameType(CommFrame::CANDataFrame);
        frame_p->setReceived(true);
        frame_p->setTimeStamp(CommFrame::TimeStamp::fromMicroSeconds(QDateTime::currentMSecsSinceEpoch()));

        checkTargettedFrame(*frame_p);

        /* enqueue frame */
        getQueue().queue();

        sendDebug("Fake frame generated!");
    }
}

void MyFakeCan::sendDebug(const QString debugText)
{
    qDebug() << debugText;
    debugOutput(debugText);
}

QTimer *MyFakeCan::createTimer()
{
    QTimer* pTimer = new QTimer();
    pTimer->setInterval(100); //tick every 0.1 seconds
    pTimer->setSingleShot(false); //keep ticking
    return pTimer;
}
