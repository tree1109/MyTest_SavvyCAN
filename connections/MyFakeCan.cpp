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
    delete m_pElapsedTimer;
    delete m_pFakeBodyMessageTimer;
    delete m_pFakeHeadMessageTimer;
    m_pElapsedTimer = nullptr;
    m_pFakeBodyMessageTimer = nullptr;
    m_pFakeHeadMessageTimer = nullptr;
}

void MyFakeCan::piStarted()
{
    qDebug() << "MyFakeCon: " << "Connecting...";

    startTimer();
}

void MyFakeCan::piStop()
{
    disconnectDevice();

    stopTimer();
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
        frame_p->setReceived(true);

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

void MyFakeCan::generateFakeBodyFrame()
{
    /* drop frame if capture is suspended */
    if(isCapSuspended())
        return;

    CommFrame* frame_p = getQueue().get();
    if(frame_p)
    {
        // Fake data.
        QByteArray fakePayload;
        {
            const double weightF = 100 + 25 * std::cos(getElapsedSecond() / 5);
            const double heightF = 100 + 50 * std::sin(getElapsedSecond() / 10);
            const double temperatureF = std::fmod(getElapsedSecond(), 100) * 100;
            const uint16_t weight = static_cast<uint16_t>(weightF);
            const uint16_t height = static_cast<uint16_t>(heightF);
            const uint16_t temperature = static_cast<uint16_t>(temperatureF);

            fakePayload.append(weight & 0x00FF);
            fakePayload.append((weight & 0xFF00) >> 8);
            fakePayload.append(height & 0x00FF);
            fakePayload.append((height & 0xFF00) >> 8);
            fakePayload.append(temperature & 0x00FF);
            fakePayload.append((temperature & 0xFF00) >> 8);
        }

        frame_p->setPayload(fakePayload);
        frame_p->setBus(0);
        frame_p->setExtendedFrameFormat(0);
        frame_p->setFrameId(100);
        frame_p->setFrameType(CommFrame::CANDataFrame);
        frame_p->setReceived(true);
        frame_p->setTimeStamp(CommFrame::TimeStamp::fromMicroSeconds(QDateTime::currentMSecsSinceEpoch() * 1000ul));

        checkTargettedFrame(*frame_p);

        /* enqueue frame */
        getQueue().queue();
    }
}

void MyFakeCan::generateFakeHeadFrame()
{
    /* drop frame if capture is suspended */
    if(isCapSuspended())
        return;

    CommFrame* frame_p = getQueue().get();
    if(frame_p)
    {
        // Fake data.
        QByteArray fakePayload;
        {
            const double temperatureF = std::fmod(getElapsedSecond(), 60) / 60 * 1000 * 60;
            const uint32_t temperature = static_cast<uint32_t>(temperatureF);

            fakePayload.append(temperature & 0xFF);
            fakePayload.append((temperature & 0xFF << 8) >> 8);
            fakePayload.append((temperature & 0xFF << 16) >> 16);
            fakePayload.append((temperature & 0xFF << 24) >> 24);
        }

        frame_p->setPayload(fakePayload);
        frame_p->setBus(0);
        frame_p->setExtendedFrameFormat(0);
        frame_p->setFrameId(200);
        frame_p->setFrameType(CommFrame::CANDataFrame);
        frame_p->setReceived(true);
        frame_p->setTimeStamp(CommFrame::TimeStamp::fromMicroSeconds(QDateTime::currentMSecsSinceEpoch() * 1000ul));

        checkTargettedFrame(*frame_p);

        /* enqueue frame */
        getQueue().queue();
    }
}

void MyFakeCan::sendDebug(const QString debugText)
{
    qDebug() << debugText;
    debugOutput(debugText);
}

void MyFakeCan::startTimer()
{
    if (!m_pFakeBodyMessageTimer) {
        m_pFakeBodyMessageTimer = new QTimer(this);
        m_pFakeBodyMessageTimer->setInterval(1000 / 33); // 33 hz
        m_pFakeBodyMessageTimer->setSingleShot(false); //keep ticking
        connect(m_pFakeBodyMessageTimer, &QTimer::timeout, this, &MyFakeCan::generateFakeBodyFrame);
    }

    if (!m_pFakeHeadMessageTimer) {
        m_pFakeHeadMessageTimer = new QTimer(this);
        m_pFakeHeadMessageTimer->setInterval(1000 / 100); // 100 hz
        m_pFakeHeadMessageTimer->setSingleShot(false); //keep ticking
        connect(m_pFakeHeadMessageTimer, &QTimer::timeout, this, &MyFakeCan::generateFakeHeadFrame);
    }

    if (!m_pElapsedTimer) {
        m_pElapsedTimer = new QElapsedTimer();
    }

    if (m_pFakeBodyMessageTimer) m_pFakeBodyMessageTimer->start();
    if (m_pFakeHeadMessageTimer) m_pFakeHeadMessageTimer->start();
    if (m_pElapsedTimer) {
        m_pElapsedTimer->start();
    }
}

void MyFakeCan::stopTimer()
{
    if (m_pFakeBodyMessageTimer) m_pFakeBodyMessageTimer->stop();
    if (m_pFakeHeadMessageTimer) m_pFakeHeadMessageTimer->stop();
}

double MyFakeCan::getElapsedSecond() const
{
    if (!m_pElapsedTimer) return 0;
    return m_pElapsedTimer->nsecsElapsed() * 10e-9;
}
