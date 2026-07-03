#include "titancan.h"

#include "Titan_USB_CAN/CAN_API.h"

namespace {
    uint32_t GetTitanCANBitrate(uint32_t bitrate) {
        switch(bitrate)
        {
        case 10000: return 10;
        case 20000: return 20;
        case 50000: return 50;
        case 100000: return 100;
        case 125000: return 125;
        case 250000: return 250;
        case 500000: return 500;
        case 800000: return 800;
        case 1000000: return 1000;
        default: return 500;
        }
    }
}

TitanCAN::TitanCAN(QString comPort)
    : CANConnection(comPort, "Titan", CANCon::TITAN_CAN, 0, 0, false, 1000, 1, 4000, true)

{
    sendDebug("TitanCAN()");

    CANBus bus_info;
    bus_info.setActive(true);
    bus_info.setListenOnly(true);
    bus_info.setSpeed(1000);
    setBusConfig(0, bus_info);
}

TitanCAN::~TitanCAN()
{
    stop();
    sendDebug("~TitanCAN()");

    delete m_ElapsedTimer;
    delete m_ReadFrameTimer;
    m_ElapsedTimer = nullptr;
    m_ReadFrameTimer = nullptr;
}

void TitanCAN::piStarted()
{
    connectDevice();
}

void TitanCAN::piStop()
{
    disconnectDevice();
}

void TitanCAN::piSetBusSettings(int pBusIdx, CANBus pBus)
{
    /* sanity checks */
    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    /* copy bus config */
    setBusConfig(pBusIdx, pBus);

    connectDevice();
}

bool TitanCAN::piGetBusSettings(int pBusIdx, CANBus &pBus)
{
    return getBusConfig(pBusIdx, pBus);
}

void TitanCAN::piSuspend(bool pSuspend)
{
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}

bool TitanCAN::piSendFrame(const CommFrame &pFrame)
{
    if (!isOpen()) {
        qDebug() << "TitanCAN: " << "CAN not opend.";
        return false;
    }

    // Convert format.
    CAN_MSG message{};
    {
        message.Id = pFrame.frameId();

        {
            const auto& rPayload = pFrame.payload();
            const int maxSize = qMin(rPayload.size(), 8);
            for (int i = 0; i < maxSize; ++i){
                message.Data[i] = rPayload[i];
            }
        }

        message.Size = pFrame.payload().size();

        message.Flags = CAN_FLAGS_STANDARD;

        message.Timestamp = getElapsedTimeS() * 1000 ;
    }

    const TCAN_STATUS status = CAN_Write(m_CanHandle, &message );
    if (status != CAN_ERR_OK) {
        qDebug() << "TitanCAN: " << "Failed to send frame.";
        return false;
    }

    return true;
}

void TitanCAN::checkFrame()
{
    if (isOpen()) {
        readFrame();
    }
    else {
        setStatus(CANCon::NOT_CONNECTED);
    }
}

void TitanCAN::connectDevice()
{
    qDebug() << "TitanCAN: " << "Start connect.";

    if (isOpen()) {
        qDebug() << "TitanCAN: " << "Already connected, disconnect first.";
        disconnectDevice();
    }

    // Com port.
    const auto portStr = QString("COM%1").arg(getPort()).toStdString();
    char* const pPortStr = const_cast<char*>(portStr.c_str());

    // Bitrate.
    CANBus busConfig;
    getBusConfig(0, busConfig);
    const auto bitrateStr = QString("%1").arg(GetTitanCANBitrate(busConfig.getDataRate())).toStdString();
    char* const pBitrateStr = const_cast<char*>(bitrateStr.c_str());

    char* const ACC_CODE = "1FFFFFFF";
    char* const ACCEPTANCE_MASK = "00000000";
    constexpr auto DEFAULT_CAN_MODE = LoopBack;

    const TCAN_HANDLE canHandle = CAN_Open(pPortStr, pBitrateStr, ACC_CODE, ACCEPTANCE_MASK, CAN_TIMESTAMP_ON, DEFAULT_CAN_MODE);
    if (canHandle > 0) {
        m_CanHandle = canHandle;
        setStatus(CANCon::CONNECTED);
        startReadFrameTimer();

        qDebug() << "TitanCAN: " << "Success to connect.";

        if (CAN_Flush(canHandle) & CAN_ERR_OK) {
            qDebug() << "TitanCAN: " << "Flush.";
        }

        if (char version[BUFSIZ]{}; CAN_Version(canHandle, version) & CAN_ERR_OK){
            qDebug() << "TitanCAN: " << "Version ["<< version <<"].";
        }
    }
    else {
        qDebug() << "TitanCAN: " << "Failed to connect.";
    }
}

void TitanCAN::disconnectDevice()
{
    qDebug() << "TitanCAN: " << "Start connect.";

    if (!isOpen()) {
        return;
    }

    const TCAN_STATUS status = CAN_Close(m_CanHandle);
    if (status == CAN_ERR_OK) {
        qDebug() << "TitanCAN: " << "Failed to disconnect.";
        m_CanHandle = 0;
        setStatus(CANCon::NOT_CONNECTED);
        stopReadFrameTimer();
    }
    else {
        qDebug() << "TitanCAN: " << "Failed to disconnect.";
    }
}

void TitanCAN::readFrame()
{
    if (isCapSuspended()) {
        return;
    }

    if (!isOpen()) {
        return;
    }

    CAN_MSG message{};

    for (;;) {
        const TCAN_STATUS status = CAN_Read ( m_CanHandle, &message);
        if ( status == CAN_ERR_OK ) {
            CommFrame* frame_p = getQueue().get();
            if(frame_p) {
                // Data.
                {
                    const uint8_t dataSize = message.Size;
                    const uint8_t* const pData = message.Data;

                    QByteArray payload;
                    payload.reserve(dataSize);

                    for (uint32_t i = 0; i < dataSize; ++i) {
                        payload.push_back(pData[i]);
                    }

                    frame_p->setPayload(payload);
                }

                frame_p->setBus(0);

                // ID type.
                {
                    const bool bIsStd = message.Flags & CAN_FLAGS_STANDARD;
                    const bool bIsExt = message.Flags & CAN_FLAGS_EXTENDED;
                    assert(bIsStd != bIsExt); // Consistence check.
                    frame_p->setExtendedFrameFormat(bIsExt);
                }

                frame_p->setFrameId(message.Id);

                // Frame type.
                {
                    const bool bIsRemoteFrame = message.Flags & CAN_FLAGS_REMOTE;
                    frame_p->setFrameType(bIsRemoteFrame ? CommFrame::RemoteRequestFrame : CommFrame::CANDataFrame);
                }

                frame_p->setReceived(true);

                // Timestamp.
                {
                    const auto convertTimestamp = CommFrame::TimeStamp::fromMicroSeconds(message.Timestamp * 1000);
                    frame_p->setTimeStamp(convertTimestamp);
                }

                checkTargettedFrame(*frame_p);

                /* enqueue frame */
                getQueue().queue();
            }

        }
        else {
            break;
        }
    }
}

void TitanCAN::sendDebug(const QString debugText)
{
    qDebug() << debugText;
    debugOutput(debugText);
}

bool TitanCAN::isOpen() const
{
    return m_CanHandle > 0;
}

void TitanCAN::startReadFrameTimer()
{
    if (m_ReadFrameTimer == nullptr) {
        m_ReadFrameTimer = new QTimer(this);
        m_ReadFrameTimer->setInterval(1000 / 100);
        m_ReadFrameTimer->setSingleShot(false);
        m_ReadFrameTimer->start();

        connect(m_ReadFrameTimer, &QTimer::timeout, this, &TitanCAN::checkFrame);
    }
    else {
        m_ReadFrameTimer->start();
    }

    if (m_ElapsedTimer == nullptr) {
        m_ElapsedTimer = new QElapsedTimer();
        m_ElapsedTimer->start();
    }
    else {
        m_ElapsedTimer->start();
    }
}

void TitanCAN::stopReadFrameTimer()
{
    if (m_ReadFrameTimer) {
        m_ReadFrameTimer->stop();
    }
}

double TitanCAN::getElapsedTimeS() const
{
    if (m_ElapsedTimer) {
        return m_ElapsedTimer->nsecsElapsed() * 10e-9;
    }
    return 0;
}
