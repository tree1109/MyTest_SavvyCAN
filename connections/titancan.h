#ifndef TITANCAN_H
#define TITANCAN_H

#include <QObject>
#include "canconnection.h"

#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QElapsedTimer>

class TitanCAN : public CANConnection
{
    Q_OBJECT

public:
    TitanCAN(QString comPort);
    virtual ~TitanCAN();

protected:

    virtual void piStarted();
    virtual void piStop();
    virtual void piSetBusSettings(int pBusIdx, CANBus pBus);
    virtual bool piGetBusSettings(int pBusIdx, CANBus& pBus);
    virtual void piSuspend(bool pSuspend);
    virtual bool piSendFrame(const CommFrame& pFrame) ;

private:
    void connectDevice();
    void disconnectDevice();

    void readFrame();

    void sendDebug(const QString debugText);

private:
    std::optional<int32_t> m_CanHandle = std::nullopt;
};

#endif // TITANCAN_H
