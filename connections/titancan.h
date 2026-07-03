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

private slots:
    void checkFrame();

private:
    void connectDevice();
    void disconnectDevice();

    void readFrame();

    void sendDebug(const QString debugText);

    bool isOpen() const;

    void startReadFrameTimer();
    void stopReadFrameTimer();

    double getElapsedTimeS() const;

private:
    int32_t m_CanHandle = 0;
    QTimer* m_ReadFrameTimer = nullptr;
    QElapsedTimer* m_ElapsedTimer = nullptr;
};

#endif // TITANCAN_H
