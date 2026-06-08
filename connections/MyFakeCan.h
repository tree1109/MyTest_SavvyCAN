#ifndef MYFAKECAN_H
#define MYFAKECAN_H

#include "canconnection.h"

#include <QThread>
#include <QTimer>
#include <QDateTime>

class MyFakeCan : public CANConnection
{
    Q_OBJECT

public:
    MyFakeCan(QString port);
    virtual ~MyFakeCan();

protected:

    virtual void piStarted();
    virtual void piStop();
    virtual void piSetBusSettings(int pBusIdx, CANBus pBus);
    virtual bool piGetBusSettings(int pBusIdx, CANBus& pBus);
    virtual void piSuspend(bool pSuspend);
    virtual bool piSendFrame(const CommFrame& pFrame) ;

    void disconnectDevice();

private slots:
    void generateFakeFrame();

private:
    void sendDebug(const QString debugText);

    QTimer* createTimer();

private:
    int m_Port = 114000;

    QTimer* m_pTimer = nullptr;
};

#endif // MQTT_BUS_H
