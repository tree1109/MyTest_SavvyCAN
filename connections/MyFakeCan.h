#ifndef MYFAKECAN_H
#define MYFAKECAN_H

#include "canconnection.h"

#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QElapsedTimer>

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
    void generateFakeBodyFrame();
    void generateFakeHeadFrame();

private:
    void sendDebug(const QString debugText);

    void initTimer();
    void startTimer();
    void stopTimer();
    double getElapsedSecond() const;

private:
    int m_Port = 114000;

    QElapsedTimer* m_pElapsedTimer = nullptr;

    QTimer* m_pFakeBodyMessageTimer = nullptr;
    QTimer* m_pFakeHeadMessageTimer = nullptr;
};

#endif // MQTT_BUS_H
