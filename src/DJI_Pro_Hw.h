#ifndef DJI_PRO_HW_H
#define DJI_PRO_HW_H

#include <QMainWindow>

#include <QString>
#include <QObject>
#include <QThread>
#include "qextserialport.h"

#define DJI_PRO_HW_BUFFER_SIZE      1024


class DJI_Pro_Hw : public QThread
{
    Q_OBJECT
public:
    explicit DJI_Pro_Hw(QObject *parent = 0);
    explicit DJI_Pro_Hw(QString name,int baudrate);
    ~DJI_Pro_Hw();

signals:

public slots:
private slots:
    void Pro_Hw_Recv();
private:
    QextSerialPort *port;
    unsigned char *pbuffer;
    int findindex(int);
public:
    void Pro_Hw_Close();
    void Pro_Hw_Flush();
    int Pro_Hw_Send(unsigned char *buf, int len);
    bool Pro_Hw_Setup(QString port_name,int baudrate);
    int load_con;
protected:
    void run();
};







extern DJI_Pro_Hw Pro_Hw;

#endif // DJI_PRO_HW_H
