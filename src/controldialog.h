#ifndef CONTROLDIALOG_H
#define CONTROLDIALOG_H

#include <QDialog>
#include <DJI_uav_control.h>

namespace Ui {
class ControlDialog;
}

class ControlDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ControlDialog(QWidget *parent = 0);
    ~ControlDialog();

private slots:
    void on_btn_submit_clicked();

    void on_btn_submit_2_clicked();

    void on_btn_start_clicked();

private:
    Ui::ControlDialog *ui;

private:
    DJI_UAV_control *UavCtrl;
};

#endif // CONTROLDIALOG_H
