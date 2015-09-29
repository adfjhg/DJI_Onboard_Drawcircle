#include "controldialog.h"
#include "ui_controldialog.h"
#include "DJI_Pro_App.h"
#include "DJI_uav_control.h"

ControlDialog::ControlDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ControlDialog)
{
    ui->setupUi(this);
    UavCtrl = new DJI_UAV_control();
}

ControlDialog::~ControlDialog()
{
    delete ui;
    delete UavCtrl;
}

void ControlDialog::on_btn_submit_clicked()
{
    control_data_t data;
    data.hori.roll_or_x = ui->LE_X->text().toFloat();
    data.hori.pitch_or_y = ui->LE_Y->text().toFloat();
    data.vert.data = ui->LE_Z->text().toFloat();
    data.yaw.data = ui->LE_YAW->text().toFloat();
    data.hori.mode = (hori_mode_e)((*(ui->LE_HoriMode->text().toLocal8Bit().data())-'0')<<6);
    data.vert.mode = (vert_mode_e)((*(ui->LE_VertMode->text().toLocal8Bit().data())-'0')<<4);
    data.yaw.mode = (yaw_mode_e)((*(ui->LE_YawMode->text().toLocal8Bit().data())-'0')<<3);
    data.hori_cor = (Coordinate_system_hori_e)((*(ui->LE_HoriCol->text().toLocal8Bit().data())-'0')<<1);
    data.yaw_cor = (Coordinate_system_yaw_e)(*(ui->LE_YawCol->text().toLocal8Bit().data())-'0');
    UavCtrl->SaveHomePos();
    UavCtrl->Control(data);


}

void ControlDialog::on_btn_submit_2_clicked()
{
    this->close();
}

void ControlDialog::on_btn_start_clicked()
{
   UavCtrl->StartOrbit();
}
