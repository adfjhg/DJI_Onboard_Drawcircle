#include "DJI_uav_control.h"
#include "DJI_Pro_App.h"
#include "DJI_Pro_Test.h"
#include "NextTargetFinder.h"
#include <QTimer>
#include <math.h>




DJI_UAV_control::DJI_UAV_control(QObject *parent) : QThread(parent)
{
    CtrlHeartbeat = new QTimer(this);
    OrbitUpdate = new QTimer(this);
    connect(CtrlHeartbeat, SIGNAL(timeout()), this, SLOT(Heartbeat()));
    connect(OrbitUpdate, SIGNAL(timeout()), this, SLOT(OrbitUpdateHandle()));
    t = 0;
}

DJI_UAV_control::~DJI_UAV_control()
{

}

void DJI_UAV_control::Heartbeat()
{
     App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&send_data, sizeof(send_data), NULL, 0, 0);
}

void DJI_UAV_control::OrbitUpdateHandle()
{
    OrbitCtrl();
}

void DJI_UAV_control::GetUavInfo(sdk_std_msg_t *uavInfo)
{
    DJI_GetUavInfo(uavInfo);
}

void DJI_UAV_control::SetCircleCenter(sdk_std_msg_t UavInfo)
{
    circle.longti = UavInfo.pos.longti;
    circle.lati = UavInfo.pos.lati;
    circle.alti = UavInfo.pos.alti;
}

void DJI_UAV_control::SaveHomePos()
{
    sdk_std_msg_t UavInfo;
    GetUavInfo(&UavInfo);
    home_longti = UavInfo.pos.longti;
    home_lati = UavInfo.pos.lati;
    home_alti = UavInfo.pos.alti;
    home_height = UavInfo.pos.height;
    printf("home_longti = %f\nhome_lati = %lf\nhome_alti = %f\nhome_heigth = %f\n",\
           home_longti,home_lati,home_alti,home_height);
}


void DJI_UAV_control::CalPos(const fp64 Cur_lati_r, const fp64 Cur_longti_r, const fp32 Cur_alti_m, \
            const fp64 lati_offset_m, const fp64 longti_offset_m, const fp32 alti_offset_m, \
            fp64* const Tar_lati_r, fp64* const Tar_longti_r, fp32* const Tar_alti_m)
{
    *Tar_lati_r = Cur_lati_r + (lati_offset_m/C_EARTH);
    *Tar_longti_r = Cur_longti_r + (longti_offset_m/(C_EARTH*cos(Cur_lati_r)));
    *Tar_alti_m = Cur_alti_m + alti_offset_m;
}

void DJI_UAV_control::CalPos(const fp64 Cur_lati_r, const fp64 Cur_longti_r, const fp32 Cur_alti_m, \
            fp64* const lati_offset_m, fp64* const longti_offset_m, fp32* const alti_offset_m,\
            const fp64 Tar_lati_r, const fp64 Tar_longti_r, const fp32 Tar_alti_m)
{
    *lati_offset_m = (Tar_lati_r - Cur_lati_r)*C_EARTH;
    *longti_offset_m = (Tar_longti_r - Cur_longti_r)*(C_EARTH*cos(Cur_lati_r));
    *alti_offset_m = Tar_alti_m - Cur_alti_m;
}

void DJI_UAV_control::StartOrbit()
{
    sdk_std_msg_t UavInfo;
    GetUavInfo(&UavInfo);   //feedback
    SetCircleCenter(UavInfo); //保存当前点为圆心

    CurCircle_r = CIRCLE_R;

    SaveHomePos();           //保存home点
    fp64 lati_offset_m = CIRCLE_R;
    fp64 longti_offset_m = 0;
    fp32 alti_offset_m = 0;
    fp64 tar_lati,tar_longti;
    fp32 tar_alti;

    //测试calpos函数
/*    CalPos(UavInfo.pos.lati,UavInfo.pos.longti,UavInfo.pos.alti,\
           lati_offset_m,longti_offset_m,alti_offset_m,\
           &tar_lati,&tar_longti,&tar_alti);

    UavInfo.pos.lati = Radian2Angle(UavInfo.pos.lati);
    UavInfo.pos.longti = Radian2Angle(UavInfo.pos.longti);
    printf("longti = %.15f lati = %.15f alti = %f\n",UavInfo.pos.longti,UavInfo.pos.lati,UavInfo.pos.alti);


    //GotoPos(tar_lati,tar_lati,tar_alti);

    tar_lati = Radian2Angle(tar_lati);
    tar_longti = Radian2Angle(tar_longti);
    printf("tar_longti = %.15f tar_lati = %.15f\n",tar_longti,tar_lati);

    lati_offset_m = 0;
    longti_offset_m = 0;
    alti_offset_m = 0;

    CalPos(Angle2Radian(UavInfo.pos.lati),Angle2Radian(UavInfo.pos.longti),UavInfo.pos.alti,\
           &lati_offset_m,&longti_offset_m,&alti_offset_m,\
           Angle2Radian(tar_lati), Angle2Radian(tar_longti),tar_alti);

    printf("longti_offset_m = %.15f lati_offset_m = %.15f alti_offset_m = %f\n",longti_offset_m,lati_offset_m,alti_offset_m);
*/
    CalPos(UavInfo.pos.lati,UavInfo.pos.longti,UavInfo.pos.alti,\
               lati_offset_m,longti_offset_m,alti_offset_m,\
               &tar_lati,&tar_longti,&tar_alti);
   /* printf("longti = %.15f lati = %.15f alti = %f\n",\
           Radian2Angle(UavInfo.pos.longti),Radian2Angle(UavInfo.pos.lati),UavInfo.pos.alti);
    printf("tar_longti = %.15f tar_lati = %.15f tar_alti = %f\n",\
           Radian2Angle(tar_longti),Radian2Angle(tar_lati),tar_alti);
    */

    GotoPos(tar_lati,tar_longti,tar_alti,0.01);

    GetUavInfo(&UavInfo);
    CurPos_x = UavInfo.pos.longti;
    CurPos_y = UavInfo.pos.lati;
    CurPos_z = UavInfo.pos.alti;

    OrbitUpdate->start(1000/FREQUENCY);             //开始控制
}

void DJI_UAV_control::OrbitCtrl()
{
    sdk_std_msg_t UavInfo;
    control_data_t data;
    fp64 x_m,y_m,circle_l,circle_angle;
    fp32 z_m;
    fp32 delta_z,delta_r;
    fp32 percent_z;
    fp32 vector_x,vector_y,vector_z,norm;

    GetUavInfo(&UavInfo);   //feedback

    //获取当前X,Y,Z坐标，单位m
    CalPos(circle.lati, circle.longti, circle.alti,\
           &x_m, &y_m, &z_m,\
           UavInfo.pos.lati, UavInfo.pos.longti, UavInfo.pos.alti);



    NextTargetFinder finder(Point2d(0, 0), 5, NextTargetFinder::CLOCKWISE, CIRCLE_V, FREQUENCY);
    Point2d ptTarget = finder.FindNextTarget(Point2d(x_m, y_m));

    CurPos_x = ptTarget.first;
    CurPos_y = ptTarget.second;

    printf("x= %.4f y = %.4f z = %.4f xl = %.4f yl = %.4f zl = %.4f\r",\
           CurPos_x,CurPos_y,CurCircle_r,\
           x_m,y_m,z_m);

    vector_x = CurPos_x-x_m;
    vector_y = CurPos_y-y_m;
    //vector_z = CurPos_z-z_m;



   norm = sqrt(vector_x*vector_x + \
                vector_y*vector_y);

    vector_x /= norm;
    vector_y /= norm;
    //vector_z /= norm;

    data.hori.roll_or_x = vector_x*CIRCLE_V;
    data.hori.pitch_or_y = vector_y*CIRCLE_V;
    //data.vert.data = vector_z*CIRCLE_V;
    data.vert.data = 0.1;
    data.hori.mode = h_vel;
    data.vert.mode = v_vel;
    data.yaw.data = 0;
    data.yaw.mode = y_rate;
    data.hori_cor = hori_ground;
    data.yaw_cor = yaw_body;
    Control(data);









   /* printf("longti=%f lati=%f alti=%f heigth=%f vx=%f vy=%f vz=%f\r\b",\
           UavInfo.pos.longti*180/C_PI, UavInfo.pos.lati*180/C_PI,UavInfo.pos.alti,UavInfo.pos.height,\
           UavInfo.v.x,UavInfo.v.y,UavInfo.v.z);
    */
}

void DJI_UAV_control::GotoPos(fp64 lati, fp64 longti, fp32 alti,fp32 err)
{
    sdk_std_msg_t UavInfo;
    control_data_t data;
    fp64 lati_offset = 0;
    fp64 longti_offset = 0;
    fp32 alti_offset = 0;
    while(1)
    {
        GetUavInfo(&UavInfo);

        CalPos(UavInfo.pos.lati,UavInfo.pos.longti,UavInfo.pos.alti,\
               &lati_offset,&longti_offset,&alti_offset,\
               lati,longti,alti);


        printf("lati = %f longti = %f alti = %f sqrt = %f\r",lati_offset,longti_offset,alti_offset,\
               sqrt(lati_offset*lati_offset + longti_offset*longti_offset));

        if(sqrt(lati_offset*lati_offset + longti_offset*longti_offset) <= err)
        {
            data.hori.roll_or_x = 0;
            data.hori.pitch_or_y = 0;
            data.vert.data = 0;
            data.hori.mode = h_pos;
            data.vert.mode = v_vel;
            data.yaw.data = 0;
            data.yaw.mode = y_rate;
            data.hori_cor = hori_ground;
            data.yaw_cor = yaw_body;
            Control(data);
            break;
        }

        data.hori.roll_or_x = lati_offset;
        data.hori.pitch_or_y = longti_offset;
        data.vert.data = alti;
        data.hori.mode = h_pos;
        data.vert.mode = v_pos;
        data.yaw.data = 0;
        data.yaw.mode = y_rate;
        data.hori_cor = hori_ground;
        data.yaw_cor = yaw_body;
        Control(data);
        msleep(20);
    }

}


void DJI_UAV_control::Control(control_data_t data)
{
    send_data.ctrl_flag = GetCtrlMode(data);
    send_data.roll_or_x =  data.hori.roll_or_x;
    send_data.pitch_or_y = data.hori.pitch_or_y;
    send_data.thr_z = data.vert.data; //m/s
    send_data.yaw = data.yaw.data;
    if(!CtrlHeartbeat->isActive())
    {
        CtrlHeartbeat->start(20);
    }
    App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&send_data, sizeof(send_data), NULL, 0, 0);
}

uint8_t  DJI_UAV_control::GetCtrlMode(control_data_t data)
{
    return (data.hori.mode | data.vert.mode | data.yaw.mode | data.hori_cor | data.yaw_cor);
}
