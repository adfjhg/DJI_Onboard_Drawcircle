#ifndef DJI_UAV_CONTROL_H
#define DJI_UAV_CONTROL_H

#include "DJI_Pro_App.h"
#include <QTimer>
#include <QThread>
#include <math.h>

#define VERT_VEL                (0<<4)
#define VERT_POS                (1<<4)
#define VERT_THRUST             (2<<4)

#define HORI_ATTI_TILT_ANG      (0<<6)
#define HORI_VEL                (1<<6)
#define HORI_POS                (2<<6)

#define YAW_ANG                 (0<<3)
#define YAW_RATE                (1<<3)

#define HORI_GROUND             (0<<1)
#define HORI_BODY               (1<<1)

#define YAW_GROUND              (0)
#define YAW_BODY                (1)

#define CIRCLE_R                (5)        //起始圆的半径
#define CIRCLE_R_DEC            (-0.5)      //每圈半径减少的值
#define CIRCLE_Z_INC            (0.5)      //每圈上升高度
#define CIRCLE_V                (3.0)        //运行速度，m/s

#define FREQUENCY               (100)


#define C_EARTH     (double)6378137.0
#define C_PI        (double)3.141592653589793

#define C_DEG2RAD   (double)0.01745329251994330

#define C_RAD2DEG   (double)57.29577951308232

#define Angle2Radian(x)    ((x)*C_DEG2RAD)
#define Radian2Angle(x)    ((x)*C_RAD2DEG)

typedef enum
{
    v_pos = VERT_POS,
    v_vel = VERT_VEL,
    v_thrust = VERT_THRUST
}vert_mode_e;

typedef enum
{
    h_atti_tile_ang = HORI_ATTI_TILT_ANG,
    h_pos = HORI_POS,
    h_vel = HORI_VEL
}hori_mode_e;

typedef enum
{
    y_ang = YAW_ANG,
    y_rate = YAW_RATE
}yaw_mode_e;

typedef enum
{
    yaw_body = YAW_BODY,
    yaw_ground = YAW_GROUND
}Coordinate_system_yaw_e;

typedef enum
{
    hori_body = HORI_BODY,
    hori_ground = HORI_GROUND
}Coordinate_system_hori_e;

typedef struct
{
    struct
    {
        fp32 data;
        vert_mode_e mode;
    }vert;

    struct
    {
        fp32 roll_or_x;
        fp32 pitch_or_y;
        hori_mode_e mode;
    }hori;

    struct
    {
        fp32 data;
        yaw_mode_e mode;
    }yaw;

    Coordinate_system_yaw_e yaw_cor;
    Coordinate_system_hori_e hori_cor;

}control_data_t;

typedef struct
{
    fp64	lati;
    fp64	longti;
    fp32	alti;
    fp32    r;
}circle_t;

class DJI_UAV_control: public QThread
{
   Q_OBJECT
public:
    explicit DJI_UAV_control(QObject *parent = 0);
    ~DJI_UAV_control();
    void Control(control_data_t data);
    void GetUavInfo(sdk_std_msg_t *uavInfo);
    void SaveHomePos();
    void StartOrbit();

private:
    uint8_t GetCtrlMode(control_data_t data);
    void GotoPos(fp64 lati, fp64 longti, fp32 alti,fp32 err);
    api_ctrl_without_sensor_data_t send_data;
    QTimer *CtrlHeartbeat,*OrbitUpdate;
    void OrbitCtrl();
    void SetCircleCenter(sdk_std_msg_t UavInfo);
    void CalPos(const fp64 Cur_lati_r, const fp64 Cur_longti_r, const fp32 Cur_alti_m, \
                const fp64 lati_offset_m, const fp64 longti_offset_m, const fp32 alti_offset_m, \
                fp64* const Tar_lati_r, fp64* const Tar_longti_r, fp32* const Tar_alti_m);

    void CalPos(const fp64 Cur_lati_r, const fp64 Cur_longti_r, const fp32 Cur_alti_m, \
                fp64* const lati_offset_m, fp64* const longti_offset_m, fp32* const alti_offset_m,\
                const fp64 Tar_lati_r, const fp64 Tar_longti_r, const fp32 Tar_alti_m);

    fp64	home_lati;
    fp64	home_longti;
    fp32	home_alti;
    fp32	home_height;
    circle_t circle;
    fp64    CurPos_x,CurPos_y,CurPos_z;
    fp32    CurCircle_r,CurCircle_z;
    int     t;

private slots:
     void Heartbeat();
     void OrbitUpdateHandle();
};

#endif // DJI_UAV_CONTROL_H
