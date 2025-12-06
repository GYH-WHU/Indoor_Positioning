#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>

using namespace std;
using namespace Eigen;

/*-----------------------------------------------
    常数定义
------------------------------------------------*/
/* 常数 */
#define PAI 3.141592653589      // 圆周率
#define Deg 180.0/PAI           // 弧度转化为度
#define Rad PAI/180.0           // 度转化为弧度
#define YAW_START -90*Rad       // 初始航向角
#define MAG_DEC 14.11           // 当地磁偏角
#define B_Start 51.08098100     // 初始纬度
#define L_Start -114.12872700   // 初始经度
#define STEP 0.7/3.0            // 步长
/* WGS84坐标系常数 */
#define R_WGS84     6378137.0                                   // 地球半长轴 [米]
#define F_WGS84     (1.0 / 298.257223563)                       // 地球扁率

/*-----------------------------------------------
    结构体定义
------------------------------------------------*/
/* 加速度计数据 */
struct ACCDAT
{
    double X, Y, Z;
    ACCDAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/* 陀螺仪数据 */
struct GYRODAT
{
    double X, Y, Z;
    GYRODAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/* 磁强计数据 */
struct MAGDAT
{
    double X, Y, Z;
    MAGDAT() { X = Y = Z = 0.0; }
};

/* 原始数据 */
/* GPS时间 */
struct GPSTIME
{
    double Second;  // 秒
    GPSTIME() { Second = 0; }
};

struct RAWDAT
{
    GPSTIME time;   // 周 秒
    ACCDAT acc;     // 加速度计数据
    GYRODAT gyr;    // 陀螺仪数据（弧度）
    MAGDAT mag;     // 磁强计
};

/* 位置结果 */
struct POSITION
{
    double latitude;    // 纬度
    double longitude;   // 经度
    double H;           // 高程
    POSITION() { latitude = longitude = H = 0.0; }
};

/* 粗对准结构体 */
struct ALIGNPOS
{
    GPSTIME time;   // 记录时间
    GPSTIME prevtime;   // 上一个历元的时间
    double billimean[3];   // 比力平均值
    double anglevelmean[3]; // 角速度平均值
    double magmean[3];
    double yaw;     // 航向角
    double pitch;   // 俯仰角
    double roll;    // 横滚角

    ALIGNPOS()
    {
        yaw = 0.0; pitch = 0.0; roll = 0.0;
        for (int i = 0; i < 3; i++)
        {
            billimean[i] = 0.0; anglevelmean[i] = 0.0; magmean[i] = 0.0;
        }
    }
};

/* 笛卡尔坐标 (X, Y, Z) */
union XYZ
{
    struct {
        double x;
        double y;
        double z;
    };
    double xyz[3];
};

/* 大地坐标 (经度, 纬度, 高程) */
union GEOCOOR
{
    struct {
        double time;
        double latitude;    // 纬度
        double longitude;   // 经度
        double height;      // 高程
    };
    double Blh[3];
};

/* 测站地平坐标 (北, 东, 天顶) */
union NEU
{
    struct {
        double dE;
        double dN;
        double dU;
    };
    double Neu[3];  // ENU
};

/*-----------------------------------------------
    文件操作函数
------------------------------------------------*/
bool ReadRawData(ifstream& file, RAWDAT& Rawdata);   // 读取一行原始数据
bool ReadRefData(ifstream& file, GEOCOOR& Rawdata);  // 读取一行参考数据

/*-----------------------------------------------
    姿态计算函数
------------------------------------------------*/
void Align_EverySecond(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile);   // 每秒粗对准
void Align_EveryEpoch(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile);    // 每历元对准
void CalYawWithMag(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile);       // 每秒计算航向角（地磁）

/*-----------------------------------------------
    航位推算计算函数
------------------------------------------------*/
void CalReference(ifstream& File, GEOCOOR& Rawdata, ofstream& OutputFile);  // 计算参考结果
void GyrDeadReckoning(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile);    // 陀螺推算
void MagDeadReckoning(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile);    // 磁强计推算

/*-----------------------------------------------
    坐标变换函数
------------------------------------------------*/
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F);  // BLH2XYZ
void XYZToBLH(const double XYZ[3], double BLH[3], const double R, const double F);  // XYZ2BLH
void BlhToNeuMat(const GEOCOOR* Blh, Matrix3d& Mat);
void CompEnudPos(const double Xs[], const double Xr[], const GEOCOOR* Blh, double dENU[]);  // 计算enu