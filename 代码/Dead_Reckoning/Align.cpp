/******* 对准 ********/
#include"DR_Structs.h"
using namespace std;

/* 每个历元粗对准 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输出：姿态 例：ALIGNPOS pos.yaw
// 输出：姿态文件
void Align_EveryEpoch(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;
	double dt = 0.0;
	Vector3d cbn = Vector3d::Zero();
	Vector3d wibb = Vector3d::Zero();

	// 逐行读取文件
	while (ReadRawData(File, Rawdata))
	{
		// 提取时间
		Pos.time = Rawdata.time;

		Pos.billimean[0] = Rawdata.acc.X;
		Pos.billimean[1] = Rawdata.acc.Y;
		Pos.billimean[2] = Rawdata.acc.Z;

		Pos.anglevelmean[0] = Rawdata.gyr.X * Rad;	// 弧度
		Pos.anglevelmean[1] = Rawdata.gyr.Y * Rad;	// 弧度
		Pos.anglevelmean[2] = Rawdata.gyr.Z * Rad;	// 弧度
		epochnum++;

		// 计算姿态角
		Pos.roll = atan2(-Pos.billimean[1], -Pos.billimean[2]);
		Pos.pitch = atan2(Pos.billimean[0], sqrt(pow(Pos.billimean[1], 2) + pow(Pos.billimean[2], 2)));

		dt = Pos.time.Second - Pos.prevtime.Second;
		cbn[0] = -sin(Pos.pitch);
		cbn[1] = sin(Pos.roll) * cos(Pos.pitch);
		cbn[2] = cos(Pos.roll) * cos(Pos.pitch);
		
		wibb[0] = Pos.anglevelmean[0];
		wibb[1] = Pos.anglevelmean[1];
		wibb[2] = Pos.anglevelmean[2];

		double wd = cbn.transpose() * wibb;

		if (epochnum == 0)Pos.yaw = wd * dt + YAW_START;
		else Pos.yaw = wd * dt + Pos.yaw;

		if (Pos.yaw >= PAI)Pos.yaw -= 2 * PAI;
		if (Pos.yaw <= -PAI)Pos.yaw += 2 * PAI;

		double accmol = sqrt(pow(Pos.billimean[0], 2) + pow(Pos.billimean[1], 2) + pow(Pos.billimean[2], 2));

		OutputFile << fixed << setprecision(12);
		OutputFile << Pos.time.Second << ",";
		OutputFile << Pos.yaw * Deg << "," << Pos.pitch * Deg << "," << Pos.roll * Deg << "," << accmol << endl;

		Pos.prevtime = Pos.time;
	}
	cout << "\n每历元的姿态角求解成功，结果已保存在文件中！\n";
}

/* 每秒粗对准 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输出：姿态 例：ALIGNPOS pos.yaw
// 输出：姿态文件
void Align_EverySecond(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0, epochnum2 = 0.0, T = 0.0;
	double totaltime = 0.0;
	double dt = 0.0;
	Vector3d cbn = Vector3d::Zero();
	Vector3d wibb = Vector3d::Zero();

	// 逐行读取文件
	while (ReadRawData(File, Rawdata))
	{
		// 提取时间
		Pos.time = Rawdata.time;
		if (epochnum != 0)dt = Pos.time.Second - Pos.prevtime.Second;
		totaltime += dt;
		// 当读取时间小于1秒时
		if (fabs(totaltime - 1.0) > 1e-3)
		{
			accmean[0] += Rawdata.acc.X;
			accmean[1] += Rawdata.acc.Y;
			accmean[2] += Rawdata.acc.Z;

			avelmean[0] += Rawdata.gyr.X * Rad;	// 弧度
			avelmean[1] += Rawdata.gyr.Y * Rad;	// 弧度
			avelmean[2] += Rawdata.gyr.Z * Rad;	// 弧度

			epochnum += 1.0;
		}
		// 等于一秒时
		else if (fabs(totaltime - 1.0) < 1e-3)
		{
			// 求平均
			for (int i = 0; i < 3; i++)
			{
				Pos.billimean[i] = accmean[i] / epochnum;
				Pos.anglevelmean[i] = avelmean[i] / epochnum;
			}
			// 计算姿态角
			Pos.roll = atan2(-Pos.billimean[1], -Pos.billimean[2]);
			Pos.pitch = atan2(Pos.billimean[0], sqrt(pow(Pos.billimean[1], 2) + pow(Pos.billimean[2], 2)));

			cbn[0] = -sin(Pos.pitch);
			cbn[1] = sin(Pos.roll) * cos(Pos.pitch);
			cbn[2] = cos(Pos.roll) * cos(Pos.pitch);

			wibb[0] = Pos.anglevelmean[0];
			wibb[1] = Pos.anglevelmean[1];
			wibb[2] = Pos.anglevelmean[2];

			double wd = cbn.transpose() * wibb;

			if (epochnum2 == 0) { Pos.yaw = wd * totaltime + YAW_START; epochnum2++; }
			else Pos.yaw = wd * totaltime + Pos.yaw;

			if (Pos.yaw >= PAI)Pos.yaw -= 2 * PAI;
			if (Pos.yaw <= -PAI)Pos.yaw += 2 * PAI;

			T++;

			OutputFile << fixed << setprecision(0) << T << ",";
			OutputFile << fixed << setprecision(12);
			OutputFile << Pos.yaw * Deg << "," << Pos.pitch * Deg << "," << Pos.roll * Deg << endl;

			// 重新初始化
			for (int i = 0; i < 3; i++)
			{
				accmean[i] = 0.0;
				avelmean[i] = 0.0;
			}
			totaltime = 0.0;
			epochnum = 1e-13;	// 避免跳过下一个历元
		}
		// 将该历元时间保存
		Pos.prevtime = Pos.time;
	}
	cout << "\n每秒平均的姿态角求解成功，结果已保存在文件中！\n";
}

void CalYawWithMag(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile)
{
	OutputFile << "%Time Yaw(Deg):\n";

	// 初始化加速度计和角速度的平均值
	double magmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double epochnum = 0.0, T = 0.0;
	double totaltime = 0.0;
	double dt = 0.0;

	// 逐行读取文件
	while (ReadRawData(File, Rawdata))
	{
		// 提取时间
		Pos.time = Rawdata.time;
		if (epochnum != 0)dt = Pos.time.Second - Pos.prevtime.Second;
		totaltime += dt;
		// 当读取时间小于1秒时
		if (fabs(totaltime - 1.0) > 1e-3)
		{
			magmean[0] += Rawdata.mag.X;
			magmean[1] += Rawdata.mag.Y;
			magmean[2] += Rawdata.mag.Z;

			accmean[0] += Rawdata.acc.X;
			accmean[1] += Rawdata.acc.Y;
			accmean[2] += Rawdata.acc.Z;

			epochnum += 1.0;
		}
		// 等于一秒时
		else if (fabs(totaltime - 1.0) < 1e-3)
		{
			// 求平均
			for (int i = 0; i < 3; i++)
			{
				Pos.billimean[i] = accmean[i] / epochnum;
				Pos.magmean[i] = magmean[i] / epochnum;
			}

			// 计算姿态角
			Pos.roll = atan2(-Pos.billimean[1], -Pos.billimean[2]);
			Pos.pitch = atan2(Pos.billimean[0], sqrt(pow(Pos.billimean[1], 2) + pow(Pos.billimean[2], 2)));
			
			double mx = Pos.magmean[0] * cos(Pos.pitch) + Pos.magmean[1] * sin(Pos.roll) * sin(Pos.pitch) + Pos.magmean[2] * cos(Pos.roll) * sin(Pos.pitch);
			double my = Pos.magmean[1] * cos(Pos.roll) - Pos.magmean[2] * sin(Pos.roll);
			Pos.yaw = -atan2(my, mx) + MAG_DEC * Rad;

			T++;

			OutputFile << fixed << setprecision(0) << T << ",";
			OutputFile << fixed << setprecision(12);
			OutputFile << Pos.yaw * Deg << endl;

			// 重新初始化
			for (int i = 0; i < 3; i++)
			{
				accmean[i] = 0.0;
				magmean[i] = 0.0;
			}
			totaltime = 0.0;
			epochnum = 1e-13;	// 避免跳过下一个历元
		}
		// 将该历元时间保存
		Pos.prevtime = Pos.time;
	}
	cout << "\n每秒平均的航向角求解成功，结果已保存在文件中！\n";
}