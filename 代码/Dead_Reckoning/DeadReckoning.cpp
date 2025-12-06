#include"DR_Structs.h"

/* 行人航迹推算 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输出：位置 例：ALIGNPOS pos.yaw
// 输出：位置文件
void CalReference(ifstream& File, GEOCOOR& Rawdata, ofstream& OutputFile)
{
	OutputFile << "%dE dN(m):\n";

	GEOCOOR Baseblh;
	Baseblh.longitude = L_Start; Baseblh.latitude = B_Start; Baseblh.height = 0;
	double base[3] = {B_Start ,L_Start, 0 }, Basexyz[3] = { 0.0,0.0,0.0 };
	BLHToXYZ(base, Basexyz, R_WGS84, F_WGS84);
	// 逐行读取文件
	while (ReadRefData(File, Rawdata))
	{
		double Roverxyz[3] = { 0.0,0.0,0.0 };
		double Roverblh[3] = { Rawdata.latitude,Rawdata.longitude,0 };
		BLHToXYZ(Roverblh, Roverxyz, R_WGS84, F_WGS84);

		double dENU[3] = { 0.0,0.0,0.0 };
		CompEnudPos(Basexyz, Roverxyz, &Baseblh, dENU);

		OutputFile << fixed << setprecision(3) << Rawdata.time << ",";
		OutputFile << fixed << setprecision(12);
		OutputFile << dENU[0] << "," << dENU[1] << endl;

	}
	cout << "\n参考文件的路径计算成功，结果已保存在文件中！\n";
}

/* 陀螺行人航迹推算 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输出：位置 例：ALIGNPOS pos.yaw
// 输出：位置文件
void GyrDeadReckoning(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile)
{
	OutputFile << "%E,N(m),G(m/s2):\n";

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double accmol = 0.0;		// 加速度计的模
	double E = 0.0, N = 0.0;	// 初始化位置

	double epochnum = 0.0, epochnum2 = 0.0;
	double dt = 0.0, totaltime = 0.0;

	double Gnum = 0.0;	// 重力加速度小于阈值的次数
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
		if (totaltime <= 1 + 1e-3)
		{
			accmean[0] += Rawdata.acc.X;
			accmean[1] += Rawdata.acc.Y;
			accmean[2] += Rawdata.acc.Z;

			avelmean[0] += Rawdata.gyr.X * Rad;	// 弧度
			avelmean[1] += Rawdata.gyr.Y * Rad;	// 弧度
			avelmean[2] += Rawdata.gyr.Z * Rad;	// 弧度

			double curacc[3] = { 0.0,0.0,0.0 };
			curacc[0] = Rawdata.acc.X;
			curacc[1] = Rawdata.acc.Y;
			curacc[2] = Rawdata.acc.Z;

			accmol = sqrt(pow(curacc[0], 2) + pow(curacc[1], 2) + pow(curacc[2], 2));
			if (accmol <= 9.2)Gnum++;	// 重力小于阈值则加步长

			epochnum += 1.0;
		}
		// 等于一秒时
		if (fabs(totaltime - 1) < 1e-3 && fabs(totaltime - 1) > -1e-3)
		{
			// 求平均
			for (int i = 0; i < 3; i++)
			{
				Pos.billimean[i] = accmean[i] / epochnum;
				Pos.anglevelmean[i] = avelmean[i] / epochnum;
			}
			// 更新位置
			// 计算姿态角
			Pos.roll = atan2(-Pos.billimean[1], -Pos.billimean[2]);
			Pos.pitch = atan2(Pos.billimean[0], sqrt(pow(Pos.billimean[1], 2) + pow(Pos.billimean[2], 2)));

			cbn[0] = -sin(Pos.pitch);
			cbn[1] = sin(Pos.roll) * cos(Pos.pitch);
			cbn[2] = cos(Pos.roll) * cos(Pos.pitch);

			// 使用陀螺计算航向角
			wibb[0] = Pos.anglevelmean[0];
			wibb[1] = Pos.anglevelmean[1];
			wibb[2] = Pos.anglevelmean[2];

			double wd = cbn.transpose() * wibb;

			if (epochnum2 == 0) { Pos.yaw = wd * totaltime + YAW_START; epochnum2++; }
			else Pos.yaw = wd * totaltime + Pos.yaw;

			if (Pos.yaw >= PAI)Pos.yaw -= 2 * PAI;
			if (Pos.yaw <= -PAI)Pos.yaw += 2 * PAI;

			// 利用航向角来推算位置
			E += (STEP * Gnum * sin(Pos.yaw));
			N += (STEP * Gnum * cos(Pos.yaw));

			// 重新初始化
			for (int i = 0; i < 3; i++)
			{
				accmean[i] = 0.0;
				avelmean[i] = 0.0;
			}
			Gnum = 0;
			totaltime = 0.0;
			epochnum = 1e-13;	// 避免跳过下一个历元

			OutputFile << fixed << setprecision(2) << Rawdata.time.Second << ",";
			OutputFile << fixed << setprecision(12);
			OutputFile << E << "," << N << endl;

		}
		// 将该历元时间保存
		Pos.prevtime = Pos.time;
	}
}/* 磁强计行人航迹推算 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输出：位置 例：ALIGNPOS pos.yaw
// 输出：位置文件
void MagDeadReckoning(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile)
{
	OutputFile << "%E,N(m):\n";

	// 初始化加速度计和角速度的平均值
	double magmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double epochnum = 0.0, T = 0.0;
	double totaltime = 0.0;
	double dt = 0.0;

	double accmol = 0.0;
	double Gnum = 0.0;
	double E = 0.0, N = 0.0;
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

			double curacc[3] = { 0.0,0.0,0.0 };
			curacc[0] = Rawdata.acc.X;
			curacc[1] = Rawdata.acc.Y;
			curacc[2] = Rawdata.acc.Z;

			accmol = sqrt(pow(curacc[0], 2) + pow(curacc[1], 2) + pow(curacc[2], 2));
			if (accmol <= 9.2)Gnum++;	// 重力小于阈值则加步长

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

			// 利用航向角来推算位置
			E += (STEP * Gnum * sin(Pos.yaw));
			N += (STEP * Gnum * cos(Pos.yaw));


			OutputFile << fixed << setprecision(0) << Rawdata.time.Second << ",";
			OutputFile << fixed << setprecision(12);
			OutputFile << E << "," << N << endl;

			// 重新初始化
			for (int i = 0; i < 3; i++)
			{
				accmean[i] = 0.0;
				magmean[i] = 0.0;
			}
			Gnum = 0;
			totaltime = 0.0;
			epochnum = 1e-13;	// 避免跳过下一个历元
		}
		// 将该历元时间保存
		Pos.prevtime = Pos.time;
	}
	cout << "\n每秒平均的航向角求解成功，结果已保存在文件中！\n";
}
