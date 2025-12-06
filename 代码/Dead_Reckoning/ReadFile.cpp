#include"DR_Structs.h"

/* 读取惯性导航的数据 */
// 输入：文件流flie (ifstream)
// 输出：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
bool ReadRawData(ifstream& file, RAWDAT& Rawdata)
{	
	// 读取文件数据
	string line;
	while (getline(file, line))
	{
		// 行为空则继续循环
		if (line.empty())return false;

		// 获得第一个","之前的内容
		stringstream ss(line);
		string part;
		
		// 时间
		getline(ss, part, ',');
		Rawdata.time.Second = stod(part);

		// 陀螺（deg/s）
		getline(ss, part, ',');	// X
		Rawdata.gyr.X = stod(part);
		getline(ss, part, ',');	// Y
		Rawdata.gyr.Y = stod(part);
		getline(ss, part, ',');	// Z
		//Rawdata.gyr.Z = stod(part) + 0.24;	// 零偏补偿
		Rawdata.gyr.Z = stod(part);

		// 加速度计（m/s^2）
		getline(ss, part, ',');	// X
		Rawdata.acc.X = stod(part);
		getline(ss, part, ',');	// Y
		Rawdata.acc.Y = stod(part);
		getline(ss, part, ',');	// Z
		Rawdata.acc.Z = stod(part);

		// 磁强计（mGauss）
		getline(ss, part, ',');	// X
		Rawdata.mag.X = stod(part);
		getline(ss, part, ',');	// Y
		Rawdata.mag.Y = stod(part);
		getline(ss, part, ',');	// Z
		Rawdata.mag.Z = stod(part);

		return true;	// 读取成功
	}
	return false;	// 读取一行失败
}

/* 读取参考坐标的数据 */
// 输入：文件流flie (ifstream)
// 输出：位置信息
bool ReadRefData(ifstream& file, GEOCOOR& Rawdata)
{
	// 读取文件数据
	string line;
	while (getline(file, line))
	{
		// 行为空则继续循环
		if (line.empty())return false;

		// 获得第一个","之前的内容
		stringstream ss(line);
		string part;

		// 时间
		getline(ss, part, ' ');
		Rawdata.time = stod(part);

		// 经纬度
		getline(ss, part, ' ');	// X
		Rawdata.latitude = stod(part);
		getline(ss, part, ' ');	// Y
		Rawdata.longitude= stod(part);

		return true;	// 读取成功
	}
	return false;	// 读取一行失败
}