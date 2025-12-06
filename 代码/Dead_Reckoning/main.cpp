#include"DR_Structs.h"

int main()
{
	// 打开文件
	string rawdata_file_path = "数据-行人航迹推算\\sensors_matrix.txt";
	string refdata_file_path = "数据-行人航迹推算\\pos_reference.txt";
	string result_file_path1 = "Result\\AlignEpoch.txt";
	string result_file_path2 = "Result\\AlignSecond.txt";
	string result_file_path3 = "Result\\YawSecond.txt";
	string result_file_path4 = "Result\\RefdENU.txt";
	string result_file_path5 = "Result\\GyrDeadReckoning.txt";
	string result_file_path6 = "Result\\MagDeadReckoning.txt";

	ifstream rawdata_file1(rawdata_file_path);
	ifstream rawdata_file2(rawdata_file_path);
	ifstream rawdata_file3(rawdata_file_path);
	ifstream rawdata_file5(rawdata_file_path);
	ifstream rawdata_file6(rawdata_file_path);
	ifstream refdata_file1(refdata_file_path);
	ofstream result_file1(result_file_path1);
	ofstream result_file2(result_file_path2);
	ofstream result_file3(result_file_path3);
	ofstream result_file4(result_file_path4);
	ofstream result_file5(result_file_path5);
	ofstream result_file6(result_file_path6);

	// 结构体定义
	RAWDAT Rawdata;
	ALIGNPOS pos1, pos2, pos3, pos5, pos6;
	GEOCOOR blh;

	// 计算每个历元的姿态角
	Align_EveryEpoch(rawdata_file1, Rawdata, pos1, result_file1);
	Align_EverySecond(rawdata_file2, Rawdata, pos2, result_file2);
	CalYawWithMag(rawdata_file3, Rawdata, pos3, result_file3);

	CalReference(refdata_file1, blh, result_file4);
	GyrDeadReckoning(rawdata_file5, Rawdata, pos5, result_file5);
	MagDeadReckoning(rawdata_file6, Rawdata, pos6, result_file6);
	
}