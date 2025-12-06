from Geometry_Distance_Car import GeometryLocation, calculate_dop, \
    LoadBaseData_Geo, LoadRoverData_Geo, LoadReferenceData_Geo, \
    plot_3d_trajectory, plot_2d_trajectory, plot_error_comparison, plot_dop_values  # 加载几何定位函数（计算、读数据、绘图）

from Dead_Reckoning import LoadRawData_DR, LoadAlignData_DR, LoadYawData_DR, LoaddENData_DR, \
    plotrawdata_DR, plotalign_DR, plotyaw_DR, plot_gravity_DR, plotdeadReckong_DR, plotdenu_DR

from Database_Matching import LoadDatabase_DB, LoadRefPointLocation_DB, LoadTestRef_DB, \
    smooth_signal_strength, \
    knn_location, \
    plot_trajectory, plot_error_comparison_DB

def main():
    print("请选择定位方式：")
    print("1. 几何距离定位（无人车）")
    print("2. 航迹推算")
    print("3. 数据库匹配")

    choice = input("请输入选择的数字: ")

    if choice == '1':
        # 文件路径
        base_file_path = "数据-距离交会/数据-小车-UWB/base-station-location.txt"
        rover_file_path1 = "数据-距离交会/数据-小车-UWB/range_tag_2.txt"
        rover_file_path2 = "数据-距离交会/数据-小车-UWB/range_tag_3.txt"
        reference_file_path1 = "数据-距离交会/数据-小车-UWB/true_position_tag_2.txt"  # 参考结果文件路径
        reference_file_path2 = "数据-距离交会/数据-小车-UWB/true_position_tag_3.txt"  # 参考结果文件路径

        # 加载基站、流动站和参考结果数据
        basestations = LoadBaseData_Geo(base_file_path)
        rover1 = LoadRoverData_Geo(rover_file_path1)
        rover2 = LoadRoverData_Geo(rover_file_path2)
        reference_data1 = LoadReferenceData_Geo(reference_file_path1)
        reference_data2 = LoadReferenceData_Geo(reference_file_path2)

        # 进行几何定位计算
        results1 = GeometryLocation(rover1, basestations)
        results2 = GeometryLocation(rover2, basestations)

        # 输出定位结果
        # print("\n几何定位结果：")
        # for result in results:
        #     timestamp, x, y, z, hdop, vdop, gdop = result  # 解算结果包含DOP值
        #     print(f"时间戳: {timestamp}, 位置: x = {x:.3f}, y = {y:.3f}, z = {z:.3f}")
        #     print(f"  HDOP = {hdop:.3f}, VDOP = {vdop:.3f}, GDOP = {gdop:.3f}")

        # 绘制参考结果与解算结果的三维与二维图
        plot_3d_trajectory(reference_data1, results1)
        plot_2d_trajectory(reference_data1, results1)
        # 绘制误差图：计算并显示解算结果与参考结果的误差
        plot_error_comparison(reference_data1, results1)
        # 绘制 DOP 图：展示水平 DOP、垂直 DOP 和三维 DOP 随时间变化的趋势
        plot_dop_values(reference_data1, results1, basestations)

        # 绘制参考结果与解算结果的三维图
        plot_3d_trajectory(reference_data2, results2)
        plot_2d_trajectory(reference_data2, results2)
        # 绘制误差图：计算并显示解算结果与参考结果的误差
        plot_error_comparison(reference_data2, results2)
        # 绘制 DOP 图：展示水平 DOP、垂直 DOP 和三维 DOP 随时间变化的趋势
        plot_dop_values(reference_data2, results2, basestations)

    elif choice == '2':
        # 文件路径
        Rawdata_file_path = "Dead_Reckoning/Dead_Reckoning/数据-行人航迹推算/sensors_matrix.txt"
        AlignEpoch_file_path = "Dead_Reckoning/Dead_Reckoning/Result/AlignEpoch.txt"
        AlignSecond_file_path = "Dead_Reckoning/Dead_Reckoning/Result/AlignSecond.txt"
        AlignYaw_file_path = "Dead_Reckoning/Dead_Reckoning/Result/YawSecond.txt"
        RefdEN_file_path = "Dead_Reckoning/Dead_Reckoning/Result/RefdENU.txt"
        GyrDR_file_path = "Dead_Reckoning/Dead_Reckoning/Result/GyrDeadReckoning.txt"
        MagDR_file_path = "Dead_Reckoning/Dead_Reckoning/Result/MagDeadReckoning.txt"

        # 读取文件内容
        Rawdata = LoadRawData_DR(Rawdata_file_path)
        AlignEpoch = LoadAlignData_DR(AlignEpoch_file_path, 0)
        AlignSeceond = LoadAlignData_DR(AlignSecond_file_path, 1)
        AlignYaw = LoadYawData_DR(AlignYaw_file_path)
        RefdEN = LoaddENData_DR(RefdEN_file_path)
        GyrDR = LoaddENData_DR(GyrDR_file_path)
        MagDR = LoaddENData_DR(MagDR_file_path)

        # 画图
        plotrawdata_DR(Rawdata)
        plotalign_DR(AlignEpoch)
        plotalign_DR(AlignSeceond)
        plotyaw_DR(AlignSeceond, AlignYaw)
        plot_gravity_DR(AlignEpoch)
        plotdeadReckong_DR(RefdEN)
        # plotdeadReckong_DR(GyrDR)
        # plotdeadReckong_DR(MagDR)
        plotdenu_DR(GyrDR, MagDR)

        i=0

    elif choice == '3':
        # 文件路径
        Database_file_path = "数据-数据库匹配/数据-信号强度指纹识别/database.data"
        RefPoint_file_path = "数据-数据库匹配/数据-信号强度指纹识别/rp_loc.txt"
        TestRef_file_path = "数据-数据库匹配/数据-信号强度指纹识别/test1_with_ref.data"

        # 加载数据库、参考点位置、测试数据
        Database = LoadDatabase_DB(Database_file_path)
        RefPoint = LoadRefPointLocation_DB(RefPoint_file_path)
        TestRef = LoadTestRef_DB(TestRef_file_path)

        # 平滑测试数据中的基站数据
        Smoothed_TestRef = smooth_signal_strength(TestRef)

        # 使用 KNN 定位算法计算设备位置
        for k in range(1, 7):  # KNN中的K值
            results1 = knn_location(Database, RefPoint, TestRef, k)
            results2 = knn_location(Database, RefPoint, Smoothed_TestRef, k)

            plot_trajectory(TestRef, results1)
            plot_trajectory(TestRef, results2)

            plot_error_comparison_DB(TestRef, results1)
            plot_error_comparison_DB(TestRef, results2)

    else:
        print("无效输入，请重新选择！")


if __name__ == "__main__":
    main()
