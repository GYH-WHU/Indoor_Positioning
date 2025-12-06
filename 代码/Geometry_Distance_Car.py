import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def LoadBaseData_Geo(file_path):
    """
    从txt文件中加载基站的数据
    :param file_path: 文件路径
    :return: 基站编号以及坐标的字典
    """
    basestations = {}

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()  # 去掉每行的空白字符
            if line:  # 如果该行非空
                parts = line.split()
                if len(parts) == 4:  # 确保该行有4列数据
                    try:
                        station_id = int(parts[0])  # 基站编号
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])  # 坐标
                        basestations[station_id] = (x, y, z)
                    except ValueError:
                        print(f"警告：跳过格式错误的行：{line}")
                else:
                    print(f"警告：跳过格式错误的行：{line}")

    return basestations

def LoadRoverData_Geo(file_path):
    """
    从文件中加载流动站数据
    :param file_path: 流动站数据文件的路径
    :return: 时间戳和距离数据的列表
    """
    data = []

    with open(file_path, 'r') as file:
        for line in file:
            parts = line.split()
            # 第一个值是时间戳，其余8个值是基站的距离
            timestamp = int(parts[0])  # 时间戳
            distances = list(map(float, parts[1:]))  # 基站1-8的距离
            data.append((timestamp, distances))

    return data

def LoadReferenceData_Geo(file_path):
    """
    从文件中加载参考结果数据（时间戳和三维位置）。
    :param file_path: 文件路径
    :return: 包含时间戳和三维位置的列表 [(timestamp, x, y, z), ...]
    """
    data = []

    with open(file_path, 'r') as file:
        for line in file:
            # 去除每行的空白字符
            line = line.strip()

            if line:  # 如果该行非空
                parts = line.split()  # 按空白字符分割
                if len(parts) == 4:  # 确保该行有4列数据
                    try:
                        timestamp = int(parts[0])  # 时间戳
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])  # 三维坐标
                        data.append((timestamp, x, y, z))  # 将数据作为元组添加到列表中
                    except ValueError:
                        print(f"警告：跳过格式错误的行：{line}")
                else:
                    print(f"警告：跳过格式错误的行：{line}")

    return data

def GeometryLocation(rover, base):
    """
    使用几何定位方法实现距离交汇定位
    :param rover: 流动站数据（时间戳、基站距离）
    :param base: 基站数据（基站编号、位置）
    :return: 计算得到的目标位置（x，y，z）
    """
    results = []

    for timestamp, distances in rover:
        # 初始化目标位置（假设初始值为原点）
        initial_guess = [0, 0, 0]

        # 定义残差函数，计算每个基站的距离差
        def residuals(params):
            x, y, z = params
            res = []
            for i, distance in enumerate(distances):
                station_id = i + 1  # 基站编号从1开始
                x_i, y_i, z_i = base[station_id]
                # 计算目标位置和基站之间的距离差
                predicted_distance = np.sqrt((x - x_i)**2 + (y - y_i)**2 + (z - z_i)**2)
                res.append(predicted_distance - distance)
            return np.array(res)

        # 调用 least_squares 进行非线性最小二乘法计算
        result = least_squares(residuals, initial_guess)

        # 存储时间戳和计算得到的位置
        x, y, z = result.x
        results.append((timestamp, x, y, z))

    return results

def calculate_dop(base, computed_position):
    """
    计算水平 DOP (HDOP)、高程 DOP (VDOP) 和三维 DOP (GDOP)。
    :param base: 基站数据（字典，基站编号到位置的映射）
    :param computed_position: 计算得到的目标位置（x, y, z）
    :return: 水平 DOP（HDOP），高程 DOP（VDOP），三维 DOP（GDOP）
    """
    # 计算基站与目标位置之间的偏导数（Jacobian矩阵）
    jacobian = []
    x, y, z = computed_position  # 目标位置

    for station_id, (x_i, y_i, z_i) in base.items():
        # 计算基站和目标位置之间的距离
        r = np.sqrt((x - x_i)**2 + (y - y_i)**2 + (z - z_i)**2)
        # 计算目标位置和基站的偏导数
        dx = (x - x_i) / r
        dy = (y - y_i) / r
        dz = (z - z_i) / r
        jacobian.append([dx, dy, dz])  # 每个基站的偏导数

    jacobian = np.array(jacobian)

    # 计算Jacobian矩阵的伪逆
    jacobian_pseudo_inv = np.linalg.pinv(jacobian)

    # 计算协方差矩阵 Q
    covariance_matrix = np.dot(jacobian_pseudo_inv, jacobian_pseudo_inv.T)

    # 提取水平、垂直和三维DOP
    hdop = np.sqrt(covariance_matrix[0, 0] + covariance_matrix[1, 1])  # 水平DOP (HDOP)
    vdop = np.sqrt(covariance_matrix[2, 2])  # 高程DOP (VDOP)
    gdop = np.sqrt(np.trace(covariance_matrix))  # 三维DOP (GDOP)

    return hdop, vdop, gdop

######## 画图 ########
def plot_3d_trajectory(reference_data, computed_data):
    """
    绘制参考结果和解算结果的三维图，使用细线表示。
    :param reference_data: 参考结果数据 [(timestamp, x, y, z), ...]
    :param computed_data: 解算结果数据 [(timestamp, x, y, z), ...]
    """
    # 提取参考结果的 x, y, z 坐标
    ref_x = [x for _, x, _, _ in reference_data]
    ref_y = [y for _, _, y, _ in reference_data]
    ref_z = [z for _, _, _, z in reference_data]

    # 提取解算结果的 x, y, z 坐标
    comp_x = [x for _, x, _, _ in computed_data]
    comp_y = [y for _, _, y, _ in computed_data]
    comp_z = [z for _, _, _, z in computed_data]

    # 创建一个 3D 图
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制参考结果（细线，较小点）
    ax.scatter(ref_x, ref_y, ref_z, c='b', label='Reference', marker='.', s=15, linewidths=0.5)  # 大小 15, 线宽 0.5

    # 绘制解算结果（细线，较小点）
    ax.scatter(comp_x, comp_y, comp_z, c='r', label='Ours', marker='.', s=15, linewidths=0.5)  # 大小 15, 线宽 0.5

    # 设置图形标题和轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.grid(True)  # 启用网格

    # 显示图例
    ax.legend()

    # 显示图形
    plt.show()

def plot_2d_trajectory(reference_data, computed_data):
    """
    绘制参考结果和解算结果的平面二维轨迹图，使用不同颜色区分。
    :param reference_data: 参考结果数据 [(timestamp, x, y, z), ...]
    :param computed_data: 解算结果数据 [(timestamp, x, y, z), ...]
    """
    # 提取参考结果的 x 和 y 坐标（忽略 z 坐标）
    ref_x = [x for _, x, _, _ in reference_data]
    ref_y = [y for _, _, y, _ in reference_data]

    # 提取解算结果的 x 和 y 坐标（忽略 z 坐标）
    comp_x = [x for _, x, _, _ in computed_data]
    comp_y = [y for _, _, y, _ in computed_data]

    # 创建一个平面二维图
    plt.figure(figsize=(10, 8))

    # 绘制解算结果的轨迹
    plt.plot(comp_x, comp_y, 'r-', label='Ours Track', linewidth=2)

    # 绘制参考结果的轨迹
    plt.plot(ref_x, ref_y, 'b-', label='Reference Track', linewidth=1)

    # 设置图形标题和轴标签
    plt.xlabel('X')
    plt.ylabel('Y')

    # 显示图例
    plt.legend()

    # 显示网格
    plt.grid(True)

    # 显示图形
    plt.tight_layout()
    plt.show()

def rmse(errors):
    return np.sqrt(np.mean(np.array(errors) ** 2))

def standard_deviation(errors):
    return np.std(errors)

def plot_error_comparison(reference_data, computed_data):
    """
    绘制解算结果与参考结果的误差图，展示 x、y、z 的误差随时间变化的趋势。
    :param reference_data: 参考结果数据 [(timestamp, x, y, z), ...]
    :param computed_data: 解算结果数据 [(timestamp, x, y, z), ...]
    """
    # 获取最小的数据量
    min_len = min(len(reference_data), len(computed_data))

    # 提取时间戳、参考结果和解算结果，使用最小长度
    time_stamps = [timestamp for timestamp, _, _, _ in reference_data[:min_len]]
    ref_x = [x for _, x, _, _ in reference_data[:min_len]]
    ref_y = [y for _, _, y, _ in reference_data[:min_len]]
    ref_z = [z for _, _, _, z in reference_data[:min_len]]

    comp_x = [x for _, x, _, _ in computed_data[:min_len]]
    comp_y = [y for _, _, y, _ in computed_data[:min_len]]
    comp_z = [z for _, _, _, z in computed_data[:min_len]]

    # 计算误差
    error_x = [comp_x[i] - ref_x[i] for i in range(min_len)]
    error_y = [comp_y[i] - ref_y[i] for i in range(min_len)]
    error_z = [comp_z[i] - ref_z[i] for i in range(min_len)]

    # 计算 RMSE 和 STD
    rmse_x = rmse(error_x)
    rmse_y = rmse(error_y)
    rmse_z = rmse(error_z)

    std_x = standard_deviation(error_x)
    std_y = standard_deviation(error_y)
    std_z = standard_deviation(error_z)

    # 创建误差图：三个坐标轴的误差随时间变化
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))

    # 绘制 X 轴误差
    axs[0].plot(time_stamps, error_x, 'b-', label=" dx ")
    axs[0].set_title('X')
    axs[0].text(0.95, 0.95, f'RMSE: {rmse_x:.3f}m\nSTD: {std_x:.3f}m', transform=axs[0].transAxes,
                ha='right', va='top', fontsize=10, bbox=dict(facecolor='white', alpha=0.7))
    axs[0].grid(True)  # 启用网格

    # 绘制 Y 轴误差
    axs[1].plot(time_stamps, error_y, 'r-', label=" dy ")
    axs[1].set_title('Y')
    axs[1].text(0.95, 0.95, f'RMSE: {rmse_y:.3f}m\nSTD: {std_y:.3f}m', transform=axs[1].transAxes,
                ha='right', va='top', fontsize=10, bbox=dict(facecolor='white', alpha=0.7))
    axs[1].grid(True)  # 启用网格

    # 绘制 Z 轴误差
    axs[2].plot(time_stamps, error_z, 'g-', label=" dz ")
    axs[2].set_title('Z')
    axs[2].set_xlabel('TimeStamp')
    axs[2].text(0.95, 0.95, f'RMSE: {rmse_z:.3f}m\nSTD: {std_z:.3f}m', transform=axs[2].transAxes,
                ha='right', va='top', fontsize=10, bbox=dict(facecolor='white', alpha=0.7))
    axs[2].grid(True)  # 启用网格

    # 调整子图布局
    plt.tight_layout()

    # 显示图形
    plt.show()

def plot_dop_values(reference_data, computed_data, base):
    """
    绘制PDOP、VDOP、HDOP值随时间变化的趋势。
    :param reference_data: 参考结果数据 [(timestamp, x, y, z), ...]
    :param computed_data: 解算结果数据 [(timestamp, x, y, z), ...]
    :param base: 基站数据（字典，基站编号到位置的映射）
    """
    # 获取最小的数据量
    min_len = min(len(reference_data), len(computed_data))

    # 提取时间戳、参考结果和解算结果，使用最小长度
    time_stamps = [timestamp for timestamp, _, _, _ in reference_data[:min_len]]
    ref_x = [x for _, x, _, _ in reference_data[:min_len]]
    ref_y = [y for _, _, y, _ in reference_data[:min_len]]
    ref_z = [z for _, _, _, z in reference_data[:min_len]]

    comp_x = [x for _, x, _, _ in computed_data[:min_len]]
    comp_y = [y for _, _, y, _ in computed_data[:min_len]]
    comp_z = [z for _, _, _, z in computed_data[:min_len]]

    # 计算 DOP 并存储
    hdop_values = []
    vdop_values = []
    gdop_values = []

    for i in range(min_len):
        # 计算 DOP 值
        computed_position = [comp_x[i], comp_y[i], comp_z[i]]
        hdop, vdop, gdop = calculate_dop(base, computed_position)

        # 将计算结果添加到列表
        hdop_values.append(hdop)
        vdop_values.append(vdop)
        gdop_values.append(gdop)

    # 创建 DOP 图：三个坐标轴的误差随时间变化
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))

    # 绘制 HDOP (水平 DOP)
    axs[0].plot(time_stamps, hdop_values, 'b-', label='HDOP')
    axs[0].set_title('HDOP')
    axs[0].set_ylabel('DOP')
    axs[0].legend(loc='best')
    axs[0].grid(True)

    # 绘制 VDOP (垂直 DOP)
    axs[1].plot(time_stamps, vdop_values, 'r-', label='VDOP')
    axs[1].set_title('VDOP')
    axs[1].set_ylabel('DOP')
    axs[1].legend(loc='best')
    axs[1].grid(True)

    # 绘制 GDOP (三维 DOP)
    axs[2].plot(time_stamps, gdop_values, 'g-', label='GDOP')
    axs[2].set_title('GDOP')
    axs[2].set_xlabel('TimeStamp')
    axs[2].set_ylabel('PDOP')
    axs[2].legend(loc='best')
    axs[2].grid(True)

    # 调整子图布局
    plt.tight_layout()

    # 显示图形
    plt.show()