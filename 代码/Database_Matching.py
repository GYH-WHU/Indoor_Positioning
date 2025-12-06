import copy
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

def LoadDatabase_DB(filepath):
    """
    从数据库文件中加载数据，返回参考点号及其对应的信号强度的平均值。
    :param filepath: 数据库文件路径
    :return: 包含参考点号和信号强度的字典，格式为 {参考点号: [信号强度1, 信号强度2, ..., 信号强度30]}
    """
    database = {}

    # 读取文件并加载数据
    with open(filepath, 'r') as file:
        for line in file:
            parts = line.split()  # 按空格分割每一行

            # 第一列是参考点号
            reference_point = parts[0]

            # 后面30列是信号强度，转换为浮动小数
            signal_strengths = []
            for strength in parts[1:]:
                signal_strengths.append(int(strength))

            # 如果参考点号已存在，则将当前信号添加到已有的记录中
            if reference_point in database:
                database[reference_point].append(signal_strengths)
            else:
                # 如果是第一次出现该参考点号，则创建一个新记录
                database[reference_point] = [signal_strengths]

    # 对每个参考点的所有信号进行平滑和求平均
    averaged_database = {}

    for reference_point, signals_list in database.items():
        # 初始化一个列表来保存每个基站信号的总和
        num_measurements = len(signals_list)
        sum_signals = np.zeros(len(signals_list[0]))  # 假设每个基站有相同数量的信号强度

        # 对每个信号进行平滑
        for j in range(len(signals_list[0])):  # 对每个基站（列）进行处理
            valid_signals = []

            # 遍历所有历元的该基站信号，收集非-110的信号
            for signals in signals_list:
                if signals[j] != -110:
                    valid_signals.append(signals[j])

            # 如果有有效信号，则用平均值替换-110
            if valid_signals:
                avg_signal = np.mean(valid_signals)
                # 更新所有信号中该基站的位置
                for i in range(len(signals_list)):
                    if signals_list[i][j] == -110:
                        signals_list[i][j] = avg_signal

        # 计算所有信号的平均值
        for signals in signals_list:
            sum_signals += np.array([strength if strength != -110 else -110 for strength in signals])

        # 计算平均值，避免除零错误
        averaged_signals = sum_signals / num_measurements
        averaged_database[reference_point] = averaged_signals.tolist()

    return averaged_database



def LoadRefPointLocation_DB(filepath):
    """
    从参考点坐标文件中加载数据，返回参考点号及其对应的东北地参考坐标。
    :param filepath: 参考点坐标文件路径
    :return: 包含参考点号和坐标的字典，格式为 {参考点号: (经度, 纬度, 高度)}
    """
    ref_points = {}

    with open(filepath, 'r') as file:
        for line in file:
            parts = line.split()  # 按空格分割每一行

            # 第一列是参考点号
            reference_point = int(parts[0])

            # 第二、三、四列是东北地参考坐标
            E = float(parts[1])
            N = float(parts[2])
            D = float(parts[3])

            # 将参考点号和坐标存入字典中
            ref_points[reference_point] = (E, N, D)

    return ref_points

def LoadTestRef_DB(file_path):
    """
    从测试样例文件中加载数据，返回时间戳、信号强度和东北地参考坐标。
    :param file_path: 测试样例文件路径
    :return: 包含时间戳、信号强度和东北地参考坐标的数据列表
    """
    data = []

    with open(file_path, 'r') as file:
        for line in file:
            parts = line.split()  # 按空格分割每一行

            # 第一列是时间戳
            timestamp = float(parts[0])

            # 第二到三十二列是信号强度，转换为整数
            signal_strengths = []
            for strength in parts[1:31]:  # 30个基站的信号强度
                signal_strengths.append(int(strength))

            # 第三十三到三十五列是东北地参考坐标
            E = float(parts[31])
            N = float(parts[32])
            D = float(parts[33])

            # 将时间戳、信号强度和坐标存入数据列表
            data.append((timestamp, signal_strengths, (E, N, D)))

    return data

def smooth_signal_strength(data):
    """
    对信号强度为-110的基站进行平滑，使用前后各两个历元的非-110信号进行插值。
    :param data: 包含时间戳、信号强度和东北地参考坐标的数据列表。
    :return: 平滑后的数据列表，不修改原始数据。
    """
    # 深拷贝数据，确保原始数据不被修改
    smoothed_data = copy.deepcopy(data)
    result = []

    for i in range(len(smoothed_data)):
        timestamp, signal_strengths, coordinates = smoothed_data[i]

        # 遍历每个基站的信号强度
        for j in range(len(signal_strengths)):
            if signal_strengths[j] == -110:
                # 查找前后两个历元的该基站的信号强度
                prev_lines = []
                next_lines = []

                # 获取前两行的信号强度（对应相同基站）
                for k in range(1, 3):  # 前两个历元
                    if i - k >= 0:
                        if smoothed_data[i - k][1][j] != -110:
                            prev_lines.append(smoothed_data[i - k][1][j])  # 获取前两行相同基站的信号强度

                # 获取后两行的信号强度（对应相同基站）
                for k in range(1, 3):  # 后两个历元
                    if i + k < len(smoothed_data):
                        if smoothed_data[i + k][1][j] != -110:
                            next_lines.append(smoothed_data[i + k][1][j])  # 获取后两行相同基站的信号强度

                # 合并前后两行有效信号
                valid_signals = prev_lines + next_lines

                # 如果找到了有效信号，则进行平滑
                if len(valid_signals) > 0:
                    smoothed_value = np.mean(valid_signals)  # 使用有效信号的均值
                    signal_strengths[j] = smoothed_value

        # 将平滑后的数据添加到结果列表
        result.append((timestamp, signal_strengths, coordinates))

    return result


def euclidean_distance(signal1, signal2):
    """
    计算两个信号强度向量之间的欧式距离（基于信号强度差的平方和），
    忽略信号强度为-110的基站数据，并根据有效基站数进行标准化。
    :param signal1: 第一个信号强度列表
    :param signal2: 第二个信号强度列表
    :return: 两个信号的欧式距离（标准化）
    """
    # 只计算信号强度不为-110的基站的距离
    signal1 = np.array(signal1)
    signal2 = np.array(signal2)

    # 找到有效的基站（不为-110）
    valid_mask = (signal1 != -110) & (signal2 != -110)  # 只选择信号不为-110的基站

    # 计算有效信号的欧式距离
    diff = signal1[valid_mask] - signal2[valid_mask]
    squared_diff = diff ** 2

    # 如果有有效信号，则计算标准化欧式距离
    if len(squared_diff) > 0:
        return np.sqrt(np.sum(squared_diff) / len(squared_diff))  # 除以有效基站数，进行标准化
    else:
        return float('inf')  # 如果没有有效基站，返回无穷大，表示距离不可计算

def knn_location(database, reference_points, test_data, k):
    """
    使用KNN方法计算设备位置。
    :param database: 包含参考点编号及其信号强度的字典 {参考点号: [信号强度1, 信号强度2, ..., 信号强度30]}
    :param reference_points: 包含参考点号和坐标的字典 {参考点号: (E, N, D)}
    :param test_data: 测试数据列表 [(时间戳, 信号强度, 坐标)]
    :param k: KNN中的k值，默认值为3
    :return: 计算得到的设备位置 [(时间戳, 设备位置)]
    """
    results = []

    for timestamp, signal_strengths, _ in test_data:
        distances = []

        # 计算测试样本与每个参考点之间的距离
        for ref_point, ref_strengths in database.items():
            # 计算信号强度差的欧式距离，忽略-110
            distance = euclidean_distance(signal_strengths, ref_strengths)
            distances.append((ref_point, distance))

        # 按照距离排序，选择最邻近的k个参考点
        distances.sort(key=lambda x: x[1])
        nearest_neighbors = distances[:k]

        # 计算加权平均位置
        weighted_position = np.array([0.0, 0.0, 0.0])
        total_weight = 0

        for neighbor, distance in nearest_neighbors:
            # 获取参考点的坐标
            neighbor = int(neighbor)
            ref_position = reference_points[neighbor]
            # 权重是与距离成反比，距离越小权重越大
            weight = 1 / (distance + 1e-6)  # 防止除零错误
            weighted_position += np.array(ref_position) * weight
            total_weight += weight

        # 归一化，得到最终的设备位置
        weighted_position /= total_weight
        results.append((timestamp, weighted_position))

    return results


def plot_trajectory(reference_data, computed_data):
    """
    绘制参考真值和计算得到的设备位置的二维平面图，展示E（东向）和N（北向）坐标。
    :param reference_data: 参考结果数据 [(时间戳, E, N, D), ...]
    :param computed_data: 计算得到的设备位置 [(时间戳, E, N, D), ...]
    """
    # 提取参考真值的E、N坐标
    ref_E = [coords[0] for _, _, coords in reference_data]  # 获取（E）
    ref_N = [coords[1] for _, _, coords in reference_data]  # 获取（N）

    # 提取计算得到的设备位置的E、N坐标
    comp_E = [coords[0] for _, coords in computed_data]  # 获取计算结果的（E）
    comp_N = [coords[1] for _, coords in computed_data]  # 获取计算结果的（N）

    # 创建一个二维平面图
    plt.figure(figsize=(10, 6))

    # 绘制参考真值轨迹
    plt.plot(ref_E, ref_N, 'bo-', label='Reference', markersize=4, linewidth=1)

    # 绘制计算得到的设备轨迹
    plt.plot(comp_E, comp_N, 'r^-', label='Ours', markersize=4, linewidth=1)

    # 添加标题和轴标签
    plt.xlabel('E')
    plt.ylabel('N')

    # 显示图例
    plt.legend()

    # 显示网格
    plt.grid(True)

    # 展示图像
    plt.show()

def rmse(errors):
    """计算RMSE"""
    return np.sqrt(np.mean(np.array(errors) ** 2))

def standard_deviation(errors):
    """计算标准差"""
    return np.std(errors)

def plot_error_comparison_DB(reference_data, computed_data):
    """
    绘制参考真值与计算值的误差图，展示 E 和 N 方向误差随时间变化的趋势。
    :param reference_data: 参考结果数据 [(时间戳, E, N, D), ...]
    :param computed_data: 计算得到的设备位置 [(时间戳, E, N, D), ...]
    """
    # 提取参考真值的E、N坐标和时间戳
    ref_time = [timestamp for timestamp, _, _ in reference_data]  # 提取时间戳
    ref_E = [coords[0] for _, _, coords in reference_data]  # 获取参考的E坐标
    ref_N = [coords[1] for _, _, coords in reference_data]  # 获取参考的N坐标

    # 提取计算得到的设备位置的E、N坐标
    comp_E = [coords[0] for _, coords in computed_data]  # 获取计算结果的E坐标
    comp_N = [coords[1] for _, coords in computed_data]  # 获取计算结果的N坐标

    # 计算E方向误差（参考值与计算值之差）
    dE = [ref_E[i] - comp_E[i] for i in range(len(ref_E))]
    dN = [ref_N[i] - comp_N[i] for i in range(len(ref_N))]

    # 计算 RMSE 和 STD
    rmse_E = rmse(dE)
    rmse_N = rmse(dN)
    std_E = standard_deviation(dE)
    std_N = standard_deviation(dN)

    # 创建误差图：E方向和N方向误差随时间变化
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    # 绘制 E 方向误差图
    axs[0].plot(ref_time, dE, 'b-', label="dE (E Error)")
    axs[0].set_title('dE')
    axs[0].set_xlabel('TimeStamp')
    axs[0].set_ylabel('dE')
    axs[0].grid(True)
    axs[0].legend()

    # 标注 RMSE 和 STD
    axs[0].text(0.95, 0.95, f'RMSE: {rmse_E:.3f}m\nSTD: {std_E:.3f}m', transform=axs[0].transAxes,
                ha='right', va='top', fontsize=10, bbox=dict(facecolor='white', alpha=0.7))

    # 绘制 N 方向误差图
    axs[1].plot(ref_time, dN, 'r-', label="dN (N Error)")
    axs[1].set_title('dN')
    axs[1].set_xlabel('TimeStamp')
    axs[1].set_ylabel('dN')
    axs[1].grid(True)
    axs[1].legend()

    # 标注 RMSE 和 STD
    axs[1].text(0.95, 0.95, f'RMSE: {rmse_N:.3f}m\nSTD: {std_N:.3f}m', transform=axs[1].transAxes,
                ha='right', va='top', fontsize=10, bbox=dict(facecolor='white', alpha=0.7))

    # 调整子图布局
    plt.tight_layout()

    # 显示图形
    plt.show()