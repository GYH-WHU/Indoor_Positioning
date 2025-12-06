import numpy as np
import matplotlib.pyplot as plt

def LoadRawData_DR(file_path):
    """
    从文件中加载原始数据，包括时间戳、陀螺仪角速度、加速度计加速度和磁强计数据。
    :param file_path: 文件路径
    :return: 包含时间、陀螺仪角速度、加速度计加速度和磁强计数据的列表
    """
    data = []

    with open(file_path, 'r') as file:
        for line in file:
            # 去除行尾的换行符和空白字符
            line = line.strip()

            # 如果该行非空
            if line:
                parts = line.split(',')  # 按逗号分割每一行数据

                # 提取各个数据项
                timestamp = float(parts[0])  # 时间
                gyro = [float(parts[i]) for i in range(1, 4)]  # 陀螺仪角速度
                accelerometer = [float(parts[i]) for i in range(4, 7)]  # 加速度计加速度
                magnetometer = [float(parts[i]) for i in range(7, 10)]  # 磁强计数据

                # 将时间、陀螺仪角速度、加速度计加速度和磁强计数据存入数据列表
                data.append((timestamp, gyro, accelerometer, magnetometer))

    return data

def LoadAlignData_DR(file_path, mode):
    """
    从文件中加载对准数据，返回时间戳、俯仰角、航向角和横滚角。
    :param file_path: 文件路径
    :return: 包含时间戳、俯仰角、航向角、横滚角的数据列表
    """
    data = []

    with open(file_path, 'r') as file:
        lines = file.readlines()

        # 跳过第一行
        for line in lines[1:]:
            parts = line.split(',')  # 按逗号分割每一行数据

            # 第一列是时间戳
            timestamp = float(parts[0])

            # 第二到四列是俯仰角、航向角、横滚角
            pitch = float(parts[1])  # 俯仰角
            yaw = float(parts[2])  # 航向角
            roll = float(parts[3])  # 横滚角
            if mode == 0:
                g = float(parts[4]) # 重力

            # 将时间戳和角度数据存入数据列表
            if mode == 0:
                data.append((timestamp, pitch, yaw, roll, g))
            if mode == 1:
                data.append((timestamp, pitch, yaw, roll))

    return data

def LoadYawData_DR(file_path):
    """
    从航向角数据文件中加载数据，返回时间戳和航向角。
    :param file_path: 文件路径
    :return: 包含时间戳和航向角的数据列表 [(timestamp, yaw), ...]
    """
    data = []

    with open(file_path, 'r') as file:
        lines = file.readlines()

        # 跳过第一行
        for line in lines[1:]:
            parts = line.split(',')  # 按逗号分割每一行数据

            # 第一列是时间戳
            timestamp = float(parts[0])

            # 第二列是航向角
            yaw = float(parts[1])

            # 将时间戳和航向角存入数据列表
            data.append((timestamp, yaw))

    return data

def LoaddENData_DR(file_path):
    """
    从位置数据文件中加载数据，返回时间戳、E和N坐标。
    :param file_path: 文件路径
    :return: 包含时间戳、E和N坐标的数据列表 [(timestamp, E, N), ...]
    """
    data = []

    with open(file_path, 'r') as file:
        lines = file.readlines()

        # 跳过第一行
        for line in lines[1:]:
            parts = line.split(',')  # 按逗号分割每一行数据

            # 第一列是时间戳
            timestamp = float(parts[0])

            # 第二列和第三列分别是E和N坐标
            E = float(parts[1])  # E坐标
            N = float(parts[2])  # N坐标

            # 将时间戳和坐标存入数据列表
            data.append((timestamp, E, N))

    return data

def plotrawdata_DR(rawdata):
    """
    绘制加速度计、陀螺仪、磁强计的数据，分别在一张图的上、中、下三个位置显示。
    :param rawdata: 数据列表，每个元素是一个元组 (timestamp, gyro_data, accel_data, mag_data)
    """
    # 提取时间戳、陀螺仪、加速度计、磁强计的数据
    timestamps = [data[0] for data in rawdata]

    gyro_data = [data[1] for data in rawdata]  # 陀螺仪数据
    accel_data = [data[2] for data in rawdata]  # 加速度计数据
    mag_data = [data[3] for data in rawdata]  # 磁强计数据

    # 提取每个传感器的三轴数据
    gyro_x = [g[0] for g in gyro_data]
    gyro_y = [g[1] for g in gyro_data]
    gyro_z = [g[2] for g in gyro_data]

    accel_x = [a[0] for a in accel_data]
    accel_y = [a[1] for a in accel_data]
    accel_z = [a[2] for a in accel_data]

    mag_x = [m[0] * 10e-3 for m in mag_data]
    mag_y = [m[1] * 10e-3  for m in mag_data]
    mag_z = [m[2] * 10e-3  for m in mag_data]

    # 创建图形
    fig, axs = plt.subplots(3, 1, figsize=(12, 18))

    # 绘制陀螺仪数据（上图）
    axs[0].plot(timestamps, gyro_x, label="Gyro X", color="r")
    axs[0].plot(timestamps, gyro_y, label="Gyro Y", color="g")
    axs[0].plot(timestamps, gyro_z, label="Gyro Z", color="b")
    axs[0].set_title('Gyr')
    axs[0].set_ylabel('°/s')
    axs[0].legend(loc="best")
    axs[0].grid(True)

    # 绘制加速度计数据（中图）
    axs[1].plot(timestamps, accel_x, label="Accel X", color="r")
    axs[1].plot(timestamps, accel_y, label="Accel Y", color="g")
    axs[1].plot(timestamps, accel_z, label="Accel Z", color="b")
    axs[1].set_title('Acc')
    axs[1].set_ylabel('m/s²')
    axs[1].legend(loc="best")
    axs[1].grid(True)

    # 绘制磁强计数据（下图）
    axs[2].plot(timestamps, mag_x, label="Mag X", color="r")
    axs[2].plot(timestamps, mag_y, label="Mag Y", color="g")
    axs[2].plot(timestamps, mag_z, label="Mag Z", color="b")
    axs[2].set_title('Mag')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Gauss')
    axs[2].legend(loc="best")
    axs[2].grid(True)

    # 调整子图布局
    plt.tight_layout()

    # 显示图形
    plt.show()

def plotalign_DR(align):
    """
    绘制俯仰角和横滚角随时间变化的趋势图。
    :param align: 数据列表，每个元素是一个元组 (timestamp, pitch, roll, yaw)
    """
    # 提取时间戳、俯仰角和横滚角数据
    timestamps = [data[0] for data in align]
    pitch = [data[2] for data in align]
    roll = [data[3] for data in align]

    # 创建一个包含两个子图的图形
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    # 绘制俯仰角（Pitch）变化图
    axs[0].plot(timestamps, pitch, label='Pitch', color='b')
    axs[0].set_title('Pitch')
    axs[0].set_ylabel('Degree')
    axs[0].grid(True)
    axs[0].legend()

    # 绘制横滚角（Roll）变化图
    axs[1].plot(timestamps, roll, label='Roll', color='r')
    axs[1].set_title('Roll')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Degree')
    axs[1].grid(True)
    axs[1].legend()

    # 调整布局，避免子图重叠
    plt.tight_layout()

    # 显示图形
    plt.show()

def plotyaw_DR(gyr, mag):
    """
    绘制陀螺仪和磁强计计算出的航向角（Yaw）随时间的变化。
    :param gyr: 来自陀螺仪的数据列表，每个元素是 (timestamp, yaw, pitch, roll)
    :param mag: 来自磁强计的数据列表，每个元素是 (timestamp, yaw)
    """
    # 提取时间戳和航向角数据
    gyr_time = [data[0] for data in gyr]
    gyr_yaw = [data[1] for data in gyr]  # 陀螺仪的航向角

    mag_time = [data[0] for data in mag]
    mag_yaw = [data[1] for data in mag]  # 磁强计的航向角

    # 创建图形
    plt.figure(figsize=(10, 6))

    # 绘制陀螺仪航向角
    plt.plot(gyr_time, gyr_yaw, label='Gyro Yaw', color='b', linestyle='-', marker='o', markersize=4)

    # 绘制磁强计航向角
    plt.plot(mag_time, mag_yaw, label='Mag Yaw', color='r', linestyle='-', marker='x', markersize=4)

    # 添加标题和轴标签
    plt.title('Yaw Angle', fontsize=14)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Degrees', fontsize=12)

    # 显示图例
    plt.legend()

    # 显示网格
    plt.grid(True)

    # 显示图形
    plt.tight_layout()
    plt.show()


def plot_gravity_DR(data):
    # 提取时间和重力加速度数据
    time = [entry[0] for entry in data]
    gravity = [entry[4] for entry in data]

    # 创建图形
    plt.figure(figsize=(10, 6))

    # 绘制重力加速度随时间变化的曲线
    plt.plot(time, gravity, label='Gravity', color='blue')

    # 标记重力加速度小于等于9.2的点
    for i in range(len(gravity)):
        if gravity[i] <= 9.2:
            plt.scatter(time[i], gravity[i], color='red', marker='x', label='Gravity <= 9.2' if i == 0 else "")

    # 设置图形的标题和轴标签
    plt.xlabel('Time (s)')
    plt.ylabel('Gravity Magnitude (m/s^2)')

    # 添加图例
    plt.legend(loc='upper right')

    # 显示图形
    plt.grid(True)
    plt.show()

def plotdeadReckong_DR(den):
    """
    绘制使用陀螺仪计算的E、N轨迹图。
    :param den: 数据列表，每个元素是一个元组 (timestamp, E, N)，E为东向坐标，N为北向坐标
    """
    # 提取时间戳、E和N坐标
    time_stamps = [data[0] for data in den]
    E = [-data[1] for data in den]  # E坐标（东向）
    N = [data[2] for data in den]  # N坐标（北向）

    # 创建图形
    plt.figure(figsize=(10, 6))

    # 绘制轨迹图
    plt.plot(E, N, label='Trajectory', color='b', linestyle='-', marker='o', markersize=4)

    # 添加标题和轴标签
    plt.title('Trajectory', fontsize=14)
    plt.xlabel('N [m]', fontsize=12)
    plt.ylabel('E [m]', fontsize=12)

    # 显示图例
    plt.legend()

    # 显示网格
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')

    # 展示图像
    plt.tight_layout()
    plt.show()

def plotdenu_DR(gyr, mag):
    """
    绘制陀螺仪计算的 E 和 N，以及磁强计计算的 E 和 N 轨迹图。
    :param gyr: 由陀螺仪计算的数据，格式为 [(时间戳, E, N), ...]
    :param mag: 由磁强计计算的数据，格式为 [(时间戳, E, N), ...]
    """
    # 提取陀螺仪数据中的 E 和 N 坐标
    gyr_time = [data[0] for data in gyr]
    gyr_E = [data[1] for data in gyr]  # E坐标（东向）
    gyr_N = [data[2] for data in gyr]  # N坐标（北向）

    # 提取磁强计数据中的 E 和 N 坐标
    mag_time = [data[0] for data in mag]
    mag_E = [data[1] for data in mag]  # E坐标（东向）
    mag_N = [data[2] for data in mag]  # N坐标（北向）

    # 创建图形
    plt.figure(figsize=(10, 6))

    # 绘制陀螺仪的 E 和 N 轨迹
    plt.plot(gyr_E, gyr_N, label='Gyro', color='r', linestyle='-', marker='o', markersize=1)

    # 绘制磁强计的 E 和 N 轨迹
    plt.plot(mag_E, mag_N, label='Mag', color='b', linestyle='-', marker='x', markersize=1)

    # 设置坐标轴的刻度，以50米为一个格
    step = 50  # 设置每个刻度步长为50米
    plt.xticks(range(int(min(gyr_E + mag_E) // step) * step, int(max(gyr_E + mag_E) // step + 1) * step, step))
    plt.yticks(range(int(min(gyr_N + mag_N) // step) * step, int(max(gyr_N + mag_N) // step + 1) * step, step))

    # 添加标题和轴标签
    plt.title('Trajectory', fontsize=14)
    plt.xlabel('E [m]', fontsize=12)
    plt.ylabel('N [m]', fontsize=12)

    # 显示图例
    plt.legend()

    # 显示网格
    plt.grid(True)

    # 保证横纵坐标的比例相同
    plt.gca().set_aspect('equal', adjustable='box')

    # 展示图像
    plt.tight_layout()
    plt.show()
