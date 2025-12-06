# 室内定位技术项目

本项目实现了三种室内定位方法：几何距离定位（UWB）、航迹推算（IMU）和数据库匹配（信号强度指纹识别）。

## 📋 目录

- [项目结构](#项目结构)
  - [整体目录结构](#整体目录结构)
  - [代码模块详细说明](#代码模块详细说明)
  - [数据目录结构](#数据目录结构)
  - [结果输出目录](#结果输出目录)
- [环境要求](#环境要求)
- [安装依赖](#安装依赖)
- [使用方法](#使用方法)
- [数据格式说明](#数据格式说明)
- [算法原理](#算法原理)
- [输出结果](#输出结果)
- [使用注意事项](#使用注意事项)

## 📁 项目结构

### 整体目录结构

```
.
├── 代码/                              # 核心代码目录
│   ├── main.py                       # 主程序入口，统一调用三种定位方法
│   ├── Geometry_Distance_Car.py      # 几何距离定位模块（Python实现）
│   ├── Dead_Reckoning.py             # 航迹推算模块（Python实现）
│   ├── Database_Matching.py          # 数据库匹配模块（Python实现）
│   ├── Dead_Reckoning/               # 航迹推算C++实现目录
│   │   ├── main.cpp                  # C++主程序入口
│   │   ├── DeadReckoning.cpp         # 航迹推算核心算法实现
│   │   ├── Align.cpp                 # 姿态解算功能
│   │   ├── CoordinateChange.cpp      # 坐标转换功能
│   │   ├── ReadFile.cpp              # 文件读取功能
│   │   ├── DR_Structs.h              # 数据结构定义头文件
│   │   ├── Data/                     # 输入数据目录
│   │   │   ├── sensors_matrix.txt   # 原始传感器数据
│   │   │   └── pos_reference.txt    # 参考位置数据
│   │   └── Result/                  # 输出结果目录
│   │       ├── AlignEpoch.txt        # 按历元对齐的姿态数据
│   │       ├── AlignSecond.txt       # 按秒对齐的姿态数据
│   │       ├── YawSecond.txt        # 航向角数据
│   │       ├── GyrDeadReckoning.txt # 基于陀螺仪的航迹推算结果
│   │       ├── MagDeadReckoning.txt # 基于磁强计的航迹推算结果
│   │       └── RefdENU.txt          # 参考位置数据（ENU坐标系）
│   ├── 数据-距离交会/                 # 几何定位实验数据
│   │   └── 数据-小车-UWB/
│   │       ├── base-station-location.txt  # 基站位置坐标
│   │       ├── range_tag_2.txt            # 流动站2测距数据
│   │       ├── range_tag_3.txt            # 流动站3测距数据
│   │       ├── true_position_tag_2.txt    # 流动站2参考真值
│   │       └── true_position_tag_3.txt     # 流动站3参考真值
│   ├── 数据-行人航迹推算/             # 航迹推算实验数据
│   │   ├── sensors_matrix.txt       # 原始IMU传感器数据
│   │   └── pos_reference.txt        # 参考位置数据
│   └── 数据-数据库匹配/                # 数据库匹配实验数据
│       └── 数据-信号强度指纹识别/
│           ├── database.data        # 指纹数据库
│           ├── rp_loc.txt          # 参考点位置坐标
│           └── test1_with_ref.data # 测试数据（含真值）
├── Figures/                           # 可视化结果图表
│   ├── 几何定位/                      # 几何定位可视化图表
│   ├── 航迹推算/                      # 航迹推算可视化图表
│   └── 数据库匹配/                    # 数据库匹配可视化图表
├── 结课报告/                           # 实验报告文档
│   ├── 实验报告.pdf
│   └── 图例.pptx
└── README.md                          # 项目说明文档
```

### 代码模块详细说明

#### 1. 主程序模块 (`main.py`)

**功能**：统一入口，提供交互式菜单选择三种定位方法

**结构**：
- 导入三个定位模块的函数
- `main()` 函数：显示菜单，根据用户选择调用对应模块
- 三种定位模式：
  - 模式1：几何距离定位（调用 `Geometry_Distance_Car.py`）
  - 模式2：航迹推算（调用 `Dead_Reckoning.py`）
  - 模式3：数据库匹配（调用 `Database_Matching.py`）

#### 2. 几何距离定位模块 (`Geometry_Distance_Car.py`)

**功能**：基于UWB基站测距的三维定位

**主要函数**：
- **数据加载**：
  - `LoadBaseData_Geo()` - 加载基站位置数据
  - `LoadRoverData_Geo()` - 加载流动站测距数据
  - `LoadReferenceData_Geo()` - 加载参考真值数据
- **定位计算**：
  - `GeometryLocation()` - 使用非线性最小二乘法进行位置解算
  - `calculate_dop()` - 计算DOP值（HDOP、VDOP、GDOP）
- **可视化**：
  - `plot_3d_trajectory()` - 三维轨迹图
  - `plot_2d_trajectory()` - 二维轨迹图
  - `plot_error_comparison()` - 误差分析图
  - `plot_dop_values()` - DOP值变化图

**数据流向**：
```
基站位置文件 → LoadBaseData_Geo() → 基站字典
测距数据文件 → LoadRoverData_Geo() → 测距列表
                    ↓
            GeometryLocation() → 位置解算结果
                    ↓
            calculate_dop() → DOP值
                    ↓
            可视化函数 → 图表输出
```

#### 3. 航迹推算模块 (`Dead_Reckoning.py`)

**功能**：基于IMU传感器的行人航迹推算

**主要函数**：
- **数据加载**：
  - `LoadRawData_DR()` - 加载原始传感器数据（陀螺仪、加速度计、磁强计）
  - `LoadAlignData_DR()` - 加载姿态角数据（俯仰角、横滚角、航向角）
  - `LoadYawData_DR()` - 加载航向角数据
  - `LoaddENData_DR()` - 加载位置数据（E、N坐标）
- **可视化**：
  - `plotrawdata_DR()` - 原始传感器数据可视化
  - `plotalign_DR()` - 姿态角变化图
  - `plotyaw_DR()` - 航向角对比图（陀螺仪 vs 磁强计）
  - `plot_gravity_DR()` - 重力加速度变化图
  - `plotdeadReckong_DR()` - 航迹推算轨迹图
  - `plotdenu_DR()` - 陀螺仪与磁强计轨迹对比图

**数据流向**：
```
原始传感器数据 → LoadRawData_DR() → 原始数据列表
C++处理结果文件 → LoadAlignData_DR() / LoadYawData_DR() / LoaddENData_DR() → 处理结果
                    ↓
            可视化函数 → 图表输出
```

**说明**：本模块主要负责数据加载和可视化，核心算法在C++模块中实现。

#### 4. 航迹推算C++模块 (`Dead_Reckoning/`)

**功能**：航迹推算核心算法实现（C++版本）

**文件结构**：
- `main.cpp` - 主程序入口
- `DR_Structs.h` - 数据结构定义
- `ReadFile.cpp` - 文件读取功能
- `Align.cpp` - 姿态解算功能
- `CoordinateChange.cpp` - 坐标转换功能
- `DeadReckoning.cpp` - 航迹推算核心算法
- `Data/` - 输入数据目录
  - `sensors_matrix.txt` - 原始传感器数据
  - `pos_reference.txt` - 参考位置数据
- `Result/` - 输出结果目录
  - `AlignEpoch.txt` - 按历元对齐的姿态数据
  - `AlignSecond.txt` - 按秒对齐的姿态数据
  - `YawSecond.txt` - 航向角数据
  - `GyrDeadReckoning.txt` - 基于陀螺仪的航迹推算结果
  - `MagDeadReckoning.txt` - 基于磁强计的航迹推算结果
  - `RefdENU.txt` - 参考位置数据（ENU坐标系）

**处理流程**：
```
原始传感器数据 → 姿态解算 → 航向角计算 → 步态检测 → 位置更新 → 结果输出
```

#### 5. 数据库匹配模块 (`Database_Matching.py`)

**功能**：基于信号强度指纹识别的定位方法

**主要函数**：
- **数据加载**：
  - `LoadDatabase_DB()` - 加载指纹数据库（参考点信号强度）
  - `LoadRefPointLocation_DB()` - 加载参考点位置坐标
  - `LoadTestRef_DB()` - 加载测试数据（包含真值）
- **数据处理**：
  - `smooth_signal_strength()` - 信号强度平滑处理（处理缺失值-110）
- **定位算法**：
  - `euclidean_distance()` - 计算信号强度欧式距离
  - `knn_location()` - KNN定位算法实现
- **可视化**：
  - `plot_trajectory()` - 轨迹对比图
  - `plot_error_comparison_DB()` - 误差分析图

**数据流向**：
```
指纹数据库 → LoadDatabase_DB() → 数据库字典
参考点位置 → LoadRefPointLocation_DB() → 位置字典
测试数据 → LoadTestRef_DB() → 测试数据列表
                    ↓
        smooth_signal_strength() → 平滑后的测试数据
                    ↓
            knn_location() → 位置估计结果
                    ↓
            可视化函数 → 图表输出
```

**算法流程**：
```
测试信号强度 → 与数据库计算距离 → 选择K个最近邻 → 加权平均 → 位置估计
```

### 数据目录结构

#### `数据-距离交会/数据-小车-UWB/`
- `base-station-location.txt` - 基站位置坐标
- `range_tag_2.txt` / `range_tag_3.txt` - 流动站测距数据
- `true_position_tag_2.txt` / `true_position_tag_3.txt` - 参考真值位置

#### `数据-行人航迹推算/`
- `sensors_matrix.txt` - 原始IMU传感器数据
- `pos_reference.txt` - 参考位置数据

#### `数据-数据库匹配/数据-信号强度指纹识别/`
- `database.data` - 指纹数据库（参考点信号强度）
- `rp_loc.txt` - 参考点位置坐标
- `test1_with_ref.data` - 测试数据（含真值）

### 结果输出目录

#### `Figures/`
- `几何定位/` - 几何定位的可视化图表
- `航迹推算/` - 航迹推算的可视化图表
- `数据库匹配/` - 数据库匹配的可视化图表

## 🔧 环境要求

- **Python**: 3.7+
- **依赖库**:
  - NumPy - 数值计算
  - Matplotlib - 数据可视化
  - SciPy - 科学计算（用于非线性优化）

## 📦 安装依赖

```bash
pip install numpy matplotlib scipy
```

## 🚀 使用方法

### 运行主程序

```bash
cd 代码
python main.py
```

程序会显示交互式菜单，选择对应的定位方法：
- 输入 `1` - 几何距离定位（无人车）
- 输入 `2` - 航迹推算
- 输入 `3` - 数据库匹配

### 模块调用方式

#### 几何距离定位模块

```python
from Geometry_Distance_Car import (
    LoadBaseData_Geo,      # 加载基站数据
    LoadRoverData_Geo,     # 加载流动站数据
    LoadReferenceData_Geo, # 加载参考数据
    GeometryLocation,      # 定位计算
    calculate_dop          # DOP值计算
)

# 1. 加载数据
basestations = LoadBaseData_Geo("数据-距离交会/数据-小车-UWB/base-station-location.txt")
rover = LoadRoverData_Geo("数据-距离交会/数据-小车-UWB/range_tag_2.txt")
reference = LoadReferenceData_Geo("数据-距离交会/数据-小车-UWB/true_position_tag_2.txt")

# 2. 进行定位计算
results = GeometryLocation(rover, basestations)  # 返回 [(timestamp, x, y, z), ...]

# 3. 计算DOP值（可选）
for timestamp, x, y, z in results:
    hdop, vdop, gdop = calculate_dop(basestations, (x, y, z))
```

#### 航迹推算模块

```python
from Dead_Reckoning import (
    LoadRawData_DR,        # 加载原始传感器数据
    LoadAlignData_DR,      # 加载姿态数据
    LoadYawData_DR,        # 加载航向角数据
    LoaddENData_DR         # 加载位置数据
)

# 加载原始传感器数据
rawdata = LoadRawData_DR("数据-行人航迹推算/sensors_matrix.txt")
# 返回格式: [(timestamp, [gyro_x, gyro_y, gyro_z], [acc_x, acc_y, acc_z], [mag_x, mag_y, mag_z]), ...]

# 加载C++处理后的结果（需要先运行C++程序）
align_data = LoadAlignData_DR("Dead_Reckoning/Result/AlignSecond.txt", mode=1)
# mode=0: 包含重力加速度数据
# mode=1: 仅包含姿态角数据
```

**注意**：航迹推算的核心算法在C++模块中实现，Python模块主要用于数据加载和可视化。需要先编译运行C++程序生成结果文件。

#### 数据库匹配模块

```python
from Database_Matching import (
    LoadDatabase_DB,           # 加载指纹数据库
    LoadRefPointLocation_DB,   # 加载参考点位置
    LoadTestRef_DB,            # 加载测试数据
    smooth_signal_strength,    # 信号平滑
    knn_location              # KNN定位
)

# 1. 加载数据
database = LoadDatabase_DB("数据-数据库匹配/数据-信号强度指纹识别/database.data")
# 返回格式: {参考点号: [信号强度1, 信号强度2, ..., 信号强度30]}

ref_points = LoadRefPointLocation_DB("数据-数据库匹配/数据-信号强度指纹识别/rp_loc.txt")
# 返回格式: {参考点号: (E, N, D)}

test_data = LoadTestRef_DB("数据-数据库匹配/数据-信号强度指纹识别/test1_with_ref.data")
# 返回格式: [(timestamp, [信号强度列表], (E, N, D)), ...]

# 2. 信号平滑（可选）
smoothed_data = smooth_signal_strength(test_data)

# 3. KNN定位
results = knn_location(database, ref_points, test_data, k=3)
# 返回格式: [(timestamp, (E, N, D)), ...]
```

## 📊 数据格式说明

### 几何距离定位数据格式

#### 基站位置文件 (`base-station-location.txt`)
**格式**：空格分隔，每行4列
```
基站编号 X坐标 Y坐标 Z坐标
1 0.0 0.0 0.0
2 10.0 0.0 0.0
...
```
**说明**：基站编号从1开始，坐标为三维坐标（单位：米）

#### 流动站测距文件 (`range_tag_*.txt`)
**格式**：空格分隔，第一列为时间戳，后续列为各基站距离
```
时间戳 基站1距离 基站2距离 ... 基站8距离
1000 5.2 6.1 7.3 8.4 9.5 10.6 11.7 12.8
```
**说明**：距离单位为米，基站顺序与基站位置文件中的编号对应

#### 参考位置文件 (`true_position_tag_*.txt`)
**格式**：空格分隔，每行4列
```
时间戳 X坐标 Y坐标 Z坐标
1000 1.5 2.3 0.5
```
**说明**：用于对比验证定位精度

### 航迹推算数据格式

#### 原始传感器数据文件 (`sensors_matrix.txt`)
**格式**：逗号分隔，每行10列
```
时间戳,陀螺仪X,陀螺仪Y,陀螺仪Z,加速度计X,加速度计Y,加速度计Z,磁强计X,磁强计Y,磁强计Z
0.0,0.1,0.2,0.3,0.0,0.0,9.8,0.0,0.0,0.0
```
**单位**：
- 陀螺仪：度/秒 (°/s)
- 加速度计：米/秒² (m/s²)
- 磁强计：需要乘以 10e-3 转换为高斯 (Gauss)

#### C++处理结果文件格式

**姿态数据文件** (`AlignEpoch.txt`, `AlignSecond.txt`)
**格式**：逗号分隔，第一行为标题行
```
时间戳,俯仰角,航向角,横滚角[,重力加速度]
0.0,10.5,45.2,5.3,9.8
```
**说明**：`AlignEpoch.txt` 包含重力加速度（5列），`AlignSecond.txt` 不包含（4列）

**航向角文件** (`YawSecond.txt`)
**格式**：逗号分隔，第一行为标题行
```
时间戳,航向角
0.0,45.2
```

**位置数据文件** (`GyrDeadReckoning.txt`, `MagDeadReckoning.txt`, `RefdENU.txt`)
**格式**：逗号分隔，第一行为标题行
```
时间戳,E坐标,N坐标
0.0,10.5,20.3
```
**说明**：E为东向坐标，N为北向坐标（单位：米）

### 数据库匹配数据格式

#### 指纹数据库文件 (`database.data`)
**格式**：空格分隔，第一列为参考点号，后续30列为各基站信号强度
```
参考点号 基站1信号强度 基站2信号强度 ... 基站30信号强度
1 -50 -60 -70 -80 ... -110
```
**说明**：
- 信号强度单位为 dBm
- `-110` 表示该基站信号缺失
- 同一参考点可能有多条记录，程序会自动求平均

#### 参考点位置文件 (`rp_loc.txt`)
**格式**：空格分隔，每行4列
```
参考点号 E坐标 N坐标 D坐标
1 10.5 20.3 0.0
```
**说明**：E为东向，N为北向，D为高程（单位：米）

#### 测试数据文件 (`test1_with_ref.data`)
**格式**：空格分隔，第一列为时间戳，2-31列为信号强度，32-34列为参考位置
```
时间戳 基站1信号强度 ... 基站30信号强度 E坐标 N坐标 D坐标
1000.0 -50 -60 ... -70 10.5 20.3 0.0
```
**说明**：包含30个基站的信号强度和参考真值位置

## 🔬 算法原理

### 几何距离定位

**核心算法**：非线性最小二乘法

通过最小化测距误差的平方和来求解目标位置：

```
minimize: Σ(√((x-xi)² + (y-yi)² + (z-zi)²) - di)²
```

其中：
- `(xi, yi, zi)` - 第i个基站的已知位置
- `di` - 第i个基站的测距值
- `(x, y, z)` - 待求的目标位置

**实现**：使用 `scipy.optimize.least_squares` 进行优化求解

**DOP值计算**：
- 通过Jacobian矩阵计算协方差矩阵
- HDOP：水平精度因子
- VDOP：垂直精度因子
- GDOP：几何精度因子

### 航迹推算

**处理流程**（C++实现）：

1. **数据读取与预处理**
   - 读取IMU传感器原始数据
   - 时间对齐与数据同步

2. **姿态解算** (`Align.cpp`)
   - 使用加速度计和磁强计计算初始姿态
   - 计算俯仰角（Pitch）、横滚角（Roll）、航向角（Yaw）

3. **航向角融合** (`DeadReckoning.cpp`)
   - 陀螺仪航向角：通过积分角速度得到
   - 磁强计航向角：直接计算得到
   - 两种方法可分别用于航迹推算

4. **步态检测**
   - 基于重力加速度变化检测步态
   - 重力加速度 ≤ 9.2 m/s² 时判定为步态

5. **位置更新**
   - 根据步长和航向角更新位置
   - 坐标转换（`CoordinateChange.cpp`）

**Python模块作用**：数据加载、结果可视化，不包含核心算法

### 数据库匹配

**核心算法**：K近邻（KNN）定位

**处理流程**：

1. **数据库构建**
   - 加载参考点信号强度指纹
   - 对同一参考点的多次测量求平均
   - 处理缺失信号（-110）用平均值填充

2. **信号平滑** (`smooth_signal_strength()`)
   - 对测试数据中的缺失信号进行插值
   - 使用前后各2个历元的有效信号进行平滑

3. **距离计算** (`euclidean_distance()`)
   - 计算测试信号与数据库信号的欧式距离
   - 仅考虑有效信号（非-110）
   - 距离标准化：除以有效基站数量

4. **KNN定位** (`knn_location()`)
   - 选择K个距离最近的参考点
   - 加权平均计算位置估计
   - 权重 = 1 / (距离 + ε)，距离越小权重越大

**参数调优**：支持K值从1到6的对比测试

## 📈 输出结果

### 几何距离定位
- 位置解算结果：`[(timestamp, x, y, z), ...]`
- DOP值：HDOP、VDOP、GDOP
- 可视化图表：轨迹对比、误差分析、DOP变化

### 航迹推算
- C++输出文件：姿态数据、航向角、位置数据
- Python可视化：传感器数据、姿态角、轨迹对比

### 数据库匹配
- 位置估计结果：`[(timestamp, (E, N, D)), ...]`
- 精度指标：RMSE、STD（标准差）
- 可视化图表：轨迹对比、误差分析

## 📝 使用注意事项

1. **数据路径**：确保数据文件路径正确，相对路径基于 `代码/` 目录
2. **数据格式**：严格按照数据格式说明准备数据文件
3. **依赖安装**：运行前确保已安装所有Python依赖包
4. **C++编译**：航迹推算需要先编译运行C++程序生成结果文件
5. **文件编码**：数据文件建议使用UTF-8编码
6. **可视化**：图表会在新窗口中显示，关闭窗口可继续程序执行

## 📄 项目信息

本项目为室内定位技术课程作业，实现了三种典型的室内定位方法。

**技术栈**：
- Python 3.7+ - 主要开发语言
- C++ - 航迹推算核心算法
- NumPy / SciPy - 数值计算
- Matplotlib - 数据可视化

---

如有问题或建议，欢迎提出Issue。

