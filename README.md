# 室内定位技术实验

本仓库整理了室内定位技术课程中的三类定位方法：UWB 几何距离交会、IMU 行人航迹推算和 WiFi/信号强度指纹数据库匹配。项目以 Python 为统一入口，同时在航迹推算部分保留 C++ 核心算法实现、实验数据、可视化图表和结课报告。

## 快速使用

Python 入口位于 `代码/main.py`：

```powershell
cd .\代码
python .\main.py
```

程序提供三种模式：

```text
1  几何距离定位（UWB 小车）
2  航迹推算（IMU 行人）
3  数据库匹配（信号强度指纹）
```

Python 依赖：

```powershell
python -m pip install numpy scipy matplotlib
```

说明：仓库已包含大量 SVG/PDF 结果图。若只查看实验结果，可直接浏览 `代码/Figures/` 和 `结课报告/实验报告.pdf`。

## 主要功能

- 几何距离定位：读取 UWB 基站坐标和流动站测距数据，使用非线性最小二乘解算三维位置，并输出 2D/3D 轨迹、误差和 DOP 图。
- 航迹推算：C++ 实现姿态解算、航向角计算、步态检测和位置递推；Python 负责读取结果并绘图。
- 数据库匹配：读取 30 个基站的信号强度指纹，处理 `-110` 缺失值，使用 KNN 进行位置估计，并比较平滑/未平滑结果。
- 结果可视化：按几何定位、航迹推算、数据库匹配三类保存 SVG 图表。

## 项目结构

```text
.
├─ 代码/
│  ├─ main.py                         # Python 统一入口
│  ├─ Geometry_Distance_Car.py        # UWB 几何距离定位
│  ├─ Dead_Reckoning.py               # 航迹推算结果读取与绘图
│  ├─ Database_Matching.py            # 指纹数据库匹配定位
│  ├─ Dead_Reckoning/                 # 航迹推算 C++ 实现
│  │  ├─ main.cpp
│  │  ├─ Align.cpp
│  │  ├─ CoordinateChange.cpp
│  │  ├─ DeadReckoning.cpp
│  │  ├─ ReadFile.cpp
│  │  ├─ DR_Structs.h
│  │  ├─ Data/
│  │  └─ Result/
│  ├─ 数据-距离交会/                  # UWB 测距与真值数据
│  ├─ 数据-行人航迹推算/              # IMU 与参考位置数据
│  ├─ 数据-数据库匹配/                # 指纹数据库、参考点和测试数据
│  └─ Figures/                        # 三类方法的结果图
├─ 结课报告/
│  ├─ 实验报告.pdf
│  └─ 图例.pptx
└─ README.md
```

## 代码模块

| 文件 | 说明 |
| --- | --- |
| `Geometry_Distance_Car.py` | `LoadBaseData_Geo`、`GeometryLocation`、`calculate_dop` 和几何定位绘图 |
| `Database_Matching.py` | 指纹数据库平均、信号平滑、欧式距离、KNN 定位和误差图 |
| `Dead_Reckoning.py` | 读取 C++ 航迹推算输出，绘制姿态、航向、重力和轨迹 |
| `Dead_Reckoning/*.cpp` | 姿态解算、磁航向/陀螺航向航迹推算、参考轨迹转换 |
| `main.py` | 三种定位方法的交互式调度入口 |

## 数据与结果

几何距离定位：

```text
代码/数据-距离交会/数据-小车-UWB/
├─ base-station-location.txt
├─ range_tag_2.txt
├─ range_tag_3.txt
├─ true_position_tag_2.txt
└─ true_position_tag_3.txt
```

航迹推算：

```text
代码/Dead_Reckoning/Data/
├─ sensors_matrix.txt
└─ pos_reference.txt

代码/Dead_Reckoning/Result/
├─ AlignEpoch.txt
├─ AlignSecond.txt
├─ YawSecond.txt
├─ GyrDeadReckoning.txt
├─ MagDeadReckoning.txt
└─ RefdENU.txt
```

数据库匹配：

```text
代码/数据-数据库匹配/数据-信号强度指纹识别/
├─ database.data
├─ rp_loc.txt
└─ test1_with_ref.data
```

结果图：

```text
代码/Figures/几何定位/
代码/Figures/航迹推算/
代码/Figures/数据库匹配/
```

## 航迹推算 C++ 复算

如果需要重新生成航迹推算中间结果，可进入 C++ 目录编译：

```powershell
cd .\代码\Dead_Reckoning
g++ -std=c++11 .\main.cpp .\Align.cpp .\CoordinateChange.cpp .\DeadReckoning.cpp .\ReadFile.cpp -o dead_reckoning.exe
.\dead_reckoning.exe
```

运行后结果写入 `代码/Dead_Reckoning/Result/`。

## 注意事项

- `main.py` 中的航迹推算绘图依赖 C++ 已生成结果；如果本地路径改动，需要同步检查脚本中的文件路径。
- 图表默认通过 Matplotlib 弹窗展示；在无图形界面的环境中可改为保存图片。
- 项目中包含实验数据和图表，仓库体积相对普通纯代码仓库更大。

## 报告

```text
结课报告/实验报告.pdf
```

## 作者

GYH-WHU
