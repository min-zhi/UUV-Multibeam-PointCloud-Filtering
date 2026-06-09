# UUV Multibeam Point Cloud Filtering System
## 保留微地形特征的UUV多波束点云滤波系统

### 📋 项目概述

本项目提供了一套完整的MATLAB工具包，用于对UUV（无人水下航行器）多波束声呐采集的点云进行高效滤波，**同时保留微地形特征**。

**核心目标：**
- ✅ 有效去除水下环境中的随机噪声和离群点
- ✅ 保留沙纹、岩石纹理等微地形特征
- ✅ 自适应参数调整，适应不同海床类型
- ✅ 高效处理大规模点云数据

---

### 🎯 核心算法

#### 1. **自适应形态学滤波** (Adaptive Morphological Filter)
- 使用开闭运算进行滤波
- 自动计算结构元素大小
- 有效保留边缘和微地形特征

#### 2. **多尺度高斯金字塔滤波** (Multi-scale Gaussian Pyramid)
- 在多个尺度上分离噪声和特征
- 重建时选择性保留特征成分
- 平衡平滑度和特征保留

#### 3. **统计离群点检测** (Statistical Outlier Removal)
- 基于邻域统计的离群点检测
- 自适应阈值计算
- 有效去除孤立噪声点

#### 4. **双边滤波** (Bilateral Filter)
- 边缘感知的平滑滤波
- 保留尖锐的地形边界
- 适合微地形特征保留

---

### 📁 项目结构

```
UUV-Multibeam-PointCloud-Filtering/
├── README.md
├── LICENSE
├── config/
│   └── filter_config.m
├── algorithms/
│   ├── adaptiveMorphologicalFilter.m
│   ├── multiScaleGaussianFilter.m
│   ├── statisticalOutlierRemoval.m
│   └── bilateralFilter.m
├── core/
│   ├── PointCloudProcessor.m
│   ├── readPointCloud.m
│   └── visualizePointCloud.m
├── examples/
│   └── example_basic_filtering.m
│   └── generate_synthetic_data.m
└── utils/
    └── computeMetrics.m
```

---

### 🚀 快速开始

```matlab
% 生成合成点云
pc = generate_synthetic_data('nPoints', 50000, 'terrain', 'sandy');

% 创建处理器
processor = PointCloudProcessor(pc);

% 应用自适应形态学滤波
processor.adaptiveMorphFilter('SE_SIZE', 3, 'preserveFeatures', true);

% 可视化结果
processor.compareOriginal();
```

---

### 💻 系统要求

- **MATLAB**: R2020b 或更新版本
- **工具箱**: Image Processing, Computer Vision, Signal Processing
- **内存**: 4GB+ RAM

---

### 📊 算法对比

| 算法 | 噪声去除 | 特征保留 | 计算速度 | 适用场景 |
|------|---------|---------|--------|----------| 
| 自适应形态学 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | 微地形丰富 |
| 多尺度高斯 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ | 大规模点云 |
| 统计离群点 | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | 孤立噪声 |
| 双边滤波 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐ | 边缘保留 |

---

### 📝 示例

运行 `example_basic_filtering.m` 查看完整示例

---

### 📄 许可证

MIT License - 详见 LICENSE 文件
