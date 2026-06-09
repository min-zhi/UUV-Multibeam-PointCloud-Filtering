% ============================================================
% Example: Basic Point Cloud Filtering
% ============================================================
% 基础滤波示例
% 演示如何使用各种滤波算法
% ============================================================

clear; close all; clc;

fprintf('\\n====================================\\n');
fprintf('Basic Point Cloud Filtering Example\\n');
fprintf('====================================\\n\\n');

% 1. 生成合成点云（带噪声）
fprintf('Step 1: Generating synthetic point cloud...\\n');
pc = generate_synthetic_data('nPoints', 50000, 'noiseLevel', 0.05, 'terrain', 'sandy');

fprintf('Point cloud generated: %d points\\n\\n', pc.Count);

% 2. 创建处理器
fprintf('Step 2: Creating point cloud processor...\\n');
processor = PointCloudProcessor(pc);
processor.displayInfo();

% 3. 应用不同的滤波算法
fprintf('\\nStep 3: Applying different filters...\\n\\n');

% 3.1 统计离群点去除
fprintf('--- Filter 1: Statistical Outlier Removal ---\\n');
processor_stat = PointCloudProcessor(pc);
processor_stat.statisticalOutlierRemove('numNeighbors', 20, 'stdRatio', 2.0);
stat_points = processor_stat.pointCloud.Count;

% 3.2 自适应形态学滤波
fprintf('\\n--- Filter 2: Adaptive Morphological Filter ---\\n');
processor_morph = PointCloudProcessor(pc);
processor_morph.adaptiveMorphFilter('SE_SIZE', 3, 'preserveFeatures', true);
morph_points = processor_morph.pointCloud.Count;

% 3.3 多尺度高斯滤波
fprintf('\\n--- Filter 3: Multi-Scale Gaussian Filter ---\\n');
processor_gauss = PointCloudProcessor(pc);
processor_gauss.multiScaleGaussFilter('numScales', 3, 'baseSigma', 1.0);
gauss_points = processor_gauss.pointCloud.Count;

% 4. 对比结果
fprintf('\\n====================================\\n');
fprintf('Filter Comparison Results:\\n');
fprintf('====================================\\n');
fprintf('Original points:           %6d\\n', pc.Count);
fprintf('After Statistical filter: %6d (%.2f%% retained)\\n', ...
        stat_points, stat_points/pc.Count*100);
fprintf('After Morphological filter:%6d (%.2f%% retained)\\n', ...
        morph_points, morph_points/pc.Count*100);
fprintf('After Gaussian filter:    %6d (%.2f%% retained)\\n', ...
        gauss_points, gauss_points/pc.Count*100);

% 5. 可视化对比
fprintf('\\nStep 4: Visualizing results...\\n');

figure('Position', [100, 100, 1400, 900]);

% 原始点云
subplot(2, 2, 1);
pcshow(pc);
title(sprintf('Original Point Cloud (%d points)', pc.Count));
grid on;

% 统计离群点去除
subplot(2, 2, 2);
pcshow(processor_stat.pointCloud);
title(sprintf('Statistical Outlier Removal (%d points)', stat_points));
grid on;

% 自适应形态学滤波
subplot(2, 2, 3);
pcshow(processor_morph.pointCloud);
title(sprintf('Adaptive Morphological Filter (%d points)', morph_points));
grid on;

% 多尺度高斯滤波
subplot(2, 2, 4);
pcshow(processor_gauss.pointCloud);
title(sprintf('Multi-Scale Gaussian Filter (%d points)', gauss_points));
grid on;

fprintf('\\nVisualization complete!\\n');

% 6. 性能统计
fprintf('\\n====================================\\n');
fprintf('Performance Statistics:\\n');
fprintf('====================================\\n');

if isa(pc.Location, 'double')
    points_orig = pc.Location;
else
    points_orig = double(pc.Location);
end

if isa(processor_morph.pointCloud.Location, 'double')
    points_filt = processor_morph.pointCloud.Location;
else
    points_filt = double(processor_morph.pointCloud.Location);
end

% 计算点云覆盖范围
range_orig = max(points_orig) - min(points_orig);
range_filt = max(points_filt) - min(points_filt);

fprintf('\\nOriginal point cloud range:\\n');
fprintf('  X: %.4f, Y: %.4f, Z: %.4f\\n', range_orig(1), range_orig(2), range_orig(3));
fprintf('\\nFiltered point cloud range:\\n');
fprintf('  X: %.4f, Y: %.4f, Z: %.4f\\n', range_filt(1), range_filt(2), range_filt(3));

fprintf('\\n====================================\\n');
fprintf('Example completed successfully!\\n');
fprintf('====================================\\n\\n');
