% ============================================================
% Statistical Outlier Removal for Point Cloud
% ============================================================
% 统计离群点检测和去除
% 基于邻域距离统计的健壮离群点检测
% ============================================================

function [pc_filtered, outlier_mask] = statisticalOutlierRemoval(pc, varargin)
    % 统计离群点去除
    %
    % 用法:
    %   pc_filtered = statisticalOutlierRemoval(pc)
    %   [pc_filtered, mask] = statisticalOutlierRemoval(pc, 'stdRatio', 2.0)
    %
    % 输入:
    %   pc              - pointCloud 对象或 Nx3 矩阵
    %   varargin        - 可选参数对
    %
    % 输出:
    %   pc_filtered     - 滤波后的点云
    %   outlier_mask    - 布尔掩码
    
    % 解析输入参数
    p = inputParser;
    addParameter(p, 'numNeighbors', 20, @isnumeric);
    addParameter(p, 'stdRatio', 2.0, @isnumeric);
    addParameter(p, 'useKNN', true, @islogical);
    addParameter(p, 'verbose', true, @islogical);
    parse(p, varargin{:});
    
    num_neighbors = p.Results.numNeighbors;
    std_ratio = p.Results.stdRatio;
    use_knn = p.Results.useKNN;
    verbose = p.Results.verbose;
    
    if verbose
        fprintf('\\n=== Statistical Outlier Removal ===\\n');
        fprintf('Neighbors: %d\\n', num_neighbors);
        fprintf('Std Ratio: %.2f\\n', std_ratio);
    end
    
    % 转换为矩阵形式
    if isa(pc, 'pointCloud')
        points = pc.Location;
    else
        points = pc;
    end
    
    [num_points, ~] = size(points);
    
    if verbose
        fprintf('Processing %d points...\\n', num_points);
    end
    
    tic;
    
    % 1. 建立KD树
    kdtree = KDTreeSearcher(points);
    
    % 2. 计算邻域距离统计
    mean_distances = zeros(num_points, 1);
    
    if use_knn
        % 使用K最近邻
        for i = 1:num_points
            [~, distances] = knnsearch(kdtree, points(i, :), 'k', num_neighbors+1);
            % 排除自己（第一个）
            distances = distances(2:end);
            mean_distances(i) = mean(distances);
        end
    else
        % 使用固定半径
        radius = prctile(pdist(points), 25) / 2;
        
        for i = 1:num_points
            [idx, distances] = rangesearch(kdtree, points(i, :), radius);
            idx = idx{1};
            distances = distances{1}';
            
            if length(distances) > 1
                distances = distances(distances > 0);
                if ~isempty(distances)
                    mean_distances(i) = mean(distances);
                else
                    mean_distances(i) = radius;
                end
            else
                mean_distances(i) = radius;
            end
        end
    end
    
    % 3. 全局统计
    global_mean = mean(mean_distances);
    global_std = std(mean_distances);
    
    if verbose
        fprintf('\\nGlobal Statistics:\\n');
        fprintf('  Mean distance: %.6f\\n', global_mean);
        fprintf('  Std deviation: %.6f\\n', global_std);
    end
    
    % 4. 检测离群点
    threshold = global_mean + std_ratio * global_std;
    
    outlier_mask = mean_distances > threshold;
    num_outliers = sum(outlier_mask);
    outlier_ratio = num_outliers / num_points * 100;
    
    if verbose
        fprintf('\\nThreshold: %.6f\\n', threshold);
        fprintf('Outliers detected: %d (%.2f%%)\\n', num_outliers, outlier_ratio);
    end
    
    % 5. 创建输出
    points_filtered = points(~outlier_mask, :);
    
    if isa(pc, 'pointCloud')
        pc_filtered = pointCloud(points_filtered, 'Intensity', pc.Intensity(~outlier_mask));
    else
        pc_filtered = points_filtered;
    end
    
    elapsed_time = toc;
    
    if verbose
        fprintf('Filtering completed in %.2f seconds\\n', elapsed_time);
        fprintf('Output: %d points (%.2f%% retained)\\n\\n', ...
                size(points_filtered, 1), size(points_filtered, 1)/num_points*100);
    end
    
end
