% ============================================================
% Adaptive Morphological Filter for Point Cloud
% ============================================================
% 自适应形态学滤波算法
% 使用开闭运算保留微地形特征，有效去除噪声
% ============================================================

function pc_filtered = adaptiveMorphologicalFilter(pc, varargin)
    % 自适应形态学滤波
    %
    % 用法:
    %   pc_filtered = adaptiveMorphologicalFilter(pc)
    %   pc_filtered = adaptiveMorphologicalFilter(pc, 'SE_SIZE', 3)
    %   pc_filtered = adaptiveMorphologicalFilter(pc, 'preserveFeatures', true)
    %
    % 输入:
    %   pc             - pointCloud 对象或 Nx3 矩阵
    %   varargin       - 可选参数对
    %
    % 输出:
    %   pc_filtered    - 滤波后的点云
    %
    % 参数:
    %   SE_SIZE        - 结构元素大小 (default: 3)
    %   numIterations  - 迭代次数 (default: 1)
    %   preserveFeatures - 是否保留特征 (default: true)
    %   verbose        - 详细输出 (default: true)
    
    % 解析输入参数
    p = inputParser;
    addParameter(p, 'SE_SIZE', 3, @isnumeric);
    addParameter(p, 'numIterations', 1, @isnumeric);
    addParameter(p, 'preserveFeatures', true, @islogical);
    addParameter(p, 'verbose', true, @islogical);
    parse(p, varargin{:});
    
    SE_SIZE = p.Results.SE_SIZE;
    num_iter = p.Results.numIterations;
    preserve_features = p.Results.preserveFeatures;
    verbose = p.Results.verbose;
    
    if verbose
        fprintf('\\n=== Adaptive Morphological Filter ===\\n');
        fprintf('SE Size: %d\\n', SE_SIZE);
        fprintf('Iterations: %d\\n', num_iter);
        fprintf('Preserve Features: %s\\n', char(string(preserve_features)));
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
    
    % 1. 建立KD树用于邻域搜索
    tic;
    kdtree = KDTreeSearcher(points);
    
    % 2. 计算自适应半径（基于局部点云密度）
    if preserve_features
        k_neighbors = max(5, round(SE_SIZE * 2));
    else
        k_neighbors = SE_SIZE * 3;
    end
    
    [idx, distances] = knnsearch(kdtree, points, 'k', k_neighbors);
    local_radii = mean(distances(:, end) * ones(1, num_points))';
    
    % 3. 进行形态学滤波
    points_filtered = points;
    
    for iter = 1:num_iter
        points_temp = points_filtered;
        
        % 开运算：先腐蚀后膨胀（去除小噪声）
        points_eroded = morphological_erosion(points_filtered, kdtree, SE_SIZE);
        points_opened = morphological_dilation(points_eroded, kdtree, SE_SIZE);
        
        % 闭运算：先膨胀后腐蚀（填充小孔洞）
        points_dilated = morphological_dilation(points_filtered, kdtree, SE_SIZE);
        points_closed = morphological_erosion(points_dilated, kdtree, SE_SIZE);
        
        % 结合开闭运算结果
        if preserve_features
            % 权重混合：保留更多原始特征
            alpha = 0.6;
            points_filtered = alpha * points_opened + (1-alpha) * points_closed;
        else
            % 平均混合
            points_filtered = (points_opened + points_closed) / 2;
        end
        
        if verbose && iter == 1
            fprintf('Iteration %d/%d completed\\n', iter, num_iter);
        end
    end
    
    % 4. 可选：边界保护（保留陡峭区域）
    if preserve_features
        points_filtered = preserve_sharp_edges(points, points_filtered, kdtree, SE_SIZE);
    end
    
    % 创建输出点云
    if isa(pc, 'pointCloud')
        pc_filtered = pointCloud(points_filtered, 'Intensity', pc.Intensity);
    else
        pc_filtered = points_filtered;
    end
    
    elapsed_time = toc;
    if verbose
        fprintf('Filtering completed in %.2f seconds\\n', elapsed_time);
        fprintf('Output: %d points\\n\\n', size(points_filtered, 1));
    end
    
end

% ============================================================
% 辅助函数：形态学腐蚀
% ============================================================

function points_eroded = morphological_erosion(points, kdtree, SE_SIZE)
    % 腐蚀操作：保留邻域内距离最小的点
    num_points = size(points, 1);
    points_eroded = points;
    
    % 为每个点找邻域内的最小值
    [idx, ~] = knnsearch(kdtree, points, 'k', SE_SIZE);
    
    for i = 1:num_points
        neighbor_points = points(idx(i, :), :);
        % 计算邻域内各维度的最小值
        points_eroded(i, :) = min(neighbor_points, [], 1);
    end
    
end

% ============================================================
% 辅助函数：形态学膨胀
% ============================================================

function points_dilated = morphological_dilation(points, kdtree, SE_SIZE)
    % 膨胀操作：保留邻域内距离最大的点
    num_points = size(points, 1);
    points_dilated = points;
    
    % 为每个点找邻域内的最大值
    [idx, ~] = knnsearch(kdtree, points, 'k', SE_SIZE);
    
    for i = 1:num_points
        neighbor_points = points(idx(i, :), :);
        % 计算邻域内各维度的最大值
        points_dilated(i, :) = max(neighbor_points, [], 1);
    end
    
end

% ============================================================
% 辅助函数：边缘保护
% ============================================================

function points_protected = preserve_sharp_edges(points_orig, points_filtered, kdtree, SE_SIZE)
    % 保护尖锐边缘和陡峭区域
    
    num_points = size(points_orig, 1);
    points_protected = points_filtered;
    
    % 计算每个点的曲率（曲率大=边缘）
    [idx, ~] = knnsearch(kdtree, points_orig, 'k', min(SE_SIZE+1, 10));
    
    curvatures = zeros(num_points, 1);
    
    for i = 1:num_points
        neighbor_points = points_orig(idx(i, 2:end), :);  % 排除自己
        center = points_orig(i, :);
        
        % 计算法向量变化作为曲率估计
        if size(neighbor_points, 1) >= 3
            v1 = neighbor_points(1, :) - center;
            v2 = neighbor_points(2, :) - center;
            normal = cross(v1, v2);
            curvatures(i) = norm(normal);
        end
    end
    
    % 标准化曲率
    curvatures = (curvatures - min(curvatures)) / (max(curvatures) - min(curvatures) + eps);
    
    % 在高曲率区域保留原始点（保留边缘）
    threshold = 0.3;
    for i = 1:num_points
        if curvatures(i) > threshold
            % 加权：高曲率区域更多保留原始点
            weight = curvatures(i) - threshold;
            points_protected(i, :) = (1 - weight) * points_filtered(i, :) + weight * points_orig(i, :);
        end
    end
    
end
