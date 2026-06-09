% ============================================================
% Multi-Scale Gaussian Pyramid Filter for Point Cloud
% ============================================================
% 多尺度高斯金字塔滤波
% 在多个尺度上分离噪声和特征，有效保留微地形细节
% ============================================================

function pc_filtered = multiScaleGaussianFilter(pc, varargin)
    % 多尺度高斯金字塔滤波
    %
    % 用法:
    %   pc_filtered = multiScaleGaussianFilter(pc)
    %   pc_filtered = multiScaleGaussianFilter(pc, 'numScales', 3)
    %
    % 输入:
    %   pc             - pointCloud 对象或 Nx3 矩阵
    %   varargin       - 可选参数对
    %
    % 输出:
    %   pc_filtered    - 滤波后的点云
    
    % 解析输入参数
    p = inputParser;
    addParameter(p, 'numScales', 3, @isnumeric);
    addParameter(p, 'baseSigma', 1.0, @isnumeric);
    addParameter(p, 'scaleFactor', 1.5, @isnumeric);
    addParameter(p, 'featureThreshold', 0.1, @isnumeric);
    addParameter(p, 'verbose', true, @islogical);
    parse(p, varargin{:});
    
    num_scales = p.Results.numScales;
    base_sigma = p.Results.baseSigma;
    scale_factor = p.Results.scaleFactor;
    feature_threshold = p.Results.featureThreshold;
    verbose = p.Results.verbose;
    
    if verbose
        fprintf('\\n=== Multi-Scale Gaussian Filter ===\\n');
        fprintf('Number of Scales: %d\\n', num_scales);
        fprintf('Base Sigma: %.2f\\n', base_sigma);
    end
    
    % 转换为矩阵形式
    if isa(pc, 'pointCloud')
        points = pc.Location;
    else
        points = pc;
    end
    
    [num_points, dim] = size(points);
    
    if verbose
        fprintf('Processing %d points...\\n', num_points);
    end
    
    tic;
    
    % 1. 构建KD树
    kdtree = KDTreeSearcher(points);
    
    % 2. 建立高斯金字塔
    pyramid = cell(num_scales, 1);
    sigmas = zeros(num_scales, 1);
    
    for s = 1:num_scales
        sigma = base_sigma * (scale_factor ^ (s-1));
        sigmas(s) = sigma;
        
        % 应用高斯滤波（在点云上模拟）
        pyramid{s} = gaussian_blur_pointcloud(points, kdtree, sigma);
        
        if verbose && mod(s, max(1, num_scales/4)) == 0
            fprintf('Scale %d/%d completed (sigma=%.2f)\\n', s, num_scales, sigma);
        end
    end
    
    % 3. 计算Laplacian金字塔（差值）
    laplacian_pyramid = cell(num_scales-1, 1);
    
    for s = 1:num_scales-1
        laplacian_pyramid{s} = pyramid{s} - pyramid{s+1};
    end
    
    % 4. 特征检测和选择性重建
    points_filtered = zeros(num_points, dim);
    feature_strength = zeros(num_points, 1);
    
    for i = 1:num_points
        max_laplacian = 0;
        scale_contribution = zeros(num_scales-1, 1);
        
        % 计算每个尺度的特征强度
        for s = 1:num_scales-1
            laplacian_val = norm(laplacian_pyramid{s}(i, :));
            scale_contribution(s) = laplacian_val;
            max_laplacian = max(max_laplacian, laplacian_val);
        end
        
        feature_strength(i) = max_laplacian;
        
        % 选择合适的尺度进行重建
        if max_laplacian > feature_threshold
            % 高特征强度：倾向于使用较小尺度（保留细节）
            [~, best_scale] = max(scale_contribution);
            if best_scale > 1
                alpha = feature_strength(i);
                alpha = min(1, alpha);  % 限制到[0,1]
                points_filtered(i, :) = alpha * pyramid{best_scale}(i, :) + ...
                                        (1-alpha) * points(i, :);
            else
                points_filtered(i, :) = points(i, :);
            end
        else
            % 低特征强度：使用较大尺度（更平滑）
            points_filtered(i, :) = pyramid{num_scales}(i, :);
        end
    end
    
    % 5. 可选：迭代细化
    points_filtered = refine_with_feature_guidance(points, points_filtered, ...
                                                    feature_strength, kdtree);
    
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
% 辅助函数：点云高斯模糊
% ============================================================

function points_blurred = gaussian_blur_pointcloud(points, kdtree, sigma)
    % 对点云应用高斯模糊
    
    num_points = size(points, 1);
    points_blurred = zeros(size(points));
    
    % 确定邻域大小（基于sigma）
    search_radius = 3 * sigma;
    
    for i = 1:num_points
        % 找邻域内的点
        [idx, distances] = rangesearch(kdtree, points(i, :), search_radius);
        idx = idx{1};
        distances = distances{1}';
        
        if length(idx) > 1
            % 计算高斯权重
            weights = exp(-(distances.^2) / (2 * sigma^2));
            weights = weights / sum(weights);  % 归一化
            
            % 加权平均
            points_blurred(i, :) = weights' * points(idx, :);
        else
            % 邻域为空，保持原点
            points_blurred(i, :) = points(i, :);
        end
    end
    
end

% ============================================================
% 辅助函数：特征引导细化
% ============================================================

function points_refined = refine_with_feature_guidance(points_orig, points_filtered, ...
                                                        feature_strength, kdtree)
    % 基于特征强度进行局部调整
    
    num_points = size(points_orig, 1);
    points_refined = points_filtered;
    
    % 标准化特征强度
    fs_normalized = (feature_strength - min(feature_strength)) / ...
                    (max(feature_strength) - min(feature_strength) + eps);
    
    % 局部一致性检查
    search_radius = 0.5;
    
    for i = 1:num_points
        [idx, distances] = rangesearch(kdtree, points_orig(i, :), search_radius);
        idx = idx{1};
        
        if length(idx) > 1
            % 计算邻域特征强度的一致性
            neighbor_fs = fs_normalized(idx);
            fs_variance = var(neighbor_fs);
            
            % 如果邻域特征一致性高，允许更强的滤波
            if fs_variance < 0.1
                % 邻域特征均匀，增强滤波效果
                alpha = 1.2 * fs_normalized(i);
                alpha = min(1, max(0, alpha));
                points_refined(i, :) = (1 - alpha) * points_filtered(i, :) + ...
                                       alpha * points_orig(i, :);
            end
        end
    end
    
end
