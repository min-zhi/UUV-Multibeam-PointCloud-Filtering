% ============================================================
% UUV Multibeam Point Cloud Filtering Configuration
% ============================================================
% 点云滤波配置文件
% 调整以下参数以优化不同场景的滤波效果
% ============================================================

function config = filter_config()
    % 返回滤波配置结构体
    
    % ========== 形态学滤波参数 ==========
    config.morphological.SE_SIZE = 3;              % 结构元素大小 (3-7)
    config.morphological.num_iterations = 1;      % 开闭运算迭代次数
    config.morphological.preserve_edges = true;   % 是否保留边缘
    
    % ========== 多尺度高斯滤波参数 ==========
    config.multiscale.num_scales = 3;             % 金字塔层级数
    config.multiscale.base_sigma = 1.0;           % 基础高斯标准差
    config.multiscale.scale_factor = 1.5;         % 尺度因子
    config.multiscale.threshold = 0.1;            % 特征检测阈值
    
    % ========== 统计离群点检测参数 ==========
    config.statistical.num_neighbors = 10;        % 邻域点数
    config.statistical.std_ratio = 2.0;           % 标准差倍数阈值
    config.statistical.use_knn = true;            % 使用KNN还是固定半径
    
    % ========== 双边滤波参数 ==========
    config.bilateral.sigma_spatial = 3.0;         % 空间域标准差
    config.bilateral.sigma_range = 0.1;           % 值域标准差
    config.bilateral.kernel_size = 5;             % 核大小
    config.bilateral.num_iterations = 2;          % 迭代次数
    
    % ========== 通用参数 ==========
    config.general.verbose = true;                % 是否输出详细信息
    config.general.save_intermediate = false;     % 是否保存中间结果
    config.general.max_points = 1e6;              % 最大点数（内存限制）
    config.general.voxel_size = 0.01;             % 体素大小用于下采样
    
    % ========== 质量评估参数 ==========
    config.quality.compute_metrics = true;        % 是否计算性能指标
    config.quality.has_ground_truth = false;      % 是否有真值数据
    config.quality.noise_level = 0.05;            % 估计噪声水平
    
    % ========== 可视化参数 ==========
    config.visualization.show_results = true;     % 是否显示结果
    config.visualization.show_comparison = true;  % 是否显示对比
    config.visualization.point_size = 6;          % 点的大小
    config.visualization.colormap = 'jet';        % 颜色映射
    
end

% ============================================================
% 获取特定算法的参数
% ============================================================

function params = get_morphological_params(config)
    params = config.morphological;
end

function params = get_multiscale_params(config)
    params = config.multiscale;
end

function params = get_statistical_params(config)
    params = config.statistical;
end

function params = get_bilateral_params(config)
    params = config.bilateral;
end

% ============================================================
% 参数验证函数
% ============================================================

function is_valid = validate_config(config)
    % 验证配置参数的有效性
    is_valid = true;
    
    % 检查形态学参数
    if config.morphological.SE_SIZE < 1 || config.morphological.SE_SIZE > 15
        warning('SE_SIZE should be between 1 and 15');
        is_valid = false;
    end
    
    % 检查高斯参数
    if config.multiscale.num_scales < 1 || config.multiscale.num_scales > 10
        warning('num_scales should be between 1 and 10');
        is_valid = false;
    end
    
    % 检查统计参数
    if config.statistical.num_neighbors < 1
        warning('num_neighbors should be positive');
        is_valid = false;
    end
    
    % 检查双边滤波参数
    if config.bilateral.sigma_spatial <= 0 || config.bilateral.sigma_range <= 0
        warning('sigma values should be positive');
        is_valid = false;
    end
    
end

% ============================================================
% 推荐配置预设
% ============================================================

function config = get_preset_config(preset_name)
    % 返回预设配置
    % 预设类型: 'smooth', 'balanced', 'preserve', 'aggressive'
    
    config = filter_config();  % 获取默认配置
    
    switch lower(preset_name)
        case 'smooth'
            % 平滑型：适合噪声少、特征不复杂的情况
            config.morphological.SE_SIZE = 5;
            config.multiscale.num_scales = 2;
            config.statistical.std_ratio = 3.0;
            config.bilateral.num_iterations = 3;
            
        case 'balanced'
            % 平衡型：综合性能（默认）
            % 使用默认值
            
        case 'preserve'
            % 保留型：最大保留特征
            config.morphological.SE_SIZE = 2;
            config.multiscale.num_scales = 4;
            config.statistical.std_ratio = 1.5;
            config.bilateral.sigma_range = 0.05;
            
        case 'aggressive'
            % 激进型：最大去噪
            config.morphological.SE_SIZE = 7;
            config.multiscale.num_scales = 2;
            config.statistical.std_ratio = 1.0;
            config.bilateral.num_iterations = 4;
            
        otherwise
            warning('Unknown preset: %s. Using default config.', preset_name);
    end
    
end
