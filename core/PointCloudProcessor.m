% ============================================================
% Point Cloud Processor - Main Processing Class
% ============================================================
% 点云处理主类
% 提供统一接口调用各种滤波算法
% ============================================================

classdef PointCloudProcessor
    % 点云处理器类
    
    properties
        pointCloud          % 当前点云
        original_pointCloud % 原始点云
        config              % 配置参数
        history             % 处理历史
    end
    
    methods
        % ========== 构造函数 ==========
        function obj = PointCloudProcessor(varargin)
            % 初始化点云处理器
            
            obj.history = {};
            
            if nargin >= 1 && ~isempty(varargin{1})
                obj.pointCloud = varargin{1};
                obj.original_pointCloud = varargin{1};
            end
            
            if nargin >= 2 && ~isempty(varargin{2})
                obj.config = varargin{2};
            else
                obj.config = filter_config();
            end
        end
        
        % ========== 核心滤波方法 ==========
        
        function obj = adaptiveMorphFilter(obj, varargin)
            % 应用自适应形态学滤波
            fprintf('Applying Adaptive Morphological Filter...\\n');
            
            obj.pointCloud = adaptiveMorphologicalFilter(obj.pointCloud, varargin{:});
            obj.history{end+1} = 'Adaptive Morphological Filter';
        end
        
        function obj = multiScaleGaussFilter(obj, varargin)
            % 应用多尺度高斯滤波
            fprintf('Applying Multi-Scale Gaussian Filter...\\n');
            
            obj.pointCloud = multiScaleGaussianFilter(obj.pointCloud, varargin{:});
            obj.history{end+1} = 'Multi-Scale Gaussian Filter';
        end
        
        function obj = statisticalOutlierRemove(obj, varargin)
            % 应用统计离群点去除
            fprintf('Applying Statistical Outlier Removal...\\n');
            
            [obj.pointCloud, ~] = statisticalOutlierRemoval(obj.pointCloud, varargin{:});
            obj.history{end+1} = 'Statistical Outlier Removal';
        end
        
        % ========== 数据管理 ==========
        
        function obj = loadPointCloud(obj, filename)
            % 加载点云文件
            fprintf('Loading point cloud: %s\\n', filename);
            
            obj.pointCloud = readPointCloud(filename);
            obj.original_pointCloud = obj.pointCloud;
            obj.history = {};
        end
        
        function savePointCloud(obj, filename)
            % 保存点云文件
            fprintf('Saving point cloud: %s\\n', filename);
            
            if isa(obj.pointCloud, 'pointCloud')
                pcwrite(obj.pointCloud, filename);
            else
                writematrix(obj.pointCloud, filename, 'Delimiter', ' ');
            end
        end
        
        function obj = reset(obj)
            % 重置为原始点云
            fprintf('Resetting to original point cloud...\\n');
            
            obj.pointCloud = obj.original_pointCloud;
            obj.history = {};
        end
        
        % ========== 分析和可视化 ==========
        
        function displayInfo(obj)
            % 显示点云信息
            fprintf('\\n=== Point Cloud Information ===\\n');
            
            if isa(obj.pointCloud, 'pointCloud')
                fprintf('Number of points: %d\\n', obj.pointCloud.Count);
                fprintf('X range: [%.4f, %.4f]\\n', min(obj.pointCloud.Location(:,1)), ...
                                                     max(obj.pointCloud.Location(:,1)));
                fprintf('Y range: [%.4f, %.4f]\\n', min(obj.pointCloud.Location(:,2)), ...
                                                     max(obj.pointCloud.Location(:,2)));
                fprintf('Z range: [%.4f, %.4f]\\n', min(obj.pointCloud.Location(:,3)), ...
                                                     max(obj.pointCloud.Location(:,3)));
            else
                fprintf('Number of points: %d\\n', size(obj.pointCloud, 1));
            end
            
            fprintf('Processing history:\\n');
            for i = 1:length(obj.history)
                fprintf('  %d. %s\\n', i, obj.history{i});
            end
            fprintf('\\n');
        end
        
        function visualize(obj)
            % 可视化点云
            visualizePointCloud(obj.pointCloud);
        end
        
        function compareOriginal(obj)
            % 对比原始和滤波后的点云
            figure('Position', [100, 100, 1200, 500]);
            
            subplot(1, 2, 1);
            visualizePointCloud(obj.original_pointCloud);
            title('Original Point Cloud');
            
            subplot(1, 2, 2);
            visualizePointCloud(obj.pointCloud);
            title('Filtered Point Cloud');
        end
        
        function metrics = computeMetrics(obj, varargin)
            % 计算性能指标
            fprintf('\\nComputing metrics...\\n');
            
            metrics = computeMetrics(obj.original_pointCloud, obj.pointCloud, varargin{:});
        end
    end
end
