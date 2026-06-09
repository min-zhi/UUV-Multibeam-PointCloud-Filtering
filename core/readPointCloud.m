% ============================================================
% Read Point Cloud from File
% ============================================================
% 从文件读取点云
% 支持多种格式：XYZ, PCD, LAS, PLY
% ============================================================

function pc = readPointCloud(filename)
    % 读取点云文件
    %
    % 用法:
    %   pc = readPointCloud('data.xyz')
    %   pc = readPointCloud('data.pcd')
    %
    % 输入:
    %   filename - 文件路径
    %
    % 输出:
    %   pc - pointCloud 对象
    
    % 检查文件是否存在
    if ~isfile(filename)
        error('File not found: %s', filename);
    end
    
    % 获取文件扩展名
    [~, ~, ext] = fileparts(filename);
    ext = lower(ext);
    
    fprintf('Reading point cloud from: %s\\n', filename);
    
    switch ext
        case '.xyz'
            pc = read_xyz_file(filename);
            
        case '.pcd'
            pc = pcread(filename);
            
        case {'.ply'}
            pc = pcread(filename);
            
        case {'.las', '.laz'}
            try
                lpc = lasread(filename);
                pc = pointCloud([lpc.X, lpc.Y, lpc.Z]);
            catch
                data = readmatrix(filename);
                pc = pointCloud(data(:, 1:3));
            end
            
        otherwise
            error('Unsupported file format: %s', ext);
    end
    
    fprintf('Loaded %d points\\n', pc.Count);
    
end

% ============================================================
% XYZ文件读取
% ============================================================

function pc = read_xyz_file(filename)
    % 读取XYZ格式文件
    
    data = readmatrix(filename);
    
    if size(data, 2) >= 3
        points = data(:, 1:3);
        
        if size(data, 2) >= 4
            intensity = data(:, 4);
            pc = pointCloud(points, 'Intensity', intensity);
        else
            pc = pointCloud(points);
        end
    else
        error('XYZ file must have at least 3 columns (x, y, z)');
    end
    
end
