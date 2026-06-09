% ============================================================
% Visualize Point Cloud
% ============================================================
% 点云可视化函数
% ============================================================

function visualizePointCloud(pc, varargin)
    % 可视化点云
    %
    % 用法:
    %   visualizePointCloud(pc)
    %   visualizePointCloud(pc, 'Color', 'r')
    %   visualizePointCloud(pc, 'Size', 10)
    %
    % 参数:
    %   Color   - 点的颜色 (default: 'b')
    %   Size    - 点的大小 (default: 6)
    
    p = inputParser;
    addParameter(p, 'Color', 'b');
    addParameter(p, 'Size', 6);
    parse(p, varargin{:});
    
    if isa(pc, 'pointCloud')
        pcshow(pc, 'MarkerSize', p.Results.Size);
        grid on;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title(sprintf('Point Cloud (%d points)', pc.Count));
    else
        % 矩阵形式
        scatter3(pc(:,1), pc(:,2), pc(:,3), p.Results.Size, p.Results.Color, '.');
        grid on;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title(sprintf('Point Cloud (%d points)', size(pc, 1)));
    end
    
    axis equal;
    view(45, 45);
    
end
