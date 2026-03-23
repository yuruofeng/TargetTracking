%% partitionMeasurementSet.m - 测量集划分函数
%
%  功能说明:
%    对测量点集进行聚类划分，将相邻的测量点归为同一单元
%    使用基于距离的连通性聚类算法
%
%  依赖文件:
%    utils.Logger - 日志记录器
%
%  使用方法:
%    partitions = partitionMeasurementSet(measurements, 50, 10);
%
%  作者: 重构版本
%  日期: 2026-03-01

function partitions = partitionMeasurementSet(measurements, maxDistance, minDistance)
    % PARTITIONMEASUREMENTSET 测量集划分
    %   partitions = partitionMeasurementSet(measurements, maxDistance, minDistance)
    %
    %   输入参数:
    %       measurements - 测量点集 (double, 2 x N)
    %       maxDistance - 最大聚类距离 (double, 标量)
    %       minDistance - 最小聚类距离 (double, 标量)
    %
    %   输出参数:
    %       partitions - 划分结构体数组 (struct array)
    %           .cells - 测量单元的cell数组
    %           .numCells - 单元数量
    %
    %   示例:
    %       partitions = partitionMeasurementSet(meas, 0.8, 0.3);
    
    arguments
        measurements (2,:) double
        maxDistance (1,1) double = 0.8
        minDistance (1,1) double {mustBeNonnegative} = 0.3
    end
    
    [~, ~] = deal(maxDistance, minDistance);
    
    logger = utils.Logger('PartitionMeas', 'DEBUG');
    logger.logFunctionStart('partitionMeasurementSet');
    
    numMeasurements = size(measurements, 2);
    
    if numMeasurements == 0
        partitions = struct('cells', {{}}, 'numCells', 0);
        logger.logFunctionEnd('partitionMeasurementSet', 0);
        return;
    end
    
    if numMeasurements == 1
        partitions = struct('cells', {{measurements}}, 'numCells', 1);
        logger.logFunctionEnd('partitionMeasurementSet', 0);
        return;
    end
    
    distanceMatrix = zeros(numMeasurements, numMeasurements);
    for i = 1:numMeasurements
        for j = (i+1):numMeasurements
            distanceMatrix(i, j) = norm(measurements(:, i) - measurements(:, j));
            distanceMatrix(j, i) = distanceMatrix(i, j);
        end
    end
    
    adjacencyMatrix = distanceMatrix < maxDistance;
    
    clusterLabels = zeros(1, numMeasurements);
    currentLabel = 0;
    
    for i = 1:numMeasurements
        if clusterLabels(i) == 0
            currentLabel = currentLabel + 1;
            clusterLabels(i) = currentLabel;
            
            stack = i;
            while ~isempty(stack)
                current = stack(1);
                stack(1) = [];
                
                neighbors = find(adjacencyMatrix(current, :) & clusterLabels == 0);
                clusterLabels(neighbors) = currentLabel;
                stack = [stack, neighbors];
            end
        end
    end
    
    numClusters = currentLabel;
    
    partitions = struct('cells', cell(1, 1), 'numCells', 0);
    cells = cell(1, numClusters);
    
    for c = 1:numClusters
        indices = find(clusterLabels == c);
        cells{c} = measurements(:, indices);
    end
    
    partitions.cells = cells;
    partitions.numCells = numClusters;
    
    logger.debug(sprintf('划分为 %d 个聚类', numClusters));
    logger.logFunctionEnd('partitionMeasurementSet', 0);
end
