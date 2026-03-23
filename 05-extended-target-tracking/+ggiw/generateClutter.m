%% generateClutter.m - 杂波测量生成函数
%
%  功能说明:
%    在指定监视区域内生成泊松分布的杂波测量点
%    用于扩展目标跟踪仿真中的杂波模拟
%
%  依赖文件:
%    utils.Logger - 日志记录器
%
%  使用方法:
%    clutter = generateClutter(5e-6, [-200 200 -200 200], 100);
%
%  作者: 重构版本
%  日期: 2026-03-01

function [clutter] = generateClutter(clutterRate, areaBounds, numTimeSteps)
    % GENERATECLUTTER 生成杂波测量
    %   clutter = generateClutter(clutterRate, areaBounds, numTimeSteps)
    %
    %   输入参数:
    %       clutterRate - 杂波率，每单位面积的杂波期望数 (double, 标量)
    %       areaBounds - 监视区域边界 [xMin, xMax, yMin, yMax] (double, 向量)
    %       numTimeSteps - 时间步数 (double, 标量)
    %
    %   输出参数:
    %       clutter - 杂波测量结构体数组 (struct array)
    %           .points - 每个时间步的杂波点坐标 (2 x N matrix)
    %
    %   异常:
    %       如果参数无效，抛出 MException
    %
    %   示例:
    %       clutter = generateClutter(5e-6, [-200 200 -200 200], 100);
    
    arguments
        clutterRate (1,1) double {mustBeNonnegative}
        areaBounds (1,4) double
        numTimeSteps (1,1) double {mustBePositive}
    end
    
    logger = utils.Logger('GenerateClutter', 'INFO');
    logger.logFunctionStart('generateClutter');
    
    xMin = areaBounds(1);
    xMax = areaBounds(2);
    yMin = areaBounds(3);
    yMax = areaBounds(4);
    
    areaVolume = (xMax - xMin) * (yMax - yMin);
    
    if clutterRate == 0
        clutter = struct('points', repmat({[]}, 1, numTimeSteps));
        logger.debug('杂波率为0，返回空杂波集');
        return;
    end
    
    expectedClutter = clutterRate * areaVolume;
    logger.debug(sprintf('期望杂波数: %.2f per time step', expectedClutter));
    
    clutter = struct('points', cell(1, numTimeSteps));
    
    for k = 1:numTimeSteps
        numClutterPoints = poissrnd(expectedClutter);
        
        if numClutterPoints > 0
            xCoords = xMin + (xMax - xMin) * rand(1, numClutterPoints);
            yCoords = yMin + (yMax - yMin) * rand(1, numClutterPoints);
            clutter(k).points = [xCoords; yCoords];
        else
            clutter(k).points = [];
        end
    end
    
    logger.logFunctionEnd('generateClutter', 0);
end
