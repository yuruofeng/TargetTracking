%% plotExtentEllipse.m - 椭圆扩展可视化辅助函数
%
%  功能说明:
%    根据协方差矩阵绘制椭圆扩展，用于扩展目标可视化
%    计算椭圆轮廓的X和Y坐标点
%
%  依赖文件:
%    utils.ArrayUtils - 数组工具类
%
%  使用方法:
%    [x, y] = plotExtentEllipse(0, 0, diag([100, 50]));
%    plot(x, y);
%
%  作者: 重构版本
%  日期: 2026-03-01

function [xCoords, yCoords] = plotExtentEllipse(centerX, centerY, covarianceMatrix, ...
    numStdDev, numPoints)
    % PLOTEXTEPSELLIPSE 椭圆扩展可视化辅助函数
    %   [x, y] = plotExtentEllipse(centerX, centerY, covarianceMatrix)
    %   [x, y] = plotExtentEllipse(centerX, centerY, covarianceMatrix, numStdDev, numPoints)
    %
    %   输入参数:
    %       centerX - 椭圆中心X坐标 (double, 标量)
    %       centerY - 椭圆中心Y坐标 (double, 标量)
    %       covarianceMatrix - 协方差矩阵 (double, 2x2 matrix)
    %       numStdDev - 标准差倍数 (double, 可选, 默认1)
    %       numPoints - 绘图点数 (double, 可选, 默认20)
    %
    %   输出参数:
    %       xCoords - X坐标向量 (double, 向量)
    %       yCoords - Y坐标向量 (double, 向量)
    %
    %   异常:
    %       如果协方差矩阵不是2x2正定矩阵，抛出 MException
    %
    %   示例:
    %       [x, y] = plotExtentEllipse(0, 0, diag([100, 50]));
    %       plot(x, y);
    
    arguments
        centerX (1,1) double
        centerY (1,1) double
        covarianceMatrix (2,2) double
        numStdDev (1,1) double = 1
        numPoints (1,1) double = 20
    end
    
    if size(covarianceMatrix, 1) ~= 2 || size(covarianceMatrix, 2) ~= 2
        error('PlotExtent:InvalidDimension', ...
            '协方差矩阵必须是2x2矩阵');
    end
    
    sqrtCov = sqrtm(covarianceMatrix);
    
    if any(isnan(sqrtCov(:))) || any(isinf(sqrtCov(:)))
        sqrtCov = sqrtm(utils.ArrayUtils.ensurePositiveDefinite(covarianceMatrix));
    end
    
    angles = linspace(0, 2*pi, numPoints + 1);
    angles = angles(1:end-1);
    
    unitCircle = [cos(angles); sin(angles)];
    
    ellipse = numStdDev * sqrtCov * unitCircle;
    
    xCoords = ellipse(1, :)' + centerX;
    yCoords = ellipse(2, :)' + centerY;
end
