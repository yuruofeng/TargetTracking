classdef MeasurementModel
% MEASUREMENTMODEL  目标跟踪非线性测量模型。
%   提供雷达测量函数（距离/方位角）用于DBT，以及图像域点扩散函数用于TBD。
%
%   CT模型（协调转弯）：
%       状态向量: [x, vx, y, vy, omega]
%       测量向量: [方位角, 距离]
%
%   方法列表：
%       ctMeasFunc      - 距离/方位角测量函数
%       ctMeasJacobian  - 测量函数雅可比矩阵
%       ctDynamicMatrix - CT状态转移矩阵
%       cvDynamicMatrix - CV状态转移矩阵
%       createPsfKernel - 高斯点扩散函数核
%
%   See also: dbt.EKF, dbt.UKF, tbd.DpTbd, tbd.PfTbd

    methods (Static)

        function z = ctMeasFunc(x)
        % CTMEASFUNC  非线性距离/方位角测量函数。
        %   z = ctMeasFunc(x) 从状态向量计算雷达测量值。
        %
        %   输入参数：
        %       x - 状态向量 [x; vx; y; vy; omega]
        %
        %   输出参数：
        %       z - 测量向量 [方位角; 距离]
            z = [atan2(x(3), x(1)); norm(x([1 3]))];
        end

        function H = ctMeasJacobian(x)
        % CTMEASJACOBIAN  距离/方位角测量模型的雅可比矩阵。
        %   H = ctMeasJacobian(x) 计算测量函数关于状态的偏导数。
        %
        %   输入参数：
        %       x - 状态向量 [x; vx; y; vy; omega]
        %
        %   输出参数：
        %       H - 雅可比矩阵 [2 x 5]
            px = x(1);  py = x(3);
            r2 = px^2 + py^2;
            r  = sqrt(r2);
            H  = [-py/r2  0  px/r2  0  0; ...
                   px/r   0  py/r   0  0];
        end

        function F = ctDynamicMatrix(T, x)
        % CTDYNAMICMATRIX  协调转弯状态转移矩阵。
        %   F = ctDynamicMatrix(T, x) 返回CT模型的状态转移矩阵。
        %
        %   输入参数：
        %       T - 采样间隔 [s]
        %       x - 状态向量（使用omega = x(5)）
        %
        %   输出参数：
        %       F - 状态转移矩阵 [5 x 5]
            w = x(5);
            if abs(w) < 1e-8
                % 角速度接近零时退化为CV模型
                F = [1  T  0  0  0; ...
                     0  1  0  0  0; ...
                     0  0  1  T  0; ...
                     0  0  0  1  0; ...
                     0  0  0  0  1];
            else
                % 标准CT模型
                F = [1  sin(w*T)/w        0  -(1-cos(w*T))/w  0; ...
                     0  cos(w*T)           0  -sin(w*T)        0; ...
                     0  (1-cos(w*T))/w     1   sin(w*T)/w      0; ...
                     0  sin(w*T)           0   cos(w*T)        0; ...
                     0  0                  0   0               1];
            end
        end

        function F = cvDynamicMatrix(T, dim)
        % CVDYNAMICMATRIX  匀速运动状态转移矩阵。
        %   F = cvDynamicMatrix(T, dim) 返回CV模型的状态转移矩阵。
        %
        %   输入参数：
        %       T   - 采样间隔 [s]
        %       dim - 空间维度（2表示2D，3表示3D）
        %
        %   输出参数：
        %       F - 状态转移矩阵 [2*dim x 2*dim]
            F = kron(eye(dim), [1, T; 0, 1]);
        end

        function [psfKernel, coords] = createPsfKernel(radius)
        % CREATEPSFKERNEL  创建高斯点扩散函数核。
        %   [psfKernel, coords] = createPsfKernel(radius) 生成归一化的
        %   高斯PSF，用于图像域测量。
        %
        %   输入参数：
        %       radius - PSF半径（核大小 = 2*radius+1）
        %
        %   输出参数：
        %       psfKernel - 归一化高斯PSF [2*radius+1 x 2*radius+1]
        %       coords    - 坐标网格 [行, 列]
            [kR, kC]  = meshgrid(-radius:radius, -radius:radius);
            psfSigma  = radius / 2;
            psfKernel = exp(-(kR.^2 + kC.^2) / (2 * psfSigma^2));
            psfKernel = psfKernel / max(psfKernel(:));  % 归一化
            coords = [kR(:), kC(:)];
        end

    end
end
