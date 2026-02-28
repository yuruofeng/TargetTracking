classdef FilterUtils
% FILTERUTILS  滤波算法共享工具函数。
%   提供通用的数值计算操作，包括Cholesky分解、sigma点生成、
%   容积点生成和系统重采样，用于DBT和TBD滤波器。
%
%   方法列表：
%       cholPsd              - 鲁棒Cholesky分解（含SVD回退）
%       generateSigmaPoints  - Van der Merwe sigma点（用于UKF）
%       generateCubaturePoints - 三阶容积点（用于CKF）
%       systematicResample   - 低方差系统重采样
%       computeRmse          - 计算均方根误差
%       wrapToPi             - 角度归一化到[-pi, pi]
%
%   参考文献：
%       [1] Van der Merwe, R. (2004). Sigma-Point Kalman Filters.
%       [2] Arasaratnam, I. (2009). Cubature Kalman Filters.
%
%   See also: dbt.UKF, dbt.CKF, dbt.ParticleFilter

    methods (Static)

        function L = cholPsd(A)
        % CHOLPSD  鲁棒Cholesky分解（含SVD回退）。
        %   L = cholPsd(A) 返回下三角Cholesky因子L，满足L*L' ≈ A。
        %   对于病态矩阵自动回退到SVD分解。
        %
        %   输入参数：
        %       A - 对称半正定矩阵 [n x n]
        %
        %   输出参数：
        %       L - 下三角分解因子 [n x n]
            [~, flag] = chol(A);
            if flag == 0
                L = chol(A, 'lower');
            else
                % 病态矩阵使用SVD分解
                [~, S, V] = svd(A);
                L = V * sqrt(S);
            end
        end

        function [weights, sigmaPts] = generateSigmaPoints(mu, P, n, alpha, beta, kappa)
        % GENERATESIGMAPOINTS  生成Van der Merwe sigma点（用于UKF）。
        %   生成2n+1个sigma点及其对应的均值/协方差权重。
        %
        %   输入参数：
        %       mu    - 状态均值向量 [n x 1]
        %       P     - 状态协方差矩阵 [n x n]
        %       n     - 状态维度
        %       alpha - 主缩放参数（典型值：1e-3）
        %       beta  - 先验知识参数（高斯分布取2）
        %       kappa - 次缩放参数（典型值：0）
        %
        %   输出参数：
        %       weights  - [2 x 2n+1]矩阵，第1行为均值权重，第2行为协方差权重
        %       sigmaPts - [n x 2n+1] sigma点矩阵
            lambda = alpha^2 * (n + kappa) - n;
            sqrtP  = utils.FilterUtils.cholPsd(P * (n + lambda));

            % 生成sigma点
            sigmaPts = zeros(n, 2*n + 1);
            sigmaPts(:, 1) = mu;  % 中心点
            for i = 1:n
                sigmaPts(:, i+1)   = mu + sqrtP(:, i);   % 正向偏移
                sigmaPts(:, i+n+1) = mu - sqrtP(:, i);   % 负向偏移
            end

            % 计算权重
            wm = [lambda / (n + lambda), repmat(1 / (2*(n+lambda)), 1, 2*n)];
            wc = wm;
            wc(1) = wc(1) + (1 - alpha^2 + beta);  % 中心点协方差权重修正
            weights = [wm; wc];
        end

        function [weights, cubPts] = generateCubaturePoints(mu, P, n)
        % GENERATECUBATUREPOINTS  生成三阶容积点（用于CKF）。
        %   生成2n个容积点，权重均匀分布。
        %
        %   输入参数：
        %       mu - 状态均值向量 [n x 1]
        %       P  - 状态协方差矩阵 [n x n]
        %       n  - 状态维度
        %
        %   输出参数：
        %       weights - [1 x 2n] 均匀权重向量（1/(2n)）
        %       cubPts  - [n x 2n] 容积点矩阵
            nCp = 2 * n;
            xi  = sqrt(n) * [eye(n), -eye(n)];  % 单位容积点
            sqP = utils.FilterUtils.cholPsd(P);

            % 变换容积点
            cubPts = zeros(n, nCp);
            for i = 1:nCp
                cubPts(:, i) = mu + sqP * xi(:, i);
            end
            weights = ones(1, nCp) / nCp;  % 均匀权重
        end

        function idx = systematicResample(weights, nPart)
        % SYSTEMATICRESAMPLE  低方差系统重采样。
        %   使用系统重采样方法从归一化权重分布中重采样粒子索引。
        %   相比多项式重采样，具有更低的采样方差。
        %
        %   输入参数：
        %       weights - 归一化权重向量 [nPart x 1]
        %       nPart   - 粒子数量（可选，默认为weights长度）
        %
        %   输出参数：
        %       idx - 重采样后的粒子索引 [nPart x 1]
            if nargin < 2
                nPart = length(weights);
            end
            cumW = cumsum(weights(:));
            idx  = zeros(nPart, 1);
            u1   = rand / nPart;  % 初始偏移
            j    = 1;
            for i = 1:nPart
                u = u1 + (i - 1) / nPart;  % 系统采样点
                while cumW(j) < u
                    j = j + 1;
                end
                idx(i) = j;
            end
        end

        function rmse = computeRmse(estimates, truth, indices)
        % COMPUTERMSE  计算估计值与真值之间的RMSE。
        %   rmse = computeRmse(estimates, truth, indices) 计算指定状态
        %   分量的均方根误差。
        %
        %   输入参数：
        %       estimates - 估计状态 [dim x nSteps] 或 [nSteps x dim]
        %       truth     - 真实状态（与estimates同尺寸）
        %       indices   - 要包含的状态索引（如[1,3]表示位置）
        %
        %   输出参数：
        %       rmse - RMSE时间序列 [1 x nSteps]
            if size(estimates, 1) == length(indices)
                err = estimates(indices, :) - truth(indices, :);
            else
                err = estimates(:, indices) - truth(:, indices);
            end
            rmse = sqrt(sum(err.^2, 1));
        end

        function angle = wrapToPi(angle)
        % WRAPTOPI  将角度归一化到[-pi, pi]。
        %   angle = wrapToPi(angle) 将角度值映射到[-pi, pi]范围。
        %
        %   输入参数：
        %       angle - 角度值（标量或数组）[rad]
        %
        %   输出参数：
        %       angle - 归一化后的角度 [-pi, pi]
            angle = mod(angle + pi, 2*pi) - pi;
        end

    end
end
