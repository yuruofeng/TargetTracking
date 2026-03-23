classdef ArrayUtils
    % ARRAYUTILS 数组操作工具类
    %   提供常用的数组操作静态方法
    %
    %   使用示例:
    %       result = utils.ArrayUtils.normalizeVector(v);
    %       cov = utils.ArrayUtils.makeSymmetric(P);
    
    methods (Static)
        function normalized = normalizeVector(v)
            % NORMALIZEVECTOR 归一化向量
            %   normalized = utils.ArrayUtils.normalizeVector(v)
            %
            %   输入参数:
            %       v - 输入向量 (double array)
            %
            %   输出参数:
            %       normalized - 归一化后的向量 (double array)
            
            total = sum(v);
            if total == 0
                normalized = v;
            else
                normalized = v / total;
            end
        end
        
        function sym = makeSymmetric(A)
            % MAKESYMMETRIC 使矩阵对称
            %   sym = utils.ArrayUtils.makeSymmetric(A)
            %
            %   输入参数:
            %       A - 输入矩阵 (double matrix)
            %
            %   输出参数:
            %       sym - 对称化后的矩阵 (double matrix)
            
            sym = 0.5 * (A + A');
        end
        
        function result = ensurePositiveDefinite(A, minEigenvalue)
            % ENSUREPOSITIVEDEFINITE 确保矩阵正定
            %   result = utils.ArrayUtils.ensurePositiveDefinite(A)
            %   result = utils.ArrayUtils.ensurePositiveDefinite(A, minEigenvalue)
            %
            %   输入参数:
            %       A - 输入矩阵 (double matrix)
            %       minEigenvalue - 最小特征值 (double, 可选, 默认 1e-6)
            %
            %   输出参数:
            %       result - 正定化后的矩阵 (double matrix)
            
            arguments
                A (:,:) double
                minEigenvalue (1,1) double = 1e-6
            end
            
            [V, D] = eig(A);
            eigenvalues = diag(D);
            eigenvalues = max(eigenvalues, minEigenvalue);
            result = V * diag(eigenvalues) * V';
            result = 0.5 * (result + result');
        end
        
        function kronProduct = kronProduct(A, B)
            % KRONPRODUCT 计算Kronecker积
            %   kronProduct = utils.ArrayUtils.kronProduct(A, B)
            %
            %   输入参数:
            %       A, B - 输入矩阵 (double matrix)
            %
            %   输出参数:
            %       kronProduct - Kronecker积 (double matrix)
            
            kronProduct = kron(A, B);
        end
        
        function indices = findNonZero(array)
            % FINDNONZERO 找到非零元素的索引
            %   indices = utils.ArrayUtils.findNonZero(array)
            %
            %   输入参数:
            %       array - 输入数组 (double array)
            %
            %   输出参数:
            %       indices - 非零元素索引 (double array)
            
            indices = find(array ~= 0);
        end
        
        function result = safeDivide(numerator, denominator, defaultValue)
            % SAFEDIVIDE 安全除法，避免除零错误
            %   result = utils.ArrayUtils.safeDivide(numerator, denominator)
            %   result = utils.ArrayUtils.safeDivide(numerator, denominator, defaultValue)
            %
            %   输入参数:
            %       numerator - 分子 (double)
            %       denominator - 分母 (double)
            %       defaultValue - 除零时的默认值 (double, 可选, 默认 0)
            %
            %   输出参数:
            %       result - 除法结果或默认值 (double)
            
            arguments
                numerator double
                denominator double
                defaultValue (1,1) double = 0
            end
            
            if denominator == 0
                result = defaultValue;
            else
                result = numerator ./ denominator;
            end
        end
        
        function result = resampleWithReplacement(array, weights, numSamples)
            % RESAMPLEWITHREPLACEMENT 带权重的重采样
            %   result = utils.ArrayUtils.resampleWithReplacement(array, weights, numSamples)
            %
            %   输入参数:
            %       array - 输入数组 (double array)
            %       weights - 权重向量 (double array)
            %       numSamples - 采样数量 (double)
            %
            %   输出参数:
            %       result - 重采样结果 (double array)
            
            arguments
                array (:,1) double
                weights (:,1) double
                numSamples (1,1) double
            end
            
            cumWeights = cumsum(weights);
            cumWeights = cumWeights / cumWeights(end);
            
            result = zeros(numSamples, 1);
            for i = 1:numSamples
                r = rand();
                idx = find(cumWeights >= r, 1, 'first');
                result(i) = array(idx);
            end
        end
    end
end
