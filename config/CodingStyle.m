% CODINGSTYLE  项目编码规范
%   定义 SingleTargetTracking 项目的统一编码风格标准。
%
%   目录：
%       1. 命名规范
%       2. 文档注释规范
%       3. 代码格式规范
%       4. 类设计规范
%       5. 函数设计规范
%
%   遵循原则：
%       - 一致性优于个人偏好
%       - 可读性优于简洁性
%       - 约定优于配置

classdef CodingStyle
    properties (Constant)
        
        %% ========== 1. 命名规范 ==========
        
        % 类名：PascalCase（大驼峰式）
        %   示例：BaseFilter, KalmanFilter, ImmPhdFilter
        %   规则：
        %     - 以大写字母开头
        %     - 每个单词首字母大写
        %     - 避免缩写，除非是广泛认可的（如EKF, UKF, PHD）
        
        % 函数/方法名：camelCase（小驼峰式）
        %   示例：predictStep, updateState, computeLikelihood
        %   规则：
        %     - 以小写字母开头
        %     - 后续单词首字母大写
        %     - 动词开头表示动作（get, set, compute, update, init）
        
        % 变量名：camelCase（小驼峰式）
        %   示例：stateDim, measDim, numModels
        %   规则：
        %     - 循环变量可用单字母：i, j, k
        %     - 布尔变量以is/has开头：isValid, hasMeasurement
        %     - 临时变量可简短：tmp, temp
        
        % 常量：UPPER_SNAKE_CASE（大写下划线式）
        %   示例：MAX_ITERATIONS, DEFAULT_THRESHOLD
        %   规则：
        %     - 全部大写
        %     - 单词间用下划线分隔
        
        % 私有方法：前缀下划线或Access=private
        %   示例：computeStep_, validateInput
        
        %% ========== 2. 文档注释规范 ==========
        
        % 类文档模板：
        %   classdef ClassName
        %   % PACKAGE.CLASSNAME  简短描述（中英双语）
        %   %   详细描述类的作用和用法。
        %   %
        %   %   属性列表：
        %   %       propName - 属性描述
        %   %
        %   %   使用方法：
        %   %       obj = ClassName(config);
        %   %       result = obj.run(data);
        %   %
        %   %   参考文献：
        %   %       [1] 作者, "标题", 期刊, 年份.
        %   %
        %   %   See also: RelatedClass1, RelatedClass2
        
        % 方法文档模板：
        %   function result = methodName(obj, input1, input2)
        %   % METHODNAME  简短描述
        %   %   详细描述方法的作用。
        %   %
        %   %   输入参数：
        %   %       input1 - 参数1描述 [维度]
        %   %       input2 - 参数2描述 [维度]
        %   %
        %   %   输出参数：
        %   %       result - 结果描述 [维度]
        %
        %   end
        
        %% ========== 3. 代码格式规范 ==========
        
        % 缩进：
        %   - 使用4个空格，不使用Tab
        %   - switch-case中case缩进4空格
        
        % 空格：
        %   - 运算符两侧加空格：a = b + c
        %   - 逗号后加空格：[a, b, c]
        %   - 分号后不加空格（行内）
        %   - 函数名与括号间不加空格：func(a, b)
        %   - 关键字后加空格：if x, for i = 1:n
        
        % 空行：
        %   - 方法之间空一行
        %   - 逻辑块之间空一行
        %   - 属性块与方法块之间空两行
        
        % 行长度：
        %   - 最大80-100字符
        %   - 超长行使用...续行
        
        % 括号风格：
        %   - function声明：function result = name(obj, arg)
        %   - if/for/while：关键字后空格，语句在同一行或下一行
        %   - 短语句可同行：if x > 0, y = 1; end
        
        %% ========== 4. 类设计规范 ==========
        
        % 属性定义：
        %   - 使用properties块分组相关属性
        %   - 添加行内注释说明用途
        %   - 示例：
        %       properties
        %           config    % 配置对象
        %           stateDim  % 状态维度
        %       end
        
        % 访问控制：
        %   - 默认使用public
        %   - 内部方法使用Access = protected
        %   - 工具方法使用Static
        
        % 构造函数：
        %   - 允许无参数调用（使用默认配置）
        %   - 使用nargin检查可选参数
        
        %% ========== 5. 函数设计规范 ==========
        
        % 参数验证：
        %   - 使用validateattributes或自定义检查
        %   - 在函数开头验证输入
        
        % 错误处理：
        %   - 使用error('package:ErrorType', 'Message')
        %   - 提供有意义的错误信息
        
        % 返回值：
        %   - 多返回值优于返回结构体
        %   - 使用varargout处理可选输出
        
        %% ========== 6. 专用术语规范 ==========
        
        % 滤波器相关：
        %   state     - 状态向量（不用x表示状态）
        %   covar     - 协方差矩阵（不用P表示协方差）
        %   meas      - 测量（不用z表示测量）
        %   pred      - 预测值（后缀）
        %   upd       - 更新值（后缀）
        %   innov     - 新息/残差
        
        % 矩阵维度：
        %   nStates   - 状态数量
        %   nMeas     - 测测数量
        %   nModels   - 模型数量
        %   nSteps    - 时间步数
        %   nComps    - 高斯分量数
        
        % 希腊字母拼写：
        %   mu        - 均值（不用miu）
        %   sigma     - 标准差
        %   omega     - 角速度
        %   lambda    - 参数
        
    end
end
