classdef FilterFactory
% DBT.FILTERFACTORY  滤波器工厂类。
%   提供统一的滤波器创建接口，支持各种滤波器类型的实例化。
%   遵循工厂设计模式，简化滤波器配置和创建过程。
%
%   支持的滤波器类型：
%       'EKF'     - 扩展卡尔曼滤波器（CT模型）
%       'UKF'     - 无迹卡尔曼滤波器
%       'CKF'     - 容积卡尔曼滤波器
%       'EKF-CV'  - CV模型EKF
%       'EKF-CA'  - CA模型EKF
%       'EKF-CT'  - CT模型EKF
%       'EKF-Singer' - Singer模型EKF
%       'EKF-CS'  - 当前统计模型EKF
%       'IMM'     - 交互多模型滤波器
%       'PF'      - 粒子滤波器
%
%   使用方法：
%       filter = dbt.FilterFactory.create('EKF', config);
%       filter = dbt.FilterFactory.create('IMM', immConfig);
%       filter = dbt.FilterFactory.createEKF('CV', dt, q);
%
%   See also: dbt.EKF, dbt.UKF, dbt.CKF, dbt.IMM, dbt.ParticleFilter

    properties (Constant)
        SUPPORTED_TYPES = {'EKF', 'UKF', 'CKF', 'IMM', 'PF', ...
                          'EKF-CV', 'EKF-CA', 'EKF-CT', 'EKF-Singer', 'EKF-CS'}
    end

    methods (Static)
        function filter = create(filterType, varargin)
        % CREATE  创建指定类型的滤波器。
        %
        %   语法：
        %       filter = create('EKF', config)
        %       filter = create('UKF', config)
        %       filter = create('CKF', config)
        %       filter = create('IMM', immConfig)
        %       filter = create('PF', pfConfig)
        %       filter = create('EKF-CV', dt, q)
        %       filter = create('EKF-CA', dt, q)
        %       filter = create('EKF-CT', dt, q, omega)
        %
        %   输入参数：
        %       filterType - 滤波器类型字符串
        %       varargin   - 类型特定参数
        %
        %   输出参数：
        %       filter - 创建的滤波器对象
            switch upper(filterType)
                case 'EKF'
                    filter = dbt.FilterFactory.createEKFFromConfig(varargin{:});
                    
                case 'UKF'
                    filter = dbt.FilterFactory.createUKF(varargin{:});
                    
                case 'CKF'
                    filter = dbt.FilterFactory.createCKF(varargin{:});
                    
                case 'IMM'
                    filter = dbt.FilterFactory.createIMM(varargin{:});
                    
                case 'PF'
                    filter = dbt.FilterFactory.createPF(varargin{:});
                    
                case {'EKF-CV', 'EKF-CA', 'EKF-CT', 'EKF-SINGER', 'EKF-CS'}
                    filter = dbt.FilterFactory.createMotionModelEKF(filterType, varargin{:});
                    
                otherwise
                    error('dbt:UnsupportedType', ...
                        '不支持的滤波器类型: %s。支持的类型: %s', ...
                        filterType, strjoin(dbt.FilterFactory.SUPPORTED_TYPES, ', '));
            end
        end
        
        function filter = createEKFFromConfig(config)
        % CREATEEKFFROMCONFIG  从配置创建EKF。
            if nargin < 1
                config = dbt.Config();
            end
            filter = dbt.EKF(config);
        end
        
        function filter = createUKF(config)
        % CREATEUKF  创建UKF滤波器。
            if nargin < 1
                config = dbt.Config();
            end
            filter = dbt.UKF(config);
        end
        
        function filter = createCKF(config)
        % CREATECKF  创建CKF滤波器。
            if nargin < 1
                config = dbt.Config();
            end
            filter = dbt.CKF(config);
        end
        
        function filter = createIMM(config)
        % CREATEIMM  创建IMM滤波器。
            if nargin < 1
                config = dbt.ConfigIMM();
            end
            filter = dbt.IMM(config);
        end
        
        function filter = createPF(config)
        % CREATEPF  创建粒子滤波器。
            if nargin < 1
                config = dbt.Config();
            end
            filter = dbt.ParticleFilter(config);
        end
        
        function filter = createMotionModelEKF(modelType, varargin)
        % CREATEMotionMODEL EKF  创建运动模型EKF。
        %
        %   语法：
        %       filter = createMotionModelEKF('EKF-CV', dt, q)
        %       filter = createMotionModelEKF('EKF-CA', dt, q)
        %       filter = createMotionModelEKF('EKF-CT', dt, q, omega)
        %       filter = createMotionModelEKF('EKF-Singer', dt, q, tau)
        %       filter = createMotionModelEKF('EKF-CS', dt, q, tau)
        %
        %   或者使用配置对象：
        %       filter = createMotionModelEKF('EKF-CV', config)
            modelType = upper(modelType);
            modelType = strrep(modelType, 'EKF-', '');
            
            if nargin >= 2 && isa(varargin{1}, 'dbt.MotionModelConfig')
                config = varargin{1};
            else
                config = dbt.FilterFactory.parseMotionConfig(modelType, varargin{:});
            end
            
            filter = dbt.MotionModelEKF(config);
        end
        
        function filter = createDefaultIMM(modelTypes)
        % CREATEDEFAULTIMM  创建默认IMM配置。
        %
        %   输入参数：
        %       modelTypes - 模型类型元胞数组，如 {'CV', 'CA', 'CT'}
        %
        %   输出参数：
        %       filter - IMM滤波器对象
            if nargin < 1
                modelTypes = {'CV', 'CA', 'CT'};
            end
            
            filter = dbt.IMM();  % 使用默认ConfigIMM配置
        end
        
        function config = parseMotionConfig(modelType, varargin)
        % PARSEMOTIONCONFIG  解析运动模型配置参数。
        %
        %   输入参数：
        %       modelType - 模型类型 ('CV', 'CA', 'CT', 'Singer', 'CS')
        %       varargin  - 参数对 (dt, value, q, value, ...)
        %
        %   输出参数：
        %       config - MotionModelConfig对象
            p = inputParser;
            addParameter(p, 'dt', 1.0, @isscalar);
            addParameter(p, 'q', 1.0, @isscalar);
            addParameter(p, 'omega', 0.1, @isscalar);
            addParameter(p, 'tau', 10.0, @isscalar);
            addParameter(p, 'sigma_a', 5.0, @isscalar);
            parse(p, varargin{:});
            
            config = dbt.MotionModelConfig(modelType, ...
                'dt', p.Results.dt, ...
                'q', p.Results.q);
            
            if strcmp(modelType, 'CT')
                config.omega = p.Results.omega;
            elseif strcmp(modelType, 'Singer') || strcmp(modelType, 'CS')
                config.tau = p.Results.tau;
                config.sigma_a = p.Results.sigma_a;
            end
        end
        
        function types = getSupportedTypes()
        % GETSUPPORTEDTYPES  获取支持的滤波器类型列表。
            types = dbt.FilterFactory.SUPPORTED_TYPES;
        end
        
        function displayInfo(filterType)
        % DISPLAYINFO  显示滤波器类型信息。
            if nargin < 1
                fprintf('支持的滤波器类型:\n');
                for i = 1:length(dbt.FilterFactory.SUPPORTED_TYPES)
                    fprintf('  - %s\n', dbt.FilterFactory.SUPPORTED_TYPES{i});
                end
            else
                switch upper(filterType)
                    case 'EKF'
                        fprintf('EKF: 扩展卡尔曼滤波器，适用于CT模型\n');
                    case 'UKF'
                        fprintf('UKF: 无迹卡尔曼滤波器，使用sigma点\n');
                    case 'CKF'
                        fprintf('CKF: 容积卡尔曼滤波器，使用cubature点\n');
                    case 'IMM'
                        fprintf('IMM: 交互多模型滤波器，适用于机动目标\n');
                    case 'PF'
                        fprintf('PF: 粒子滤波器，适用于非线性非高斯系统\n');
                    otherwise
                        fprintf('未知滤波器类型: %s\n', filterType);
                end
            end
        end
    end
end
