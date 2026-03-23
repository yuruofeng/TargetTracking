classdef ConfigManager < handle
    % CONFIGMANAGER 配置管理器类
    %   提供统一的配置参数管理功能，支持参数验证、默认值设置和配置文件加载
    %
    %   属性说明:
    %       config - 配置参数结构体
    %       logger - 日志记录器
    %
    %   使用示例:
    %       config = utils.ConfigManager();
    %       config.set('tracking.pD', 0.99);
    %       pD = config.get('tracking.pD');
    %       config.loadFromFile('config.mat');
    %
    %   参见: Logger
    
    properties (Access = private)
        config
        logger
        validators
    end
    
    properties (Constant)
        DEFAULT_TRACKING_CONFIG = struct(...
            'pD', 0.99, ...
            'pS', 0.99, ...
            'maxIterations', 100, ...
            'convergenceThreshold', 1e-5)
        DEFAULT_MEASUREMENT_CONFIG = struct(...
            'sigmaX', 5, ...
            'sigmaY', 10, ...
            'betaD', 25, ...
            'betaFA', 5)
        DEFAULT_AREA_CONFIG = struct(...
            'xMin', -200, ...
            'xMax', 200, ...
            'yMin', -200, ...
            'yMax', 200)
    end
    
    methods
        function obj = ConfigManager()
            % CONFIGMANAGER 构造函数
            %   obj = utils.ConfigManager()
            %
            %   输出参数:
            %       obj - ConfigManager实例
            
            obj.logger = utils.Logger('ConfigManager', 'INFO');
            obj.config = struct();
            obj.validators = containers.Map;
            obj.initializeDefaults();
        end
        
        function set(obj, key, value)
            % SET 设置配置参数
            %   obj.set('key', value)
            %   obj.set('section.subsection.key', value)
            %
            %   输入参数:
            %       key - 参数键名，支持点分格式 (char)
            %       value - 参数值 (any)
            %
            %   异常:
            %       如果参数验证失败，抛出 MException
            
            arguments
                obj
                key (1,1) string
                value
            end
            
            keyStr = char(key);
            obj.validateParam(keyStr, value);
            
            keys = strsplit(keyStr, '.');
            if length(keys) == 1
                obj.config.(keys{1}) = value;
            elseif length(keys) == 2
                if ~isfield(obj.config, keys{1})
                    obj.config.(keys{1}) = struct();
                end
                obj.config.(keys{1}).(keys{2}) = value;
            elseif length(keys) == 3
                if ~isfield(obj.config, keys{1})
                    obj.config.(keys{1}) = struct();
                end
                if ~isfield(obj.config.(keys{1}), keys{2})
                    obj.config.(keys{1}).(keys{2}) = struct();
                end
                obj.config.(keys{1}).(keys{2}).(keys{3}) = value;
            else
                error('ConfigManager:InvalidKey', ...
                    '配置键名层级过深，最多支持3层: %s', keyStr);
            end
            
            obj.logger.debug(sprintf('设置配置参数: %s = %s', ...
                keyStr, obj.valueToString(value)));
        end
        
        function value = get(obj, key, defaultValue)
            % GET 获取配置参数
            %   value = obj.get('key')
            %   value = obj.get('key', defaultValue)
            %
            %   输入参数:
            %       key - 参数键名 (char)
            %       defaultValue - 默认值 (any, 可选)
            %
            %   输出参数:
            %       value - 参数值 (any)
            %
            %   异常:
            %       如果参数不存在且未提供默认值，抛出 MException
            
            arguments
                obj
                key (1,1) string
                defaultValue = []
            end
            
            keyStr = char(key);
            keys = strsplit(keyStr, '.');
            current = obj.config;
            
            try
                for i = 1:length(keys)
                    if isstruct(current)
                        current = current.(keys{i});
                    else
                        error('ConfigManager:InvalidPath', ...
                            '配置路径无效: %s', keyStr);
                    end
                end
                value = current;
            catch
                if ~isempty(defaultValue)
                    value = defaultValue;
                    obj.logger.debug(sprintf('使用默认值: %s = %s', ...
                        keyStr, obj.valueToString(value)));
                else
                    error('ConfigManager:KeyNotFound', ...
                        '配置参数不存在: %s', keyStr);
                end
            end
        end
        
        function loadFromFile(obj, filePath)
            % LOADFROMFILE 从文件加载配置
            %   obj.loadFromFile('config.mat')
            %
            %   输入参数:
            %       filePath - 配置文件路径 (char)
            %
            %   异常:
            %       如果文件不存在或格式错误，抛出 MException
            
            arguments
                obj
                filePath (1,1) string
            end
            
            filePathStr = char(filePath);
            
            if ~exist(filePathStr, 'file')
                error('ConfigManager:FileNotFound', ...
                    '配置文件不存在: %s', filePathStr);
            end
            
            obj.logger.info(sprintf('加载配置文件: %s', filePathStr));
            
            try
                loadedConfig = load(filePathStr);
                if isfield(loadedConfig, 'config')
                    obj.config = loadedConfig.config;
                else
                    obj.config = loadedConfig;
                end
                obj.logger.info('配置文件加载成功');
            catch exc
                error('ConfigManager:LoadError', ...
                    '配置文件加载失败: %s\n%s', filePathStr, exc.message);
            end
        end
        
        function saveToFile(obj, filePath)
            % SAVETOFILE 保存配置到文件
            %   obj.saveToFile('config.mat')
            %
            %   输入参数:
            %       filePath - 配置文件路径 (char)
            
            arguments
                obj
                filePath (1,1) string
            end
            
            filePathStr = char(filePath);
            configToSave = struct('config', obj.config);
            save(filePathStr, '-struct', 'configToSave');
            obj.logger.info(sprintf('配置已保存到: %s', filePathStr));
        end
        
        function addValidator(obj, key, validatorFunc)
            % ADDVALIDATOR 添加参数验证器
            %   obj.addValidator('key', @(v) validateattributes(v, {'numeric'}, {'positive'}))
            %
            %   输入参数:
            %       key - 参数键名 (char)
            %       validatorFunc - 验证函数句柄 (function_handle)
            
            arguments
                obj
                key (1,1) string
                validatorFunc (1,1) function_handle
            end
            
            obj.validators(char(key)) = validatorFunc;
        end
        
        function config = getAll(obj)
            % GETALL 获取所有配置参数
            %   config = obj.getAll()
            %
            %   输出参数:
            %       config - 完整配置结构体 (struct)
            
            config = obj.config;
        end
        
        function reset(obj)
            % RESET 重置为默认配置
            %   obj.reset()
            
            obj.config = struct();
            obj.initializeDefaults();
            obj.logger.info('配置已重置为默认值');
        end
    end
    
    methods (Access = private)
        function initializeDefaults(obj)
            % INITIALIZEDEFAULTS 初始化默认配置
            
            obj.config.tracking = obj.DEFAULT_TRACKING_CONFIG;
            obj.config.measurement = obj.DEFAULT_MEASUREMENT_CONFIG;
            obj.config.area = obj.DEFAULT_AREA_CONFIG;
            
            obj.addValidator('tracking.pD', @(v) obj.validateProbability(v, 'pD'));
            obj.addValidator('tracking.pS', @(v) obj.validateProbability(v, 'pS'));
            obj.addValidator('measurement.sigmaX', @(v) obj.validatePositive(v, 'sigmaX'));
            obj.addValidator('measurement.sigmaY', @(v) obj.validatePositive(v, 'sigmaY'));
        end
        
        function validateParam(obj, key, value)
            % VALIDATEPARAM 验证参数值
            if isKey(obj.validators, key)
                try
                    validatorFunc = obj.validators(key);
                    validatorFunc(value);
                catch exc
                    error('ConfigManager:ValidationError', ...
                        '参数验证失败 [%s]: %s', key, exc.message);
                end
            end
        end
        
        function validateProbability(obj, value, paramName)
            % VALIDATEPROBABILITY 验证概率值
            validateattributes(value, {'numeric'}, ...
                {'scalar', '>=', 0, '<=', 1}, ...
                'ConfigManager', paramName);
        end
        
        function validatePositive(obj, value, paramName)
            % VALIDATEPOSITIVE 验证正数值
            validateattributes(value, {'numeric'}, ...
                {'scalar', 'positive'}, ...
                'ConfigManager', paramName);
        end
        
        function str = valueToString(obj, value)
            % VALUETOSTRING 将值转换为字符串
            if isnumeric(value) && isscalar(value)
                str = num2str(value);
            elseif ischar(value) || isstring(value)
                str = ['"' char(value) '"'];
            elseif islogical(value)
                str = string(value);
            else
                str = mat2str(value);
            end
        end
    end
    
    methods (Static)
        function instance = getInstance()
            % GETINSTANCE 获取单例实例
            %   config = utils.ConfigManager.getInstance()
            %
            %   输出参数:
            %       instance - ConfigManager单例实例
            
            persistent singleton;
            if isempty(singleton) || ~isvalid(singleton)
                singleton = utils.ConfigManager();
            end
            instance = singleton;
        end
    end
end
