classdef Logger < handle
    % LOGGER 日志记录器类
    %   提供统一的日志记录功能，支持不同日志级别和格式化输出
    %
    %   属性说明:
    %       moduleName - 模块名称
    %       logLevel - 当前日志级别
    %       logFile - 日志文件路径（可选）
    %
    %   使用示例:
    %       logger = utils.Logger('GGIW_PHD', 'INFO');
    %       logger.info('处理开始');
    %       logger.error('处理失败', exception);
    %
    %   参见: ConfigManager
    
    properties (Access = private)
        moduleName
        logLevel
        logFile
        fileHandle
    end
    
    properties (Constant)
        LEVEL_DEBUG = 1
        LEVEL_INFO = 2
        LEVEL_WARNING = 3
        LEVEL_ERROR = 4
        LEVEL_NONE = 5
    end
    
    methods
        function obj = Logger(moduleName, logLevel, logFile)
            % LOGGER 构造函数
            %   obj = utils.Logger(moduleName, logLevel)
            %   obj = utils.Logger(moduleName, logLevel, logFile)
            %
            %   输入参数:
            %       moduleName - 模块名称 (char, 标量)
            %       logLevel - 日志级别: 'DEBUG', 'INFO', 'WARNING', 'ERROR', 'NONE' (char, 标量)
            %       logFile - 日志文件路径 (char, 可选)
            %
            %   示例:
            %       logger = utils.Logger('MyModule', 'INFO');
            %       logger = utils.Logger('MyModule', 'DEBUG', 'log.txt');
            
            arguments
                moduleName (1,1) string
                logLevel (1,1) string = 'INFO'
                logFile (1,1) string = ''
            end
            
            obj.moduleName = char(moduleName);
            obj.logLevel = obj.parseLogLevel(char(logLevel));
            obj.logFile = char(logFile);
            
            if ~isempty(obj.logFile)
                obj.fileHandle = fopen(obj.logFile, 'a');
                if obj.fileHandle == -1
                    warning('Logger:FileOpenError', ...
                        '无法打开日志文件: %s', obj.logFile);
                    obj.fileHandle = [];
                end
            else
                obj.fileHandle = [];
            end
        end
        
        function delete(obj)
            % DELETE 析构函数，关闭文件句柄
            if ~isempty(obj.fileHandle)
                fclose(obj.fileHandle);
            end
        end
        
        function debug(obj, message, varargin)
            % DEBUG 输出调试级别日志
            %   obj.debug(message)
            %   obj.debug(message, exception)
            %
            %   输入参数:
            %       message - 日志消息 (char)
            %       varargin - 可选异常对象 (MException)
            
            obj.log('DEBUG', obj.LEVEL_DEBUG, message, varargin{:});
        end
        
        function info(obj, message, varargin)
            % INFO 输出信息级别日志
            %   obj.info(message)
            %
            %   输入参数:
            %       message - 日志消息 (char)
            %       varargin - 可选异常对象 (MException)
            
            obj.log('INFO', obj.LEVEL_INFO, message, varargin{:});
        end
        
        function warning(obj, message, varargin)
            % WARNING 输出警告级别日志
            %   obj.warning(message)
            %
            %   输入参数:
            %       message - 日志消息 (char)
            %       varargin - 可选异常对象 (MException)
            
            obj.log('WARNING', obj.LEVEL_WARNING, message, varargin{:});
        end
        
        function error(obj, message, varargin)
            % ERROR 输出错误级别日志
            %   obj.error(message)
            %   obj.error(message, exception)
            %
            %   输入参数:
            %       message - 日志消息 (char)
            %       varargin - 可选异常对象 (MException)
            
            obj.log('ERROR', obj.LEVEL_ERROR, message, varargin{:});
        end
        
        function logFunctionStart(obj, funcName)
            % LOGFUNCTIONSTART 记录函数开始执行
            %   obj.logFunctionStart('functionName')
            %
            %   输入参数:
            %       funcName - 函数名称 (char)
            
            obj.debug(sprintf('函数 [%s] 开始执行', funcName));
        end
        
        function logFunctionEnd(obj, funcName, elapsedTime)
            % LOGFUNCTIONEND 记录函数执行结束
            %   obj.logFunctionEnd('functionName', elapsedTime)
            %
            %   输入参数:
            %       funcName - 函数名称 (char)
            %       elapsedTime - 执行耗时（秒） (double)
            
            obj.debug(sprintf('函数 [%s] 执行完成，耗时: %.4f 秒', ...
                funcName, elapsedTime));
        end
        
        function setLogLevel(obj, level)
            % SETLOGLEVEL 设置日志级别
            %   obj.setLogLevel('DEBUG')
            %
            %   输入参数:
            %       level - 日志级别字符串 (char)
            
            obj.logLevel = obj.parseLogLevel(level);
        end
    end
    
    methods (Access = private)
        function level = parseLogLevel(obj, levelStr)
            % PARSELOGLEVEL 解析日志级别字符串
            switch upper(levelStr)
                case 'DEBUG'
                    level = obj.LEVEL_DEBUG;
                case 'INFO'
                    level = obj.LEVEL_INFO;
                case 'WARNING'
                    level = obj.LEVEL_WARNING;
                case 'ERROR'
                    level = obj.LEVEL_ERROR;
                case 'NONE'
                    level = obj.LEVEL_NONE;
                otherwise
                    warning('Logger:InvalidLevel', ...
                        '无效的日志级别: %s，使用默认级别 INFO', levelStr);
                    level = obj.LEVEL_INFO;
            end
        end
        
        function log(obj, levelStr, level, message, varargin)
            % LOG 内部日志输出方法
            if level < obj.logLevel
                return;
            end
            
            timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
            formattedMsg = sprintf('[%s] [%s] [%s] %s', ...
                timestamp, obj.moduleName, levelStr, message);
            
            if nargin > 5 && ~isempty(varargin{1})
                if isa(varargin{1}, 'MException')
                    exc = varargin{1};
                    formattedMsg = sprintf('%s\n  异常: %s: %s', ...
                        formattedMsg, exc.identifier, exc.message);
                end
            end
            
            fprintf('%s\n', formattedMsg);
            
            if ~isempty(obj.fileHandle)
                fprintf(obj.fileHandle, '%s\n', formattedMsg);
            end
        end
    end
    
    methods (Static)
        function logger = getLogger(moduleName, logLevel)
            % GETLOGGER 获取或创建日志记录器实例
            %   logger = utils.Logger.getLogger('ModuleName')
            %   logger = utils.Logger.getLogger('ModuleName', 'DEBUG')
            %
            %   输入参数:
            %       moduleName - 模块名称 (char)
            %       logLevel - 日志级别 (char, 可选，默认 'INFO')
            %
            %   输出参数:
            %       logger - Logger实例 (Logger)
            
            arguments
                moduleName (1,1) string
                logLevel (1,1) string = 'INFO'
            end
            
            persistent loggers;
            if isempty(loggers)
                loggers = containers.Map;
            end
            
            key = char(moduleName);
            if isKey(loggers, key)
                logger = loggers(key);
            else
                logger = utils.Logger(moduleName, logLevel);
                loggers(key) = logger;
            end
        end
    end
end
