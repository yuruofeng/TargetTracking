%% test_all.m - 综合测试脚本
%
%  功能说明:
%    对所有重构模块进行功能测试和性能测试
%
%  测试内容:
%    1. utils.Logger 日志类测试
%    2. utils.ConfigManager 配置管理类测试
%    3. utils.ArrayUtils 数组工具类测试
%    4. ggiw.GgiwFilter GGIW滤波器测试
%    5. starconvex.StarConvexTracker 星凸目标跟踪器测试
%    6. phd.ExtendedTargetPhdFilter 扩展目标PHD滤波器测试
%
%  作者: 重构版本
%  日期: 2026-03-01

%% ==================== 测试初始化 ====================

close all;
clc;
clear;

scriptPath = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptPath);
addpath(projectRoot);

testResults = struct();
testResults.totalTests = 0;
testResults.passedTests = 0;
testResults.failedTests = 0;
testResults.testDetails = {};

totalStartTime = tic;

fprintf('\n');
fprintf('========================================\n');
fprintf('  MATLAB 项目重构综合测试\n');
fprintf('  测试时间: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('========================================\n\n');

%% ==================== 测试1: Logger类 ====================

fprintf('【测试1】Logger 日志类测试...\n');
testStartTime = tic;

try
    logger = utils.Logger('TestLogger', 'DEBUG');
    
    logger.debug('调试消息测试');
    logger.info('信息消息测试');
    logger.warning('警告消息测试');
    
    logger.logFunctionStart('testFunction');
    pause(0.01);
    logger.logFunctionEnd('testFunction', toc(testStartTime));
    
    logger.setLogLevel('ERROR');
    assert(strcmp(logger.logLevel, 'ERROR') || logger.logLevel == 4, ...
        '日志级别设置失败');
    
    testResults.passedTests = testResults.passedTests + 1;
    fprintf('  [通过] Logger类功能正常\n');
    testDetails = struct('name', 'Logger类', 'status', '通过', ...
        'time', toc(testStartTime));
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    fprintf('  [失败] Logger类测试失败: %s\n', exc.message);
    testDetails = struct('name', 'Logger类', 'status', '失败', ...
        'time', toc(testStartTime), 'error', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end+1} = testDetails;

%% ==================== 测试2: ConfigManager类 ====================

fprintf('\n【测试2】ConfigManager 配置管理类测试...\n');
testStartTime = tic;

try
    config = utils.ConfigManager();
    
    config.set('test.param1', 0.99);
    value = config.get('test.param1');
    assert(abs(value - 0.99) < 1e-10, '参数设置/获取失败');
    
    defaultValue = config.get('test.nonexistent', 42);
    assert(defaultValue == 42, '默认值获取失败');
    
    pD = config.get('tracking.pD', 0.99);
    assert(pD > 0 && pD <= 1, '默认配置获取失败');
    
    config.reset();
    
    testResults.passedTests = testResults.passedTests + 1;
    fprintf('  [通过] ConfigManager类功能正常\n');
    testDetails = struct('name', 'ConfigManager类', 'status', '通过', ...
        'time', toc(testStartTime));
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    fprintf('  [失败] ConfigManager类测试失败: %s\n', exc.message);
    testDetails = struct('name', 'ConfigManager类', 'status', '失败', ...
        'time', toc(testStartTime), 'error', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end+1} = testDetails;

%% ==================== 测试3: ArrayUtils类 ====================

fprintf('\n【测试3】ArrayUtils 数组工具类测试...\n');
testStartTime = tic;

try
    testVector = [1, 2, 3, 4];
    normalized = utils.ArrayUtils.normalizeVector(testVector);
    assert(abs(sum(normalized) - 1) < 1e-10, '向量归一化失败');
    
    testMatrix = [1, 2; 3, 4];
    symMatrix = utils.ArrayUtils.makeSymmetric(testMatrix);
    assert(isequal(symMatrix, 0.5 * (testMatrix + testMatrix')), ...
        '矩阵对称化失败');
    
    posDefMatrix = utils.ArrayUtils.ensurePositiveDefinite([1, 2; 2, 1], 1e-6);
    eigenvalues = eig(posDefMatrix);
    assert(all(eigenvalues > 0), '正定化失败');
    
    result = utils.ArrayUtils.safeDivide(10, 2);
    assert(result == 5, '安全除法失败');
    
    result = utils.ArrayUtils.safeDivide(10, 0, -1);
    assert(result == -1, '除零处理失败');
    
    testResults.passedTests = testResults.passedTests + 1;
    fprintf('  [通过] ArrayUtils类功能正常\n');
    testDetails = struct('name', 'ArrayUtils类', 'status', '通过', ...
        'time', toc(testStartTime));
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    fprintf('  [失败] ArrayUtils类测试失败: %s\n', exc.message);
    testDetails = struct('name', 'ArrayUtils类', 'status', '失败', ...
        'time', toc(testStartTime), 'error', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end+1} = testDetails;

%% ==================== 测试4: GGIW滤波器 ====================

fprintf('\n【测试4】GGIW GgiwFilter 滤波器测试...\n');
testStartTime = tic;

try
    config = utils.ConfigManager();
    filter = ggiw.GgiwFilter(config);
    
    assert(filter.numComponents == 0, '初始组件数应为0');
    
    F = [1, 1, 0; 0, 1, 1; 0, 0, 1];
    Q = diag([0, 0, 0.1]);
    filter.predict(F, Q);
    
    assert(filter.numComponents > 0, '预测后应有组件');
    
    testMeasurements = struct('points', {randn(2, 5) * 10});
    H = [1, 0, 0];
    R = eye(2) * 10;
    filter.update({testMeasurements}, H, R);
    
    filter.pruneAndMerge(1e-10, 4, 100);
    
    estimates = filter.extractStates(0.1);
    
    testResults.passedTests = testResults.passedTests + 1;
    fprintf('  [通过] GgiwFilter功能正常\n');
    testDetails = struct('name', 'GgiwFilter', 'status', '通过', ...
        'time', toc(testStartTime));
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    fprintf('  [失败] GgiwFilter测试失败: %s\n', exc.message);
    testDetails = struct('name', 'GgiwFilter', 'status', '失败', ...
        'time', toc(testStartTime), 'error', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end+1} = testDetails;

%% ==================== 测试5: 星凸目标跟踪器 ====================

fprintf('\n【测试5】StarConvexTracker 跟踪器测试...\n');
testStartTime = tic;

try
    numFourierCoeff = 11;
    tracker = starconvex.StarConvexTracker(numFourierCoeff, 'UKF');
    
    initialState = [100; 100; 5; -8; zeros(numFourierCoeff, 1)];
    initialState(5) = 100;
    initialCov = diag([100, 100, 10, 10, ones(1, numFourierCoeff) * 0.1]);
    
    tracker.initialize(initialState, initialCov);
    
    F = blkdiag([eye(2), 10*eye(2); zeros(2,2), eye(2)], eye(numFourierCoeff));
    Q = blkdiag(diag([10, 10, 1, 1]), diag(ones(1, numFourierCoeff) * 0.1));
    tracker.predict(F, Q);
    
    measurement = [105; 102];
    tracker.update(measurement, [0.7; 0; 0], diag([0.01, 0.1, 0.1]));
    
    position = tracker.getPosition();
    assert(length(position) == 2, '位置向量维度错误');
    
    velocity = tracker.getVelocity();
    assert(length(velocity) == 2, '速度向量维度错误');
    
    phiVector = linspace(0, 2*pi, 100);
    shape = tracker.getShape(phiVector);
    assert(size(shape, 1) == 2, '形状向量维度错误');
    
    testResults.passedTests = testResults.passedTests + 1;
    fprintf('  [通过] StarConvexTracker功能正常\n');
    testDetails = struct('name', 'StarConvexTracker', 'status', '通过', ...
        'time', toc(testStartTime));
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    fprintf('  [失败] StarConvexTracker测试失败: %s\n', exc.message);
    testDetails = struct('name', 'StarConvexTracker', 'status', '失败', ...
        'time', toc(testStartTime), 'error', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end+1} = testDetails;

%% ==================== 测试6: 扩展目标PHD滤波器 ====================

fprintf('\n【测试6】ExtendedTargetPhdFilter 滤波器测试...\n');
testStartTime = tic;

try
    config = utils.ConfigManager();
    filter = phd.ExtendedTargetPhdFilter(config);
    
    assert(filter.numComponents == 0, '初始组件数应为0');
    
    F = [1, 0, 1, 0; 0, 1, 0, 1; 0, 0, 1, 0; 0, 0, 0, 1];
    Q = diag([1, 1, 0.1, 0.1]);
    filter.predict(F, Q);
    
    assert(filter.numComponents > 0, '预测后应有组件');
    
    testCell = randn(2, 3) * 10;
    testPartition = struct('cells', {{testCell}}, 'numCells', 1);
    H = [1, 0, 0, 0; 0, 1, 0, 0];
    R = eye(2) * 10;
    filter.update({testPartition}, H, R);
    
    filter.pruneAndMerge(1e-10, 4, 100);
    
    estimates = filter.extractStates(0.1);
    
    testResults.passedTests = testResults.passedTests + 1;
    fprintf('  [通过] ExtendedTargetPhdFilter功能正常\n');
    testDetails = struct('name', 'ExtendedTargetPhdFilter', 'status', '通过', ...
        'time', toc(testStartTime));
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    fprintf('  [失败] ExtendedTargetPhdFilter测试失败: %s\n', exc.message);
    testDetails = struct('name', 'ExtendedTargetPhdFilter', 'status', '失败', ...
        'time', toc(testStartTime), 'error', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end+1} = testDetails;

%% ==================== 测试7: 测量生成函数 ====================

fprintf('\n【测试7】测量生成函数测试...\n');
testStartTime = tic;

try
    areaBounds = [-200, 200, -200, 200];
    clutter = ggiw.generateClutter(5e-6, areaBounds, 10);
    assert(length(clutter) == 10, '杂波生成时间步数错误');
    
    trueState = [0; 0; 5; 5; 0; 0];
    measurements = ggiw.generateExtendedMeasurements(trueState, 20, 0.99, ...
        diag([100, 50]), 10);
    assert(length(measurements) == 10, '测量生成时间步数错误');
    
    testResults.passedTests = testResults.passedTests + 1;
    fprintf('  [通过] 测量生成函数功能正常\n');
    testDetails = struct('name', '测量生成函数', 'status', '通过', ...
        'time', toc(testStartTime));
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    fprintf('  [失败] 测量生成函数测试失败: %s\n', exc.message);
    testDetails = struct('name', '测量生成函数', 'status', '失败', ...
        'time', toc(testStartTime), 'error', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end+1} = testDetails;

%% ==================== 测试8: 划分函数 ====================

fprintf('\n【测试8】测量划分函数测试...\n');
testStartTime = tic;

try
    measurements = [0, 0, 100, 100, 50, 50; 0, 10, 100, 110, 50, 60];
    partitions = phd.partitionMeasurementSet(measurements, 50, 10);
    
    assert(partitions.numCells > 0, '划分单元数应大于0');
    
    singleMeas = [0; 0];
    partitions = phd.partitionMeasurementSet(singleMeas, 50, 10);
    assert(partitions.numCells == 1, '单个测量应产生一个单元');
    
    testResults.passedTests = testResults.passedTests + 1;
    fprintf('  [通过] 测量划分函数功能正常\n');
    testDetails = struct('name', '测量划分函数', 'status', '通过', ...
        'time', toc(testStartTime));
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    fprintf('  [失败] 测量划分函数测试失败: %s\n', exc.message);
    testDetails = struct('name', '测量划分函数', 'status', '失败', ...
        'time', toc(testStartTime), 'error', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end+1} = testDetails;

%% ==================== 测试结果汇总 ====================

totalTestTime = toc(totalStartTime);

fprintf('\n');
fprintf('========================================\n');
fprintf('  测试结果汇总\n');
fprintf('========================================\n');
fprintf('  总测试数: %d\n', testResults.totalTests);
fprintf('  通过数: %d\n', testResults.passedTests);
fprintf('  失败数: %d\n', testResults.failedTests);
fprintf('  通过率: %.1f%%\n', testResults.passedTests / testResults.totalTests * 100);
fprintf('  总耗时: %.2f 秒\n', totalTestTime);
fprintf('========================================\n\n');

testResults.totalTime = totalTestTime;
testResults.passRate = testResults.passedTests / testResults.totalTests * 100;

save('test_results.mat', 'testResults');

if testResults.failedTests == 0
    fprintf('所有测试通过！\n');
else
    fprintf('存在失败的测试，请检查详细结果。\n');
end
