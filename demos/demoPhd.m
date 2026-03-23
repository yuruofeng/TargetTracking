%% demoPhd.m  IMM-PHD vs SIMM-PHD 多模型滤波器演示脚本
%  功能：
%    1. 生成目标真实轨迹和量测
%    2. 分别运行 IMM-PHD 和 SIMM-PHD 滤波器
%    3. 计算 OSPA 指标和势估计
%    4. 绘制对比结果图
%
%  用法：直接运行本脚本即可

clear; close all; clc;
tic;

fprintf('\n');
fprintf('============================================================\n');
fprintf('          PHD 滤波器演示 (IMM-PHD vs SIMM-PHD)\n');
fprintf('============================================================\n\n');

%% ===== 用户可配置参数 =====
nMC     = 20;       % 蒙特卡洛仿真次数
ospaC   = 30;      % OSPA 截断参数 c
ospaP   = 1;       % OSPA 阶数参数 p
saveData = false;  % 是否保存结果
savePath = 'results_phd.mat';

%% ===== 生成模型和真实轨迹 =====
fprintf('正在生成模型参数...\n');
model = phd.Config();

fprintf('正在生成真实轨迹...\n');
scenario = phd.Scenario(model);
scenario = scenario.generateTruth();
truth.X = scenario.X;
truth.N = scenario.N;
truth.K = scenario.K;
truth.track_list = scenario.track_list;
truth.total_tracks = scenario.total_tracks;

[xTrack, kBirth, kDeath] = extractTracks(truth.X, truth.track_list, truth.total_tracks);

%% ===== 绘制真实轨迹 =====
fprintf('正在绘制真实轨迹...\n');
plotGroundTruth(model, truth, xTrack, kBirth, kDeath);

%% ===== 蒙特卡洛仿真 =====
fprintf('\n开始蒙特卡洛仿真 (%d 次)...\n', nMC);

mcImmX     = cell(nMC, truth.K);
mcImmN     = zeros(truth.K, nMC);
mcImmMiu   = cell(nMC, 1);
ospaImm    = zeros(truth.K, 3, nMC);

mcSimmX    = cell(nMC, truth.K);
mcSimmN    = zeros(truth.K, nMC);
mcSimmMiu  = cell(nMC, 1);
ospaSimm   = zeros(truth.K, 3, nMC);

hWait = waitbar(0, '正在处理，请稍候...');
for iMC = 1:nMC
    scenarioMc = phd.Scenario(model);
    scenarioMc = scenarioMc.generateTruth();
    truthMc.X = scenarioMc.X;
    truthMc.N = scenarioMc.N;
    truthMc.K = scenarioMc.K;
    truthMc.track_list = scenarioMc.track_list;
    truthMc.total_tracks = scenarioMc.total_tracks;
    
    scenarioMc = scenarioMc.generateMeasurements(truthMc);
    meas.Z = scenarioMc.Z;
    meas.K = scenarioMc.K;

    % --- IMM-PHD 滤波
    immFilter = phd.ImmPhdFilter(model);
    est = immFilter.run(meas, truthMc);
    mcImmX(iMC, :) = est.IMMX';
    mcImmN(:, iMC) = est.IMMN;
    mcImmMiu{iMC} = est.miu;
    for k = 1:meas.K
        [ospaImm(k,1,iMC), ospaImm(k,2,iMC), ospaImm(k,3,iMC)] = ...
            utils.OspaMetric.compute(getComps(truthMc.X{k}, [1,4]), getComps(est.IMMX{k}, [1,4]), ospaC, ospaP);
    end

    % --- SIMM-PHD 滤波
    simmFilter = phd.SimmPhdFilter(model);
    estS = simmFilter.run(meas, truthMc);
    mcSimmX(iMC, :) = estS.IMMX';
    mcSimmN(:, iMC) = estS.IMMN;
    mcSimmMiu{iMC} = estS.miu;
    for k = 1:meas.K
        [ospaSimm(k,1,iMC), ospaSimm(k,2,iMC), ospaSimm(k,3,iMC)] = ...
            utils.OspaMetric.compute(getComps(truthMc.X{k}, [1,4]), getComps(estS.IMMX{k}, [1,4]), ospaC, ospaP);
    end

    waitbar(iMC/nMC, hWait, sprintf('已完成 %d%%', round(iMC/nMC*100)));
end
close(hWait);

%% ===== MC 均值计算 =====
fprintf('正在计算蒙特卡洛均值...\n');
immXaver  = computeMcAverageX(mcImmX, nMC, truth.K);
simmXaver = computeMcAverageX(mcSimmX, nMC, truth.K);

ospaImmMean  = mean(ospaImm, 3);
ospaSimmMean = mean(ospaSimm, 3);

immNaver  = round(mean(mcImmN, 2));
simmNaver = round(mean(mcSimmN, 2));

immMiuAver  = computeMcAverageMiu(mcImmMiu, nMC);
simmMiuAver = computeMcAverageMiu(mcSimmMiu, nMC);
colSums = sum(simmMiuAver, 1);
simmMiuAver = simmMiuAver ./ colSums;

%% ===== 绘图 =====
fprintf('正在绘制结果...\n');
plotTracksAndEstimates(model, truth, meas, xTrack, kBirth, kDeath, immXaver, simmXaver);
plotOspa(meas.K, ospaImmMean, ospaSimmMean, ospaC);
plotCardinality(meas.K, truth.N, immNaver, simmNaver);
plotModelProbability(meas.K, immMiuAver, 'IMM-PHD');
plotModelProbability(meas.K, simmMiuAver, 'SIMM-PHD');

%% ===== 打印统计结果 =====
fprintf('\n===== 统计结果 =====\n');
fprintf('平均 OSPA 距离:\n');
fprintf('  IMM-PHD:  %.2f (Loc: %.2f, Card: %.2f)\n', mean(ospaImmMean(:,1)), mean(ospaImmMean(:,2)), mean(ospaImmMean(:,3)));
fprintf('  SIMM-PHD: %.2f (Loc: %.2f, Card: %.2f)\n', mean(ospaSimmMean(:,1)), mean(ospaSimmMean(:,2)), mean(ospaSimmMean(:,3)));

%% ===== 可选：保存结果 =====
if saveData
    save(savePath, 'model', 'truth', 'meas', ...
         'immXaver', 'simmXaver', 'ospaImmMean', 'ospaSimmMean', ...
         'immNaver', 'simmNaver', 'immMiuAver', 'simmMiuAver');
    fprintf('结果已保存至 %s\n', savePath);
end

fprintf('\n演示完成！\n');
toc;

%% ========================================================================
%                          辅 助 函 数
%  ========================================================================

function cbColors = getCBColors()
% GETCBCOLORS  Return color-blind friendly color palette (Wong's palette).
%   Provides 8 colors that are distinguishable for all types of color blindness.

    cbColors = struct(...
        'black',       [0.00, 0.00, 0.00], ...
        'orange',      [0.90, 0.62, 0.00], ...
        'skyBlue',     [0.34, 0.71, 0.91], ...
        'bluishGreen', [0.00, 0.62, 0.45], ...
        'yellow',      [0.94, 0.89, 0.26], ...
        'blue',        [0.00, 0.45, 0.70], ...
        'vermillion',  [0.84, 0.37, 0.00], ...
        'reddishPurple',[0.80, 0.47, 0.65]);
end

function [xTrack, kBirth, kDeath] = extractTracks(X, trackList, totalTracks)
    K = size(X, 1);
    xDim = 6;
    kk = K;
    while kk > 0
        if ~isempty(X{kk})
            xDim = size(X{kk}, 1);
            break;
        end
        kk = kk - 1;
    end

    xTrack = zeros(xDim, K, totalTracks);
    kBirth = zeros(totalTracks, 1);
    kDeath = zeros(totalTracks, 1);
    maxIdx = 0;

    for k = 1:K
        if ~isempty(X{k})
            xTrack(:, k, trackList{k}) = X{k};
        end
        if ~isempty(trackList{k}) && max(trackList{k}) > maxIdx
            idx = find(trackList{k} > maxIdx);
            kBirth(trackList{k}(idx)) = k;
        end
        if ~isempty(trackList{k})
            maxIdx = max(trackList{k});
            kDeath(trackList{k}) = k;
        end
    end
end

function Xc = getComps(X, c)
    if isempty(X)
        Xc = [];
    else
        Xc = X(c, :);
    end
end

function xAver = computeMcAverageX(mcX, nMC, K)
    xAver = cell(K, 1);
    for k = 1:K
        allCols = [];
        for l = 1:nMC
            allCols = [allCols, cell2mat(mcX(l, k))]; %#ok<AGROW>
        end
        if ~isempty(allCols)
            xAver{k} = sum(allCols, 2) / nMC;
        end
    end
end

function miuAver = computeMcAverageMiu(mcMiu, nMC)
    nModels = size(mcMiu{nMC}, 1);
    K = size(mcMiu{nMC}, 2);
    miuAver = zeros(nModels, K);
    for i = 1:nModels
        allRows = zeros(nMC, K);
        for l = 1:nMC
            mData = mcMiu{l};
            nK = min(K, size(mData, 2));
            allRows(l, 1:nK) = mData(i, 1:nK);
        end
        miuAver(i, :) = mean(allRows, 1);
    end
end

function plotGroundTruth(model, truth, xTrack, kBirth, kDeath)
    cb = getCBColors();
    limit = [model.range_c(1,1), model.range_c(1,2), ...
             model.range_c(2,1), model.range_c(2,2)];
    
    fig = figure('Position', [100, 100, 800, 600]);
    hold on; box on;
    
    for i = 1:truth.total_tracks
        Pt = xTrack(:, kBirth(i):kDeath(i), i);
        Pt = Pt([1, 4], :);
        plot(Pt(1,:), Pt(2,:), '-', 'Color', cb.black, 'LineWidth', 2.0);
    end
    
    for i = 1:truth.total_tracks
        Pt = xTrack(:, kBirth(i):kDeath(i), i);
        Pt = Pt([1, 4], :);
        plot(Pt(1,1), Pt(2,1), 'o', 'Color', cb.bluishGreen, ...
            'MarkerSize', 10, 'MarkerFaceColor', cb.bluishGreen, 'LineWidth', 1.5);
        plot(Pt(1,end), Pt(2,end), '^', 'Color', cb.vermillion, ...
            'MarkerSize', 10, 'MarkerFaceColor', cb.vermillion, 'LineWidth', 1.5);
    end
    
    axis equal; axis(limit);
    title('Ground Truth Trajectories', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('x-coordinate (m)', 'FontSize', 12);
    ylabel('y-coordinate (m)', 'FontSize', 12);
    
    h1 = plot(NaN,NaN,'-', 'Color', cb.black, 'LineWidth', 2.0);
    h2 = plot(NaN,NaN,'o', 'Color', cb.bluishGreen, 'MarkerSize', 10, 'MarkerFaceColor', cb.bluishGreen);
    h3 = plot(NaN,NaN,'^', 'Color', cb.vermillion, 'MarkerSize', 10, 'MarkerFaceColor', cb.vermillion);
    legend([h1, h2, h3], {'Trajectory', 'Start Point', 'End Point'}, 'Location', 'best', 'FontSize', 11);
    grid on;
    set(gca, 'FontSize', 11, 'LineWidth', 1.0);
    set(fig, 'Color', 'w');
end

function plotTracksAndEstimates(model, truth, meas, xTrack, kBirth, kDeath, immXaver, simmXaver)
    cb = getCBColors();
    fig = figure('Position', [100, 100, 1000, 700]);
    
    for sp = 1:2
        subplot(2,1,sp); box on; hold on;
        
        for k = 1:meas.K
            if ~isempty(meas.Z{k})
                zData = meas.Z{k}(sp,:);
                plot(k*ones(size(zData)), zData, 'x', 'Color', [0.7, 0.7, 0.7], ...
                    'MarkerSize', 4, 'LineWidth', 0.5);
            end
        end
        
        for i = 1:truth.total_tracks
            P = xTrack(:, kBirth(i):kDeath(i), i);
            P = P([1,4], :);
            plot(kBirth(i):kDeath(i), P(sp,:), '-', 'Color', cb.black, 'LineWidth', 2.0);
        end
        
        for k = 1:meas.K
            if ~isempty(immXaver{k})
                P = immXaver{k}([1,4], :);
                plot(k*ones(size(P,2)), P(sp,:), 'o', 'Color', cb.blue, ...
                    'MarkerSize', 6, 'MarkerFaceColor', cb.blue, 'LineWidth', 1.0);
            end
            if ~isempty(simmXaver{k})
                P = simmXaver{k}([1,4], :);
                plot(k*ones(size(P,2)), P(sp,:), 's', 'Color', cb.vermillion, ...
                    'MarkerSize', 5, 'MarkerFaceColor', 'none', 'LineWidth', 1.2);
            end
        end
        
        xlabel('Time Step', 'FontSize', 12);
        if sp == 1
            ylabel('x-coordinate (m)', 'FontSize', 12);
            set(gca, 'YLim', model.range_c(1,:));
            title('Position Estimates (x-coordinate)', 'FontSize', 14, 'FontWeight', 'bold');
        else
            ylabel('y-coordinate (m)', 'FontSize', 12);
            set(gca, 'YLim', model.range_c(2,:));
            title('Position Estimates (y-coordinate)', 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        set(gca, 'XLim', [1, truth.K], 'FontSize', 11, 'LineWidth', 1.0);
        grid on;
        
        if sp == 1
            h1 = plot(NaN,NaN,'-', 'Color', cb.black, 'LineWidth', 2.0);
            h2 = plot(NaN,NaN,'o', 'Color', cb.blue, 'MarkerSize', 6, 'MarkerFaceColor', cb.blue);
            h3 = plot(NaN,NaN,'s', 'Color', cb.vermillion, 'MarkerSize', 5, 'MarkerFaceColor', 'none', 'LineWidth', 1.2);
            h4 = plot(NaN,NaN,'x', 'Color', [0.7, 0.7, 0.7], 'MarkerSize', 4);
            legend([h3, h2, h1, h4], {'SIMM-PHD', 'IMM-PHD', 'True Track', 'Measurements'}, ...
                'Location', 'best', 'FontSize', 10);
        end
    end
    
    set(fig, 'Color', 'w');
end

function plotOspa(K, ospaImm, ospaSimm, ospaC)
    cb = getCBColors();
    labels = {'OSPA Distance', 'Localization Error', 'Cardinality Error'};
    
    fig = figure('Position', [100, 100, 900, 800]);
    t = 1:K;
    
    for s = 1:3
        subplot(3,1,s); box on; hold on;
        
        plot(t, ospaImm(:,s), '-', 'Color', cb.blue, 'LineWidth', 2.0, ...
            'Marker', 'o', 'MarkerSize', 4, 'MarkerFaceColor', cb.blue);
        plot(t, ospaSimm(:,s), '--', 'Color', cb.vermillion, 'LineWidth', 2.0, ...
            'Marker', 's', 'MarkerSize', 4, 'MarkerFaceColor', 'none');
        
        grid on;
        set(gca, 'XLim', [1, K], 'YLim', [0, ospaC], 'FontSize', 11, 'LineWidth', 1.0);
        
        h1 = plot(NaN,NaN,'-', 'Color', cb.blue, 'LineWidth', 2.0, 'Marker', 'o', 'MarkerSize', 4);
        h2 = plot(NaN,NaN,'--', 'Color', cb.vermillion, 'LineWidth', 2.0, 'Marker', 's', 'MarkerSize', 4);
        legend([h1, h2], {'IMM-PHD', 'SIMM-PHD'}, 'Location', 'best', 'FontSize', 10);
        
        xlabel('Time Step', 'FontSize', 12);
        ylabel(labels{s}, 'FontSize', 12);
        
        if s == 1
            title('OSPA Performance Metrics Comparison', 'FontSize', 14, 'FontWeight', 'bold');
        end
    end
    
    set(fig, 'Color', 'w');
end

function plotCardinality(K, trueN, immN, simmN)
    cb = getCBColors();
    fig = figure('Position', [100, 100, 900, 700]);
    t = 1:K;
    
    subplot(2,1,1); box on; hold on;
    stairs(t, trueN, '-', 'Color', cb.black, 'LineWidth', 2.5);
    plot(t, immN, 'o', 'Color', cb.blue, 'MarkerSize', 5, 'MarkerFaceColor', cb.blue, 'LineWidth', 1.0);
    grid on;
    legend({'True Cardinality', 'IMM-PHD Estimate'}, 'Location', 'best', 'FontSize', 11);
    set(gca, 'XLim', [1, K], 'YLim', [0, max(trueN)+2], 'FontSize', 11, 'LineWidth', 1.0);
    xlabel('Time Step', 'FontSize', 12);
    ylabel('Number of Targets', 'FontSize', 12);
    title('Target Cardinality Estimation (IMM-PHD)', 'FontSize', 14, 'FontWeight', 'bold');
    
    subplot(2,1,2); box on; hold on;
    stairs(t, trueN, '-', 'Color', cb.black, 'LineWidth', 2.5);
    plot(t, simmN, 's', 'Color', cb.vermillion, 'MarkerSize', 5, 'MarkerFaceColor', cb.vermillion, 'LineWidth', 1.0);
    grid on;
    legend({'True Cardinality', 'SIMM-PHD Estimate'}, 'Location', 'best', 'FontSize', 11);
    set(gca, 'XLim', [1, K], 'YLim', [0, max(trueN)+2], 'FontSize', 11, 'LineWidth', 1.0);
    xlabel('Time Step', 'FontSize', 12);
    ylabel('Number of Targets', 'FontSize', 12);
    title('Target Cardinality Estimation (SIMM-PHD)', 'FontSize', 14, 'FontWeight', 'bold');
    
    set(fig, 'Color', 'w');
end

function plotModelProbability(K, miuAver, titlePrefix)
    cb = getCBColors();
    fig = figure('Position', [100, 100, 900, 500]);
    t = 1:K;
    
    hold on; box on;
    plot(t, miuAver(1,:), '-', 'Color', cb.vermillion, 'LineWidth', 2.5, ...
        'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', cb.vermillion);
    plot(t, miuAver(2,:), '--', 'Color', cb.blue, 'LineWidth', 2.5, ...
        'Marker', 's', 'MarkerSize', 3, 'MarkerFaceColor', cb.blue);
    plot(t, miuAver(3,:), ':', 'Color', cb.bluishGreen, 'LineWidth', 2.5, ...
        'Marker', 'd', 'MarkerSize', 3, 'MarkerFaceColor', cb.bluishGreen);
    
    title(sprintf('%s: Model Probabilities over Time', titlePrefix), ...
        'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Time Step', 'FontSize', 12);
    ylabel('Model Probability', 'FontSize', 12);
    grid on;
    ylim([0, 1.05]);
    
    h1 = plot(NaN,NaN,'-', 'Color', cb.vermillion, 'LineWidth', 2.5);
    h2 = plot(NaN,NaN,'--', 'Color', cb.blue, 'LineWidth', 2.5);
    h3 = plot(NaN,NaN,':', 'Color', cb.bluishGreen, 'LineWidth', 2.5);
    legend([h1, h2, h3], {'CV Model', 'CT Model', 'CA Model'}, ...
        'Location', 'best', 'FontSize', 11);
    
    set(gca, 'FontSize', 11, 'LineWidth', 1.0);
    set(fig, 'Color', 'w');
end
