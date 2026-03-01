classdef Visualizer
% VIZ.VISUALIZER  目标跟踪结果增强可视化类。
%   提供色盲友好的可视化功能，使用蓝橙色调色板，
%   支持多种图表类型包括线图、柱状图和热力图。
%
%   使用方法：
%       viz = viz.Visualizer();
%       viz.Visualizer.plotDbtTrajectory(truthX, meas, estimates);
%       viz.Visualizer.plotRmseComparison(rmseData);
%       viz.Visualizer.plotTbdResults(trueState, dpTrack, pfState, measData);
%
%   色盲友好调色板 (CUD配色)：
%       blue    - #0077BB (主色调，用于主要数据)
%       orange  - #EE7733 (次色调，用于对比数据)
%       cyan    - #33BBEE (辅助色)
%       gray    - #BBBBBB (背景/次要元素)
%       black   - #000000 (参考线/真实值)
%       green   - #228833 (正向指示)
%       purple  - #AA3377 (附加色)
%
%   方法分类：
%       通用方法：
%           createFigure       - 创建白色背景图形
%           plotHeatmap        - 绘制热力图
%           plotTimingComparison - 绘制执行时间对比
%       DBT方法：
%           plotDbtTrajectory  - 绘制DBT轨迹
%           plotRmseComparison - 绘制RMSE对比线图
%           plotRmseBarChart   - 绘制RMSE柱状图
%           plotComprehensiveDbt - 综合DBT可视化
%           plotManeuverComparison - 机动目标对比可视化
%       TBD方法：
%           plotTbdTrajectory  - 绘制TBD轨迹
%           plotTbdRmse        - 绘制TBD RMSE
%           plotPerAxisError   - 绘制分轴误差
%           plotMeasurementFrames - 绘制测量帧
%           plotComprehensiveTbd - 综合TBD可视化
%
%   See also: dbt.Config, tbd.Config

    properties (Constant)
        COLORS = struct(...
            'blue',   [0, 0.467, 0.733], ...
            'orange', [0.933, 0.467, 0.200], ...
            'cyan',   [0.200, 0.733, 0.933], ...
            'gray',   [0.733, 0.733, 0.733], ...
            'black',  [0, 0, 0], ...
            'green',  [0.133, 0.533, 0.200], ...
            'purple', [0.667, 0.333, 0.600])
        DEFAULT_FIG_WIDTH = 900
        DEFAULT_FIG_HEIGHT = 700
        DEFAULT_FONT_SIZE = 10
        DEFAULT_LINE_WIDTH = 1.5
    end

    methods (Static)

        function fig = createFigure(name, width, height)
        % CREATEFIGURE  创建白色背景的新图形窗口。
        %
        %   输入参数：
        %       name   - 图形窗口名称
        %       width  - (可选) 宽度 [像素]，默认900
        %       height - (可选) 高度 [像素]，默认700
        %
        %   输出参数：
        %       fig - 图形句柄
            if nargin < 2
                width = viz.Visualizer.DEFAULT_FIG_WIDTH;
                height = viz.Visualizer.DEFAULT_FIG_HEIGHT;
            end
            fig = figure('Name', name, 'Color', 'w', ...
                         'Position', [100, 100, width, height]);
        end

        function plotDbtTrajectory(truthX, meas, estimates, labels)
        % PLOTDBTTRAJECTORY  绘制2D轨迹与估计结果。
        %
        %   输入参数：
        %       truthX   - 真实轨迹 [stateDim x nSteps]
        %       meas     - 测量数据 [measDim x nSteps]（极坐标：[方位角; 距离]）
        %       estimates - 估计结果单元数组 {ekf, ukf, ckf, pf}
        %       labels   - 标签单元数组 {'EKF', 'UKF', 'CKF', 'PF'}
        %
        %   说明：
        %       将极坐标测量转换为笛卡尔坐标进行显示，
        %       同时显示起点和终点标记。
            viz.Visualizer.createFigure('DBT轨迹', 700, 600);

            xMeas = meas(2, :) .* cos(meas(1, :));
            yMeas = meas(2, :) .* sin(meas(1, :));

            colors = {viz.Visualizer.COLORS.blue, viz.Visualizer.COLORS.orange, ...
                      viz.Visualizer.COLORS.cyan, viz.Visualizer.COLORS.green};
            lineStyles = {'-', '--', '-.', ':'};

            hold on;
            plot(truthX(1, :), truthX(3, :), 'k-.', 'LineWidth', 1.5, 'DisplayName', '真实值');
            plot(xMeas, yMeas, 'o', 'Color', viz.Visualizer.COLORS.gray, ...
                 'MarkerSize', 4, 'DisplayName', '测量值');

            for i = 1:length(estimates)
                plot(estimates{i}(1, :), estimates{i}(3, :), ...
                     'LineStyle', lineStyles{i}, 'Color', colors{i}, ...
                     'LineWidth', 1.2, 'DisplayName', labels{i});
            end

            plot(truthX(1, 1), truthX(3, 1), '^', 'Color', viz.Visualizer.COLORS.green, ...
                 'MarkerSize', 10, 'MarkerFaceColor', viz.Visualizer.COLORS.green, ...
                 'DisplayName', '起点');
            plot(truthX(1, end), truthX(3, end), 'v', 'Color', viz.Visualizer.COLORS.orange, ...
                 'MarkerSize', 10, 'MarkerFaceColor', viz.Visualizer.COLORS.orange, ...
                 'DisplayName', '终点');

            xlabel('x [m]'); ylabel('y [m]');
            legend('Location', 'best');
            grid on; axis equal; hold off;
        end

        function plotRmseComparison(rmseData, labels, titleStr)
        % PLOTRMSECOMPARISON  绘制RMSE对比线图。
        %
        %   输入参数：
        %       rmseData - RMSE向量单元数组 {ekf, ukf, ckf, pf}
        %       labels   - 标签单元数组 {'EKF', 'UKF', 'CKF', 'PF'}
        %       titleStr - (可选) 图表标题，默认为'RMSE Comparison'
            if nargin < 3
                titleStr = 'RMSE对比';
            end

            if ~iscell(rmseData)
                rmseData = {rmseData};
            end

            viz.Visualizer.createFigure(titleStr, 800, 500);
            colors = {viz.Visualizer.COLORS.blue, viz.Visualizer.COLORS.orange, ...
                      viz.Visualizer.COLORS.cyan, viz.Visualizer.COLORS.green};
            lineStyles = {'-', '--', '-.', ':'};

            nSteps = length(rmseData{1});
            hold on;
            for i = 1:length(rmseData)
                plot(1:nSteps, rmseData{i}, 'LineStyle', lineStyles{i}, ...
                     'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', labels{i});
            end
            xlabel('时间步'); ylabel('RMSE');
            title(titleStr);
            legend('Location', 'best');
            grid on; hold off;
        end

        function plotRmseBarChart(meanRmse, labels, groups, titleStr)
        % PLOTRMSEBARCHART  绘制RMSE分组柱状图。
        %
        %   输入参数：
        %       meanRmse - 均值RMSE矩阵 [nAlgorithms x nGroups]
        %       labels   - 算法标签单元数组
        %       groups   - 分组标签单元数组（如 {'位置', '速度'}）
        %       titleStr - (可选) 图表标题
            if nargin < 4
                titleStr = '均值RMSE对比';
            end

            viz.Visualizer.createFigure(titleStr, 600, 500);
            b = bar(meanRmse, 'grouped');

            colors = [viz.Visualizer.COLORS.blue; viz.Visualizer.COLORS.orange; ...
                      viz.Visualizer.COLORS.cyan; viz.Visualizer.COLORS.green];
            for i = 1:min(length(b), size(colors, 1))
                b(i).FaceColor = 'flat';
                b(i).CData = repmat(colors(i,:), length(groups), 1);
            end

            set(gca, 'XTickLabel', groups);
            legend(labels, 'Location', 'best');
            ylabel('均值RMSE');
            title(titleStr);
            grid on;
        end

        function plotHeatmap(data, xLabel, yLabel, titleStr, cmap)
        % PLOTHEATMAP  绘制热力图。
        %
        %   输入参数：
        %       data     - 2D数据矩阵
        %       xLabel   - X轴标签
        %       yLabel   - Y轴标签
        %       titleStr - 图表标题
        %       cmap     - (可选) 颜色映射名称，默认'parula'
            if nargin < 5
                cmap = 'parula';
            end

            viz.Visualizer.createFigure(titleStr, 600, 500);
            imagesc(data); colormap(cmap); colorbar;
            xlabel(xLabel); ylabel(yLabel);
            title(titleStr);
            axis equal tight;
        end

        function plotTbdTrajectory(trueState, estimates, labels, measData)
        % PLOTTBDTRAJECTORY  在测量背景上绘制TBD轨迹。
        %
        %   输入参数：
        %       trueState - 真实轨迹 [nFrames x 5]
        %       estimates - 估计结果单元数组 {dpTrack, pfState}
        %       labels    - 标签单元数组 {'DP-TBD', 'PF-TBD'}
        %       measData  - 测量数据立方体 [rows x cols x nFrames]

            colors = {viz.Visualizer.COLORS.orange, viz.Visualizer.COLORS.blue};

            imagesc(mean(measData, 3)); colormap(gca, 'parula'); hold on;

            plot(trueState(:, 2), trueState(:, 1), '-o', ...
                 'Color', viz.Visualizer.COLORS.green, 'LineWidth', 2, ...
                 'MarkerSize', 3, 'DisplayName', '真实值');

            for i = 1:length(estimates)
                if ~isempty(estimates{i})
                    plot(estimates{i}(:, 2), estimates{i}(:, 1), ...
                         '--s', 'Color', colors{i}, 'LineWidth', 1.5, ...
                         'MarkerSize', 4, 'DisplayName', labels{i});
                end
            end

            plot(trueState(1, 2), trueState(1, 1), 'p', ...
                 'Color', viz.Visualizer.COLORS.green, 'MarkerSize', 12, ...
                 'MarkerFaceColor', viz.Visualizer.COLORS.green, 'DisplayName', '起点');
            plot(trueState(end, 2), trueState(end, 1), 'h', ...
                 'Color', viz.Visualizer.COLORS.green, 'MarkerSize', 12, ...
                 'MarkerFaceColor', viz.Visualizer.COLORS.green, 'DisplayName', '终点');

            legend('Location', 'best');
            axis equal tight; hold off;
        end

        function plotTbdRmse(dpPosRmse, pfPosRmse, dpVelRmse, pfVelRmse, nFrames)
        % PLOTTBDRMSE  绘制TBD位置和速度RMSE。
        %
        %   输入参数：
        %       dpPosRmse - DP-TBD位置RMSE向量
        %       pfPosRmse - PF-TBD位置RMSE向量
        %       dpVelRmse - DP-TBD速度RMSE向量
        %       pfVelRmse - PF-TBD速度RMSE向量
        %       nFrames   - 帧数

            subplot(1, 2, 1);
            hold on;
            plot(1:nFrames, dpPosRmse, '-', 'Color', viz.Visualizer.COLORS.orange, ...
                 'LineWidth', 1.5, 'DisplayName', 'DP-TBD');
            plot(1:nFrames, pfPosRmse, '-', 'Color', viz.Visualizer.COLORS.blue, ...
                 'LineWidth', 1.5, 'DisplayName', 'PF-TBD');
            xlabel('帧'); ylabel('位置RMSE [像素]');
            legend('Location', 'best'); grid on; hold off;

            subplot(1, 2, 2);
            hold on;
            plot(2:nFrames, dpVelRmse, '-', 'Color', viz.Visualizer.COLORS.orange, ...
                 'LineWidth', 1.5, 'DisplayName', 'DP-TBD');
            plot(1:nFrames, pfVelRmse, '-', 'Color', viz.Visualizer.COLORS.blue, ...
                 'LineWidth', 1.5, 'DisplayName', 'PF-TBD');
            xlabel('帧'); ylabel('速度RMSE [像素/帧]');
            legend('Location', 'best'); grid on; hold off;
        end

        function plotPerAxisError(dpTrack, pfState, trueState, nFrames)
        % PLOTPERAXISERROR  绘制分轴跟踪误差。
        %
        %   输入参数：
        %       dpTrack   - DP-TBD估计轨迹 [nFrames x 2]
        %       pfState   - PF-TBD估计状态 [nFrames x 5]
        %       trueState - 真实状态 [nFrames x 5]
        %       nFrames   - 帧数

            subplot(1, 2, 1);
            hold on;
            plot(1:nFrames, dpTrack(:, 1) - trueState(:, 1), '-', ...
                 'Color', viz.Visualizer.COLORS.orange, 'DisplayName', 'DP行');
            plot(1:nFrames, dpTrack(:, 2) - trueState(:, 2), '--', ...
                 'Color', viz.Visualizer.COLORS.orange, 'DisplayName', 'DP列');
            plot(1:nFrames, pfState(:, 1) - trueState(:, 1), '-', ...
                 'Color', viz.Visualizer.COLORS.blue, 'DisplayName', 'PF行');
            plot(1:nFrames, pfState(:, 2) - trueState(:, 2), '--', ...
                 'Color', viz.Visualizer.COLORS.blue, 'DisplayName', 'PF列');
            yline(0, 'k:');
            xlabel('帧'); ylabel('误差 [像素]');
            title('位置误差'); legend('Location', 'best'); grid on; hold off;

            subplot(1, 2, 2);
            hold on;
            plot(1:nFrames, pfState(:, 5), '-', 'Color', viz.Visualizer.COLORS.blue, ...
                 'LineWidth', 1.5, 'DisplayName', 'PF估计');
            plot(1:nFrames, trueState(:, 5), '--', 'Color', viz.Visualizer.COLORS.green, ...
                 'LineWidth', 1.5, 'DisplayName', '真实值');
            xlabel('帧'); ylabel('幅度');
            title('幅度估计'); legend('Location', 'best'); grid on; hold off;
        end

        function plotTimingComparison(timingData, labels, titleStr)
        % PLOTTIMINGCOMPARISON  绘制执行时间柱状图。
        %
        %   输入参数：
        %       timingData - 执行时间向量
        %       labels     - 标签单元数组
        %       titleStr   - (可选) 图表标题
            if nargin < 3
                titleStr = '执行时间对比';
            end

            viz.Visualizer.createFigure(titleStr, 600, 400);
            colors = [viz.Visualizer.COLORS.blue; viz.Visualizer.COLORS.orange; ...
                      viz.Visualizer.COLORS.cyan; viz.Visualizer.COLORS.green];

            b = bar(timingData);
            b.FaceColor = 'flat';
            for i = 1:length(timingData)
                b.CData(i,:) = colors(mod(i-1, size(colors,1))+1, :);
            end

            set(gca, 'XTickLabel', labels);
            ylabel('时间 [s]');
            title(titleStr);
            grid on;

            for i = 1:length(timingData)
                text(i, timingData(i), sprintf('%.3f s', timingData(i)), ...
                     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
            end
        end

        function plotMeasurementFrames(measData, trueState, frameIndices)
        % PLOTMEASUREMENTFRAMES  绘制多帧测量数据。
        %
        %   输入参数：
        %       measData     - 测量数据立方体 [rows x cols x nFrames]
        %       trueState    - 真实状态 [nFrames x 5]
        %       frameIndices - 要绘制的帧索引数组
            nFrames = length(frameIndices);

            for i = 1:nFrames
                tf = frameIndices(i);
                subplot(1, nFrames, i);
                imagesc(measData(:, :, tf)); colormap(gca, 'parula'); hold on;
                plot(trueState(tf, 2), trueState(tf, 1), 'w+', ...
                     'MarkerSize', 14, 'LineWidth', 2);
                title(sprintf('帧 %d', tf));
                axis equal tight; hold off;
            end
        end

        function fig = plotComprehensiveDbt(truthX, meas, estimates, rmse, labels, nSteps)
        % PLOTCOMPREHENSIVEDBT  创建综合DBT可视化。
        %
        %   输入参数：
        %       truthX    - 真实轨迹 [stateDim x nSteps]
        %       meas      - 测量数据 [measDim x nSteps]
        %       estimates - 估计结果单元数组
        %       rmse      - RMSE结构体 (.posRmse, .velRmse)
        %       labels    - 标签单元数组
        %       nSteps    - 时间步数
        %
        %   输出参数：
        %       fig - 图形句柄
            fig = viz.Visualizer.createFigure('DBT综合结果', 1200, 800);

            xMeas = meas(2, :) .* cos(meas(1, :));
            yMeas = meas(2, :) .* sin(meas(1, :));

            colors = {viz.Visualizer.COLORS.blue, viz.Visualizer.COLORS.orange, ...
                      viz.Visualizer.COLORS.cyan, viz.Visualizer.COLORS.green};
            lineStyles = {'-', '--', '-.', ':'};

            posRmse = rmse.posRmse;
            velRmse = rmse.velRmse;
            if ~iscell(posRmse), posRmse = {posRmse}; end
            if ~iscell(velRmse), velRmse = {velRmse}; end

            subplot(2, 2, 1);
            hold on;
            plot(truthX(1, :), truthX(3, :), 'k-.', 'LineWidth', 1.5, 'DisplayName', '真实值');
            plot(xMeas, yMeas, 'o', 'Color', viz.Visualizer.COLORS.gray, ...
                 'MarkerSize', 4, 'DisplayName', '测量值');
            for i = 1:length(estimates)
                plot(estimates{i}(1, :), estimates{i}(3, :), ...
                     'LineStyle', lineStyles{i}, 'Color', colors{i}, ...
                     'LineWidth', 1.2, 'DisplayName', labels{i});
            end
            plot(truthX(1, 1), truthX(3, 1), '^', 'Color', viz.Visualizer.COLORS.green, ...
                 'MarkerSize', 10, 'MarkerFaceColor', viz.Visualizer.COLORS.green, ...
                 'DisplayName', '起点');
            plot(truthX(1, end), truthX(3, end), 'v', 'Color', viz.Visualizer.COLORS.orange, ...
                 'MarkerSize', 10, 'MarkerFaceColor', viz.Visualizer.COLORS.orange, ...
                 'DisplayName', '终点');
            xlabel('x [m]'); ylabel('y [m]');
            legend('Location', 'best');
            grid on; axis equal; hold off;

            subplot(2, 2, 2);
            hold on;
            for i = 1:length(posRmse)
                plot(1:nSteps, posRmse{i}, 'LineStyle', lineStyles{i}, ...
                     'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', labels{i});
            end
            xlabel('时间步'); ylabel('位置RMSE [m]');
            title('位置RMSE');
            legend('Location', 'best'); grid on; hold off;

            subplot(2, 2, 3);
            hold on;
            for i = 1:length(velRmse)
                plot(1:nSteps, velRmse{i}, 'LineStyle', lineStyles{i}, ...
                     'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', labels{i});
            end
            xlabel('时间步'); ylabel('速度RMSE [m/s]');
            title('速度RMSE');
            legend('Location', 'best'); grid on; hold off;

            subplot(2, 2, 4);
            meanPos = cellfun(@mean, posRmse);
            meanVel = cellfun(@mean, velRmse);
            
            nAlgo = min(length(labels), length(meanPos));
            
            categories = {'位置RMSE', '速度RMSE'};
            algoLabels = labels(1:nAlgo);
            
            hold on;
            barX = 1:nAlgo;
            width = 0.35;
            
            bar(barX - width/2, meanPos(1:nAlgo), width, ...
                     'FaceColor', viz.Visualizer.COLORS.blue, 'DisplayName', categories{1});
            bar(barX + width/2, meanVel(1:nAlgo), width, ...
                     'FaceColor', viz.Visualizer.COLORS.orange, 'DisplayName', categories{2});
            
            set(gca, 'XTick', barX, 'XTickLabel', algoLabels);
            legend('Location', 'best');
            ylabel('均值RMSE');
            title('均值RMSE汇总');
            grid on; hold off;
        end

        function fig = plotComprehensiveTbd(trueState, dpTrack, pfState, measData, ...
                                            dpPosRmse, pfPosRmse, dpVelRmse, pfVelRmse, cfg)
        % PLOTCOMPREHENSIVETBD  创建综合TBD可视化。
        %
        %   输入参数：
        %       trueState - 真实状态 [nFrames x 5]
        %       dpTrack   - DP-TBD估计轨迹 [nFrames x 2]
        %       pfState   - PF-TBD估计状态 [nFrames x 5]
        %       measData  - 测量数据立方体
        %       dpPosRmse - DP-TBD位置RMSE
        %       pfPosRmse - PF-TBD位置RMSE
        %       dpVelRmse - DP-TBD速度RMSE
        %       pfVelRmse - PF-TBD速度RMSE
        %       cfg       - 配置对象
        %
        %   输出参数：
        %       fig - 图形句柄
            fig = viz.Visualizer.createFigure('TBD综合结果', 1400, 900);

            nF = cfg.numFrames;
            snapIdx = unique(round(linspace(1, nF, 4)));

            subplot(2, 3, 1);
            imagesc(mean(measData, 3)); colormap(gca, 'parula'); hold on;
            plot(trueState(:, 2), trueState(:, 1), '-o', ...
                 'Color', viz.Visualizer.COLORS.green, 'LineWidth', 2, ...
                 'MarkerSize', 3, 'DisplayName', '真实值');
            if ~isempty(dpTrack)
                plot(dpTrack(:, 2), dpTrack(:, 1), '--s', 'Color', viz.Visualizer.COLORS.orange, ...
                     'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'DP-TBD');
            end
            if ~isempty(pfState)
                plot(pfState(:, 2), pfState(:, 1), '--s', 'Color', viz.Visualizer.COLORS.blue, ...
                     'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'PF-TBD');
            end
            plot(trueState(1, 2), trueState(1, 1), 'p', ...
                 'Color', viz.Visualizer.COLORS.green, 'MarkerSize', 12, ...
                 'MarkerFaceColor', viz.Visualizer.COLORS.green, 'DisplayName', '起点');
            plot(trueState(end, 2), trueState(end, 1), 'h', ...
                 'Color', viz.Visualizer.COLORS.green, 'MarkerSize', 12, ...
                 'MarkerFaceColor', viz.Visualizer.COLORS.green, 'DisplayName', '终点');
            legend('Location', 'best');
            xlabel('列'); ylabel('行');
            title('轨迹对比');
            axis equal tight; hold off;

            subplot(2, 3, 2);
            hold on;
            plot(1:nF, dpPosRmse, '-', 'Color', viz.Visualizer.COLORS.orange, ...
                 'LineWidth', 1.5, 'DisplayName', 'DP-TBD');
            plot(1:nF, pfPosRmse, '-', 'Color', viz.Visualizer.COLORS.blue, ...
                 'LineWidth', 1.5, 'DisplayName', 'PF-TBD');
            xlabel('帧'); ylabel('位置RMSE [像素]');
            title('位置RMSE');
            legend('Location', 'best'); grid on; hold off;

            subplot(2, 3, 3);
            hold on;
            plot(1:nF, dpTrack(:, 1) - trueState(:, 1), '-', ...
                 'Color', viz.Visualizer.COLORS.orange, 'DisplayName', 'DP行');
            plot(1:nF, dpTrack(:, 2) - trueState(:, 2), '--', ...
                 'Color', viz.Visualizer.COLORS.orange, 'DisplayName', 'DP列');
            plot(1:nF, pfState(:, 1) - trueState(:, 1), '-', ...
                 'Color', viz.Visualizer.COLORS.blue, 'DisplayName', 'PF行');
            plot(1:nF, pfState(:, 2) - trueState(:, 2), '--', ...
                 'Color', viz.Visualizer.COLORS.blue, 'DisplayName', 'PF列');
            yline(0, 'k:');
            xlabel('帧'); ylabel('误差 [像素]');
            title('位置误差'); legend('Location', 'best'); grid on; hold off;

            subplot(2, 3, 4);
            hold on;
            plot(1:nF, dpVelRmse, '-', 'Color', viz.Visualizer.COLORS.orange, ...
                 'LineWidth', 1.5, 'DisplayName', 'DP-TBD');
            plot(1:nF, pfVelRmse, '-', 'Color', viz.Visualizer.COLORS.blue, ...
                 'LineWidth', 1.5, 'DisplayName', 'PF-TBD');
            xlabel('帧'); ylabel('速度RMSE [像素/帧]');
            title('速度RMSE');
            legend('Location', 'best'); grid on; hold off;

            subplot(2, 3, 5);
            meanRmse = [mean(dpPosRmse), mean(pfPosRmse); mean(dpVelRmse), mean(pfVelRmse)];
            colors = [viz.Visualizer.COLORS.orange, viz.Visualizer.COLORS.blue];
            b = bar(meanRmse, 'grouped');
            for i = 1:min(length(b), size(colors, 1))
                b(i).FaceColor = 'flat';
                b(i).CData = repmat(colors(i,:), 2, 1);
            end
            set(gca, 'XTickLabel', {'位置', '速度'});
            legend({'DP-TBD', 'PF-TBD'}, 'Location', 'best');
            ylabel('均值RMSE');
            title('均值RMSE汇总');
            grid on;

            subplot(2, 3, 6);
            axis off;
            txt = {'=== 算法汇总 ===', '', ...
                   sprintf('DP-TBD: Pos=%.2f像素, Vel=%.2f像素/帧', mean(dpPosRmse), mean(dpVelRmse)), ...
                   sprintf('PF-TBD: Pos=%.2f像素, Vel=%.2f像素/帧', mean(pfPosRmse), mean(pfVelRmse))};
            text(0.05, 0.95, txt, 'VerticalAlignment', 'top', 'FontSize', 10);
        end

        function fig = plotManeuverComparison(truthX, meas, estStates, posRmse, velRmse, labels, maneuverLabels)
        % PLOTMANEUVERCOMPARISON  创建机动目标综合对比可视化。
        %
        %   输入参数：
        %       truthX        - 真实轨迹 [stateDim x nSteps]
        %       meas          - 测量数据 [measDim x nSteps]
        %       estStates     - 估计状态单元数组 {nAlgorithms}
        %       posRmse       - 位置RMSE单元数组 {nAlgorithms}
        %       velRmse       - 速度RMSE单元数组 {nAlgorithms}
        %       labels        - 算法标签单元数组
        %       maneuverLabels - 机动类型标签单元数组 [1 x nSteps]
        %
        %   输出参数：
        %       fig - 图形句柄
            fig = viz.Visualizer.createFigure('机动目标跟踪对比', 1400, 900);
            nSteps = size(truthX, 2);
            nAlgo = length(labels);

            cudColors = containers.Map(...
                {'CV', 'CA', 'CT', 'Singer', 'CS', 'IMM', 'default'}, ...
                {[0.000, 0.447, 0.698], [0.902, 0.624, 0.000], [0.000, 0.620, 0.451], ...
                 [0.800, 0.475, 0.655], [0.968, 0.506, 0.749], [0.300, 0.300, 0.300], ...
                 [0.5, 0.5, 0.5]});

            lineStyles = containers.Map(...
                {'CV', 'CA', 'CT', 'Singer', 'CS', 'IMM', 'default'}, ...
                {'-', '--', '-.', ':', '--', '-', '-'});

            markers = containers.Map(...
                {'CV', 'CA', 'CT', 'Singer', 'CS', 'IMM', 'default'}, ...
                {'o', 's', 'd', '^', 'v', 'none', 'none'});

            colors = zeros(nAlgo, 3);
            lStyles = cell(nAlgo, 1);
            mkrs = cell(nAlgo, 1);
            for a = 1:nAlgo
                if isKey(cudColors, labels{a})
                    colors(a,:) = cudColors(labels{a});
                    lStyles{a} = lineStyles(labels{a});
                    mkrs{a} = markers(labels{a});
                else
                    colors(a,:) = cudColors('default');
                    lStyles{a} = lineStyles('default');
                    mkrs{a} = markers('default');
                end
            end
            
            fontSize = viz.Visualizer.DEFAULT_FONT_SIZE;
            trueLineWidth = 2.5;
            estLineWidth = 1.8;
            markerSize = 6;
            markerInterval = max(1, floor(nSteps / 20));
            
            subplot(2, 3, 1);
            trueX = truthX(1, :);
            trueY = truthX(4, :);
            measX = meas(2, :) .* cos(meas(1, :));
            measY = meas(2, :) .* sin(meas(1, :));
            
            plot(trueX, trueY, 'k-', 'LineWidth', trueLineWidth, 'DisplayName', '真实值'); hold on;
            plot(measX, measY, 'k.', 'MarkerSize', 2, 'Color', [0.6 0.6 0.6], ...
                 'HandleVisibility', 'off');
            
            for a = 1:nAlgo
                est = estStates{a};
                if size(est, 1) == 4
                    estX = est(1, :); estY = est(3, :);
                elseif size(est, 1) == 5
                    estX = est(1, :); estY = est(3, :);
                else
                    estX = est(1, :); estY = est(4, :);
                end
                if strcmp(mkrs{a}, 'none')
                    plot(estX, estY, lStyles{a}, 'Color', colors(a,:), 'LineWidth', estLineWidth, 'DisplayName', labels{a});
                else
                    plot(estX, estY, [lStyles{a} mkrs{a}], 'Color', colors(a,:), 'LineWidth', estLineWidth, ...
                         'MarkerSize', markerSize, 'MarkerIndices', 1:markerInterval:nSteps, 'DisplayName', labels{a});
                end
            end
            plot(trueX(1), trueY(1), 'g^', 'MarkerSize', 12, 'MarkerFaceColor', [0.2 0.8 0.2], 'DisplayName', '起点');
            xlabel('x [m]', 'FontSize', fontSize); ylabel('y [m]', 'FontSize', fontSize);
            title('轨迹对比', 'FontSize', fontSize + 2, 'FontWeight', 'bold');
            legend('Location', 'best', 'NumColumns', 2, 'FontSize', fontSize - 1);
            grid on; axis equal; hold off;
            set(gca, 'FontSize', fontSize, 'Box', 'on', 'LineWidth', 0.8);
            
            subplot(2, 3, 2);
            hold on;
            for a = 1:nAlgo
                if strcmp(mkrs{a}, 'none')
                    plot(1:nSteps, posRmse{a}, lStyles{a}, 'Color', colors(a,:), 'LineWidth', estLineWidth, 'DisplayName', labels{a});
                else
                    plot(1:nSteps, posRmse{a}, [lStyles{a} mkrs{a}], 'Color', colors(a,:), 'LineWidth', estLineWidth, ...
                         'MarkerSize', markerSize, 'MarkerIndices', 1:markerInterval:nSteps, 'DisplayName', labels{a});
                end
            end
            xlabel('时间步', 'FontSize', fontSize); ylabel('位置RMSE [m]', 'FontSize', fontSize);
            title('位置RMSE', 'FontSize', fontSize + 2, 'FontWeight', 'bold');
            legend('Location', 'best', 'FontSize', fontSize - 1);
            grid on; hold off;
            set(gca, 'FontSize', fontSize, 'Box', 'on', 'LineWidth', 0.8);
            
            subplot(2, 3, 3);
            hold on;
            for a = 1:nAlgo
                if strcmp(mkrs{a}, 'none')
                    plot(1:nSteps, velRmse{a}, lStyles{a}, 'Color', colors(a,:), 'LineWidth', estLineWidth, 'DisplayName', labels{a});
                else
                    plot(1:nSteps, velRmse{a}, [lStyles{a} mkrs{a}], 'Color', colors(a,:), 'LineWidth', estLineWidth, ...
                         'MarkerSize', markerSize, 'MarkerIndices', 1:markerInterval:nSteps, 'DisplayName', labels{a});
                end
            end
            xlabel('时间步', 'FontSize', fontSize); ylabel('速度RMSE [m/s]', 'FontSize', fontSize);
            title('速度RMSE', 'FontSize', fontSize + 2, 'FontWeight', 'bold');
            legend('Location', 'best', 'FontSize', fontSize - 1);
            grid on; hold off;
            set(gca, 'FontSize', fontSize, 'Box', 'on', 'LineWidth', 0.8);
            
            subplot(2, 3, 4);
            maneuverColors = struct('CV', [0.000, 0.447, 0.698], 'CA', [0.902, 0.624, 0.000], ...
                                    'CT', [0.000, 0.620, 0.451], 'Stop', [0.5, 0.5, 0.5]);
            maneuverTypes = {'CV', 'CA', 'CT', 'Stop'};
            yMax = max(abs(truthX(2,:))) * 1.3;
            yMin = min(truthX(2,:)) * 1.1;
            if yMin > 0; yMin = 0; end
            hold on;
            
            for t = 1:length(maneuverTypes)
                type = maneuverTypes{t};
                X = [];
                Y = [];
                k = 1;
                while k < nSteps
                    if strcmp(maneuverLabels{k}, type)
                        segStart = k;
                        while k < nSteps && strcmp(maneuverLabels{k}, type)
                            k = k + 1;
                        end
                        segEnd = k;
                        X = [X, segStart, segEnd, segEnd, segStart, NaN];
                        Y = [Y, yMin, yMin, yMax, yMax, NaN];
                    else
                        k = k + 1;
                    end
                end
                if ~isempty(X)
                    patch(X(1:end-1), Y(1:end-1), maneuverColors.(type), ...
                          'FaceAlpha', 0.25, 'EdgeColor', 'none', 'DisplayName', type);
                end
            end
            
            plot(1:nSteps, truthX(2, :), 'k-', 'LineWidth', trueLineWidth, 'DisplayName', '真实vx');
            for a = 1:nAlgo
                est = estStates{a};
                vx = est(2, :);
                if strcmp(mkrs{a}, 'none')
                    plot(1:nSteps, vx, lStyles{a}, 'Color', colors(a,:), 'LineWidth', estLineWidth, 'DisplayName', labels{a});
                else
                    plot(1:nSteps, vx, [lStyles{a} mkrs{a}], 'Color', colors(a,:), 'LineWidth', estLineWidth, ...
                         'MarkerSize', markerSize, 'MarkerIndices', 1:markerInterval:nSteps, 'DisplayName', labels{a});
                end
            end
            xlabel('时间步', 'FontSize', fontSize); ylabel('X方向速度 [m/s]', 'FontSize', fontSize);
            title('速度估计与机动段', 'FontSize', fontSize + 2, 'FontWeight', 'bold');
            legend('Location', 'bestoutside', 'NumColumns', 2, 'FontSize', fontSize - 2);
            grid on; hold off;
            set(gca, 'FontSize', fontSize, 'Box', 'on', 'LineWidth', 0.8);
            
            subplot(2, 3, 5);
            meanPos = cellfun(@mean, posRmse);
            meanVel = cellfun(@mean, velRmse);
            barData = [meanPos'; meanVel'];
            b = bar(barData);
            b(1).FaceColor = 'flat'; 
            b(2).FaceColor = 'flat';
            for a = 1:nAlgo
                b(1).CData(a,:) = colors(a,:);
                b(2).CData(a,:) = min(colors(a,:) + 0.3, 1);
            end
            set(gca, 'XTickLabel', labels, 'FontSize', fontSize);
            ylabel('RMSE', 'FontSize', fontSize);
            title('均值RMSE对比', 'FontSize', fontSize + 2, 'FontWeight', 'bold');
            legend({'位置', '速度'}, 'Location', 'best', 'FontSize', fontSize - 1);
            grid on;
            set(gca, 'FontSize', fontSize, 'Box', 'on', 'LineWidth', 0.8);
            
            subplot(2, 3, 6); axis off;
            txt = {'=== 算法汇总 ===', ''};
            for a = 1:nAlgo
                txt{end+1} = sprintf('%s: 位置=%.2fm, 速度=%.2fm/s', ...
                    labels{a}, meanPos(a), meanVel(a));
            end
            txt{end+1} = '';
            txt{end+1} = '=== 机动段 ===';
            txt{end+1} = '蓝色: CV, 橙色: CA, 绿色: CT';
            text(0.05, 0.95, txt, 'VerticalAlignment', 'top', ...
                 'FontSize', fontSize, 'FontName', 'FixedWidth');
        end

    end
end
