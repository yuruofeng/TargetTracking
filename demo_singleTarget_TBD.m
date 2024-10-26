% track before detect for single target tracking
%
% 注意：
%   1. 目标存在跟丢的风险；
%   2.（代码可能出现报错）似然函数可能存在超过double可表示最大值的风险；

close all; clear; clc;

%% 参数设置
% ---- 系统参数
dimState  = 5; % 状态向量维度（x, y, vx, vy, L）
dt        = 1; % 时间步长
sigma_x   = 3; % x方向过程噪声标准差
sigma_y   = 3; % y方向过程噪声标准差
sigma_vx  = 1; % x方向速度过程噪声标准差
sigma_vy  = 1; % y方向速度过程噪声标准差
sigma_L   = 0.1; % 目标长度过程噪声标准差

% 测量参数
delta_x = 2; delta_y = 2; % 测量分辨率
meas_X = 10:delta_x:88;   % x测量范围
meas_Y = 10:delta_y:88;   % y测量范围
L_X = 10; X = 3*delta_x;  % 距离单元相关常数
L_Y = 10; Y = 3*delta_y;  % 方位单元相关常数
SNR = 20;                 % 信噪比
N_x = length(meas_X);     % x观测分辨单元数
N_y = length(meas_Y);     % y观测分辨单元数

[gridX, gridY] = meshgrid(meas_X, meas_Y);

% 点扩散函数
h = @(x, y) exp( -(x-gridX).^2*L_X/(2*X) -(y-gridY).^2*L_Y/(2*Y) );

% 模拟时间步数
numSteps = 20;

% ---- 粒子滤波参数
numParticles = 500; % 粒子数量


%% 生成航迹
% 目标初始状态（CV模型）
x0  = 85; % 初始x位置（米转换为千米）
y0  = 85; % 初始y位置
vx0 = -2; % 初始x方向速度
vy0 = -3; % 初始y方向速度
L0  = 10; % 初始目标强度

% 真实状态初始化
trueStates = zeros(numSteps, dimState);
trueStates(1, 1) = x0;
trueStates(1, 2) = y0;
trueStates(1, 3) = vx0;
trueStates(1, 4) = vy0;
trueStates(1, 5) = raylrnd(L0);

for k = 2:numSteps
    trueStates(k, 1) = trueStates(k - 1, 1) + trueStates(k - 1, 3) * dt;
    trueStates(k, 2) = trueStates(k - 1, 2) + trueStates(k - 1, 4) * dt;
    trueStates(k, 3) = trueStates(k - 1, 3);
    trueStates(k, 4) = trueStates(k - 1, 4);
    trueStates(k, 5) = raylrnd(L0);
end

%% 生成量测
measuredData = zeros(N_x, N_y, numSteps); % 假设测量数据是三维的
sigma_n = zeros(1, numSteps);
for k = 1:numSteps
    targetAmplitude = trueStates(k, 5);
    sigma_n(1, k) =  sqrt(targetAmplitude.^2 / 10^(SNR/10));  % 量测噪声标准差
    backgroundNoise = raylrnd(sigma_n(1, k), N_x, N_y);       % 量测噪声生成
    targetmapping   = trueStates(k, 5) .* h(trueStates(k, 1), trueStates(k, 2)); % 目标量测生成
    % ----
    measuredData(:, :, k) = targetmapping + backgroundNoise;
end

%% 滤波
% 初始化粒子和权重
particles = trueStates(1, :) + randn(numParticles, dimState);
weights   = ones(numParticles, 1) / numParticles;
stateEst  = zeros(numSteps, dimState);
posRMSE      = zeros(1, numSteps);
velRMSE      = zeros(1, numSteps);
q = waitbar(0, 'filtering');
% ----
for j = 1:numSteps
    waitbar(j./numSteps, q, 'filtering');
    for ip = 1:numParticles
        % 预测（状态预测）
        particles(ip, 1) = particles(ip, 1) + particles(ip, 3)*dt + sigma_x*randn;
        particles(ip, 2) = particles(ip, 2) + particles(ip, 4)*dt + sigma_y*randn;
        particles(ip, 3) = particles(ip, 3) + sigma_vx*randn;
        particles(ip, 4) = particles(ip, 4) + sigma_vy*randn;
        particles(ip, 5) = particles(ip, 5) + abs(sigma_L*randn);
        % 更新（权重更新）
        % ---- H1假设
        B1 = particles(ip, 5) .* h(particles(ip, 1), particles(ip, 2)) + sigma_n(1, j);
        p1 = raylpdf(measuredData(:, :, j), B1);
        % ---- H0假设
        B0 = sigma_n(1, j);
        p0 = raylpdf(measuredData(:, :, j), B0);
        % 似然比
        likelihoodRatio = p1./p0;
        % likelihoodRatio(likelihoodRatio>1e100) = 1e100;
        likelihood = prod(likelihoodRatio, "all");
        weights(ip, 1) = weights(ip, 1)*likelihood;
    end
    weights = weights ./ sum(weights); % 归一化
    
    % 状态估计
    stateEst(j, :) = weights' * particles;
    
    % ---- 重采样
    indexResample = randsample(1:length(weights), numParticles, true, weights);
    weights = ones(numParticles, 1) / numParticles;
    particles = particles(indexResample, :);
    
    % 计算误差
    posRMSE(1, j) = sqrt(sum((stateEst(j, 1:2) - trueStates(j, 1:2)).^2));
    velRMSE(1, j) = sqrt(sum((stateEst(j, 3:4) - trueStates(j, 3:4)).^2));
end
close(q);

figure;
imagesc(measuredData(:, :, 18));
title('观测数据示例');

figure;
subplot(3, 1, 1); hold on; grid on; box on;
plot(stateEst(:, 1), stateEst(:, 2), 'ro');
plot(trueStates(:, 1),trueStates(:, 2), 'bx');
title('目标估计结果');

subplot(3, 1, 2);
plot(1:numSteps, posRMSE);
title('位置估计RMSE');

subplot(3, 1, 3);
plot(1:numSteps, velRMSE);
title('速度估计RMSE');