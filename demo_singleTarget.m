clear; close all; clc

filter = SingleTargetFilter;
filter = filter.gen_model;
MCRuns = 1;     %
%% 扩展卡尔曼滤波EKF
% 初始化————航迹起始方法可以参考其他文献
RMSE_posEKF = zeros(MCRuns,filter.K);
RMSE_velEKF = zeros(MCRuns,filter.K);
RMSE_posUKF = zeros(MCRuns,filter.K); 
RMSE_velUKF = zeros(MCRuns,filter.K);
RMSE_posCKF = zeros(MCRuns,filter.K); 
RMSE_velCKF = zeros(MCRuns,filter.K);
RMSE_posPF = zeros(MCRuns,filter.K);
RMSE_velPF = zeros(MCRuns,filter.K);

for iMCruns = 1:MCRuns
    stateUpd_EKF = [ 0; 6; 0; 1; 0.02 ];
    covarUpd_EKF = blkdiag(10*eye(4),pi/90);

    stateUpd_UKF = [ 0; 6; 0; 1; 0.02 ];
    covarUpd_UKF = blkdiag(10*eye(4),pi/90);

    stateUpd_CKF = [ 0; 6; 0; 1; 0.02 ];
    covarUpd_CKF = blkdiag(10*eye(4),pi/90);

    state_init = [ 0; 6; 0; 1; 0.02 ];
    covar_init = blkdiag(10*eye(4),pi/90);
    [weightUpd_PF,stateUpd_PF] = filter.particles_init(state_init,covar_init);

    est_EKF = zeros(filter.targetStateDim,filter.K);
    est_UKF = zeros(filter.targetStateDim,filter.K);
    est_CKF = zeros(filter.targetStateDim,filter.K);
    est_PF = zeros(filter.targetStateDim,filter.K);

    tEKF = 0; tUKF = 0; tCKF = 0; tPF = 0;
    for k = 1:filter.K
        h = waitbar(iMCruns/MCRuns);
        %%
        tic
        % EKF预测
        [statePre_EFK,covarPre_EKF] = filter.EKFpredict(stateUpd_EKF,covarUpd_EKF);
        % EKF校正
        [stateUpd_EKF,covarUpd_EKF] = filter.EKFupdate(filter.meas(:,k),statePre_EFK,covarPre_EKF);
        % 保存滤波结果
        est_EKF(:,k) = stateUpd_EKF;
        RMSE_posEKF(iMCruns,k) = sqrt(sum(stateUpd_EKF([1 3])-filter.truth_X([1 3],k)).^2);
        RMSE_velEKF(iMCruns,k) = sqrt(sum(stateUpd_EKF([2 4])-filter.truth_X([2 4],k)).^2);
        tEKF = tEKF+toc;
        %%
        tic
        % UKF预测
        [weightState_SP,statePre_UFK,covarPre_UKF] = filter.UKFpredict(stateUpd_UKF,covarUpd_UKF);
        % UKF校正
        [stateUpd_UKF,covarUpd_UKF] = filter.UKFupdate(filter.meas(:,k),statePre_UFK,covarPre_UKF,weightState_SP);
        % 保存滤波结果
        est_UKF(:,k) = stateUpd_UKF;
        RMSE_posUKF(iMCruns,k) = sqrt(sum(stateUpd_UKF([1 3])-filter.truth_X([1 3],k)).^2);
        RMSE_velUKF(iMCruns,k) = sqrt(sum(stateUpd_UKF([2 4])-filter.truth_X([2 4],k)).^2);
        tUKF = tUKF+toc;
        %%
        tic
        % CKF预测
        [statePre_CFK,covarPre_CKF] = filter.CKFpredict(stateUpd_CKF,covarUpd_CKF);
        % CKF校正
        [stateUpd_CKF,covarUpd_CKF] = filter.CKFupdate(filter.meas(:,k),statePre_CFK,covarPre_CKF);
        % 保存滤波结果
        est_CKF(:,k) = stateUpd_CKF;
        RMSE_posCKF(iMCruns,k) = sqrt(sum(stateUpd_CKF([1 3])-filter.truth_X([1 3],k)).^2);
        RMSE_velCKF(iMCruns,k) = sqrt(sum(stateUpd_CKF([2 4])-filter.truth_X([2 4],k)).^2);
        tCKF = tCKF+toc;
        %%
        tic
        % PF预测
        [weightPre_PF,statePre_PF] = filter.PFpredict(weightUpd_PF,stateUpd_PF);
        % PF校正
        [weightUpd_PF,stateUpd_PF] = filter.PFupdate(filter.meas(:,k),weightPre_PF,statePre_PF);
        % PF重采样
        [weightUpd_PF,stateUpd_PF] = filter.resampling(weightUpd_PF,stateUpd_PF);
        % 保存滤波结果
        est_PF(:,k) = stateUpd_PF*weightUpd_PF;
        RMSE_posPF(iMCruns,k) = sqrt(sum(est_PF([1 3],k)-filter.truth_X([1 3],k)).^2);
        RMSE_velPF(iMCruns,k) = sqrt(sum(est_PF([2 4],k)-filter.truth_X([2 4],k)).^2);
        tPF = tPF+toc;

    end
    %     disp(['Current Iteration is ',num2str(iMCruns),'.']);
    %
    disp('========================');
    disp('耗费时间/s：');
    disp(['EKF:',num2str(tEKF)]);
    disp(['UKF:',num2str(tUKF)]);
    disp(['CKF:',num2str(tCKF)]);
    disp(['PF:',num2str(tPF)]);
end
close(h);
RMSE_posEKF = mean(RMSE_posEKF,1); RMSE_velEKF = mean(RMSE_velEKF,1);
RMSE_posUKF = mean(RMSE_posUKF,1); RMSE_velUKF = mean(RMSE_velUKF,1);
RMSE_posCKF = mean(RMSE_posCKF,1); RMSE_velCKF = mean(RMSE_velCKF,1);
RMSE_posPF = mean(RMSE_posPF,1); RMSE_velPF = mean(RMSE_velPF,1);

%% 画图
figure; 
subplot(311);
xlabel('x轴/m'); ylabel('y轴/m');
plot(filter.truth_X(1,:),filter.truth_X(3,:),'k-.');hold on;
plot(filter.meas(2,:).*cos(filter.meas(1,:)),filter.meas(2,:).*sin(filter.meas(1,:)),'ro')
plot(est_EKF(1,:),est_EKF(3,:),'c-v');
plot(est_UKF(1,:),est_UKF(3,:),'g-x');
plot(est_CKF(1,:),est_CKF(3,:),'b-*');
plot(est_CKF(1,:),est_PF(3,:),'m-s');
legend('真实航迹','测量数据','EKF估计结果','UKF估计结果','CKF估计结果','PF估计结果','Location','northwest'); grid on; grid minor; 
subplot(312);
plot(1:filter.K,RMSE_posEKF,'c.-','LineWidth',1.5); hold on;
plot(1:filter.K,RMSE_posUKF,'g.-','LineWidth',1.5);
plot(1:filter.K,RMSE_posCKF,'b.-','LineWidth',1.5);
plot(1:filter.K,RMSE_posPF,'m.-','LineWidth',1.5);
xlabel('采样时刻/s'); ylabel('估计RMSE/m');grid on; grid minor; 
legend('EKF位置RMSE','UKF位置RMSE','CKF位置RMSE','PF位置RMSE','Location','northwest');
subplot(313);
plot(1:filter.K,RMSE_velEKF,'c.-','LineWidth',1.5); hold on;
plot(1:filter.K,RMSE_velUKF,'g.-','LineWidth',1.5);
plot(1:filter.K,RMSE_velCKF,'b.-','LineWidth',1.5);
plot(1:filter.K,RMSE_velPF,'m.-','LineWidth',1.5);
xlabel('采样时刻/s'); ylabel('估计RMSE/m');grid on; grid minor; 
legend('EKF速度RMSE','UKF速度RMSE','CKF速度RMSE','PF速度RMSE','Location','northwest');
