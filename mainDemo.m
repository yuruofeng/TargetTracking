% mainDemo.m
% =========================================================================
%  Unified Target Tracking Demo
%  Runs both Detect-Before-Track (DBT) and Track-Before-Detect (TBD)
%  algorithms via the TargetTracker static class.
%
%  Usage:
%      >> mainDemo          % runs both demos with default settings
%
%  User Options (modify below):
%      demoMode   — 'both', 'dbt', or 'tbd'
%      mcRuns     — number of Monte Carlo runs for DBT
%      saveData   — true/false to export workspace results to .mat
%      saveFile   — output filename (used only when saveData == true)
% =========================================================================

close all; clear; clc;

%% ========================= USER OPTIONS =================================
demoMode   = 'both';           % 'both' | 'dbt' | 'tbd'
mcRuns     = 1;                % Monte Carlo runs for DBT filters
saveData   = false;            % set true to save results to .mat
saveFile   = 'tracking_results.mat';
% =========================================================================

results = struct();

%% ========================================================================
%                   DETECT-BEFORE-TRACK (DBT) DEMO
% =========================================================================
if ismember(demoMode, {'both', 'dbt'})
    fprintf('========== DBT Demo ==========\n');
    dbtCfg = TargetTracker.defaultDbtConfig();

    nK = dbtCfg.numSteps;
    nD = dbtCfg.stateDim;

    % Pre-allocate RMSE accumulators
    rmseAcc = struct( ...
        'posEkf', zeros(mcRuns, nK), 'velEkf', zeros(mcRuns, nK), ...
        'posUkf', zeros(mcRuns, nK), 'velUkf', zeros(mcRuns, nK), ...
        'posCkf', zeros(mcRuns, nK), 'velCkf', zeros(mcRuns, nK), ...
        'posPf',  zeros(mcRuns, nK), 'velPf',  zeros(mcRuns, nK));

    for iMc = 1:mcRuns
        % Generate scenario
        [truthX, meas] = TargetTracker.generateDbtScenario(dbtCfg);

        % Common initial conditions
        x0 = dbtCfg.initState;
        P0 = blkdiag(10*eye(4), pi/90);

        % Filter states
        xEkf = x0;  pEkf = P0;
        xUkf = x0;  pUkf = P0;
        xCkf = x0;  pCkf = P0;
        [wPf, xPf] = TargetTracker.pfInit(x0, P0, dbtCfg);

        estEkf = zeros(nD, nK);
        estUkf = zeros(nD, nK);
        estCkf = zeros(nD, nK);
        estPf  = zeros(nD, nK);
        elapsed = struct('ekf', 0, 'ukf', 0, 'ckf', 0, 'pf', 0);

        for k = 1:nK
            z = meas(:, k);

            % EKF
            tic;
            [xEkfPre, pEkfPre] = TargetTracker.ekfPredict(xEkf, pEkf, dbtCfg);
            [xEkf, pEkf]       = TargetTracker.ekfUpdate(z, xEkfPre, pEkfPre, dbtCfg);
            elapsed.ekf = elapsed.ekf + toc;
            estEkf(:, k) = xEkf;

            % UKF
            tic;
            [wSp, xUkfPre, pUkfPre] = TargetTracker.ukfPredict(xUkf, pUkf, dbtCfg);
            [xUkf, pUkf]            = TargetTracker.ukfUpdate(z, xUkfPre, pUkfPre, wSp, dbtCfg);
            elapsed.ukf = elapsed.ukf + toc;
            estUkf(:, k) = xUkf;

            % CKF
            tic;
            [xCkfPre, pCkfPre] = TargetTracker.ckfPredict(xCkf, pCkf, dbtCfg);
            [xCkf, pCkf]       = TargetTracker.ckfUpdate(z, xCkfPre, pCkfPre, dbtCfg);
            elapsed.ckf = elapsed.ckf + toc;
            estCkf(:, k) = xCkf;

            % PF
            tic;
            [wPfPre, xPfPre] = TargetTracker.pfPredict(wPf, xPf, dbtCfg);
            [wPf, xPf]       = TargetTracker.pfUpdate(z, wPfPre, xPfPre, dbtCfg);
            [wPf, xPf]       = TargetTracker.pfResample(wPf, xPf, dbtCfg);
            elapsed.pf = elapsed.pf + toc;
            estPf(:, k) = xPf * wPf;

            % Per-step RMSE
            posIdx = [1 3];  velIdx = [2 4];
            rmseAcc.posEkf(iMc, k) = norm(xEkf(posIdx) - truthX(posIdx, k));
            rmseAcc.velEkf(iMc, k) = norm(xEkf(velIdx) - truthX(velIdx, k));
            rmseAcc.posUkf(iMc, k) = norm(xUkf(posIdx) - truthX(posIdx, k));
            rmseAcc.velUkf(iMc, k) = norm(xUkf(velIdx) - truthX(velIdx, k));
            rmseAcc.posCkf(iMc, k) = norm(xCkf(posIdx) - truthX(posIdx, k));
            rmseAcc.velCkf(iMc, k) = norm(xCkf(velIdx) - truthX(velIdx, k));
            rmseAcc.posPf(iMc, k)  = norm(estPf(posIdx, k) - truthX(posIdx, k));
            rmseAcc.velPf(iMc, k)  = norm(estPf(velIdx, k) - truthX(velIdx, k));
        end

        fprintf('MC run %d/%d — Elapsed [s]: EKF %.3f | UKF %.3f | CKF %.3f | PF %.3f\n', ...
            iMc, mcRuns, elapsed.ekf, elapsed.ukf, elapsed.ckf, elapsed.pf);
    end

    % Average RMSE across MC runs
    rmse = struct();
    fNames = fieldnames(rmseAcc);
    for i = 1:numel(fNames)
        rmse.(fNames{i}) = mean(rmseAcc.(fNames{i}), 1);
    end

    % Plot
    TargetTracker.plotDbtResults(truthX, meas, estEkf, estUkf, estCkf, estPf, ...
                                rmse, nK);

    results.dbt = struct('truthX', truthX, 'meas', meas, ...
        'estEkf', estEkf, 'estUkf', estUkf, 'estCkf', estCkf, 'estPf', estPf, ...
        'rmse', rmse, 'elapsed', elapsed);
    fprintf('DBT demo complete.\n\n');
end

%% ========================================================================
%                   TRACK-BEFORE-DETECT (TBD) DEMO
% =========================================================================
if ismember(demoMode, {'both', 'tbd'})
    fprintf('========== TBD Demo ==========\n');
    tbdCfg = TargetTracker.defaultTbdConfig();
    rng(tbdCfg.rngSeed);

    fprintf('Grid: %dx%d | Frames: %d | SNR ~ %.1f dB\n', ...
        tbdCfg.gridSize(1), tbdCfg.gridSize(2), tbdCfg.numFrames, ...
        20*log10(tbdCfg.amplitude / tbdCfg.noiseStd));

    % Scenario
    trueState = TargetTracker.generateTbdTrajectory(tbdCfg);
    [measData, psfKernel] = TargetTracker.generateTbdMeasurement(trueState, tbdCfg);

    fprintf('  Start: (%.1f, %.1f)  vel: (%.1f, %.1f)\n', ...
        trueState(1,1), trueState(1,2), trueState(1,3), trueState(1,4));

    % DP-TBD
    fprintf('Running DP-TBD ...\n');
    tic;
    [dpTrack, dpScore, ~] = TargetTracker.runDpTbd(measData, psfKernel, tbdCfg);
    dpTime = toc;
    fprintf('  DP-TBD: %.2f s | mean pos RMSE: %.2f px\n', ...
        dpTime, mean(sqrt(sum((dpTrack - trueState(:,1:2)).^2, 2))));

    % PF-TBD
    fprintf('Running PF-TBD (%d particles) ...\n', tbdCfg.numParticles);
    tic;
    [pfState, pfPosRmse, pfVelRmse] = TargetTracker.runPfTbd( ...
        measData, trueState, psfKernel, tbdCfg);
    pfTime = toc;
    fprintf('  PF-TBD: %.2f s | mean pos RMSE: %.2f px | mean vel RMSE: %.2f\n', ...
        pfTime, mean(pfPosRmse), mean(pfVelRmse));

    % Plot
    TargetTracker.plotTbdResults(trueState, dpTrack, pfState, measData, ...
                                pfPosRmse, pfVelRmse, dpScore, tbdCfg);

    results.tbd = struct('trueState', trueState, 'measData', measData, ...
        'dpTrack', dpTrack, 'dpScore', dpScore, 'pfState', pfState, ...
        'pfPosRmse', pfPosRmse, 'pfVelRmse', pfVelRmse);
    fprintf('TBD demo complete.\n\n');
end

%% ========================= SAVE RESULTS =================================
if saveData
    save(saveFile, '-struct', 'results');
    fprintf('Results saved to %s\n', saveFile);
end

fprintf('=== All done ===\n');
