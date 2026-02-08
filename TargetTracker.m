classdef TargetTracker
% TARGETTRACKER  Unified target tracking algorithm suite (all static).
%   Provides both Detect-Before-Track (DBT) and Track-Before-Detect (TBD)
%   algorithms for single-target tracking scenarios.
%
%   DBT Filters (Constant Turn-Rate model, radar range/azimuth measurements):
%       Extended Kalman Filter  (EKF)
%       Unscented Kalman Filter (UKF)
%       Cubature Kalman Filter  (CKF)
%       Particle Filter         (PF)
%
%   TBD Algorithms (Constant Velocity model, image-domain measurements):
%       Dynamic Programming TBD (DP-TBD)
%       Particle Filter TBD    (PF-TBD)
%
%   Usage:
%       dbtCfg = TargetTracker.defaultDbtConfig();
%       tbdCfg = TargetTracker.defaultTbdConfig();
%
%   Reference:
%       [1] He You. Radar Data Processing and Applications (Chinese).
%       [2] Systematic resampling adapted from open-source implementations.
% =========================================================================

    methods (Static)

        %% ================================================================
        %                   CONFIGURATION GENERATORS
        % =================================================================

        function cfg = defaultDbtConfig()
        % DEFAULTDBTCONFIG  Return default DBT simulation parameters.
        %   Contains model dimensions, noise covariances, UKF/PF tuning,
        %   and a flag for trajectory + measurement generation.
            cfg.numSteps       = 100;       % total observation frames
            cfg.dt             = 1;         % sampling interval [s]
            cfg.stateDim       = 5;         % [x, vx, y, vy, omega]
            cfg.procNoiseDim   = 3;
            cfg.measDim        = 2;         % [azimuth, range]

            T = cfg.dt;
            cfg.procDriveMat   = [T^2/2 0 0; T 0 0; ...
                                  0 T^2/2 0; 0 T 0; ...
                                  0 0     1];
            cfg.procNoiseCov   = diag([1, 1, 4e-4]);
            cfg.measNoiseCov   = diag([pi/90, 5]);

            % UKF tuning
            cfg.ukfAlpha       = 1e-3;
            cfg.ukfBeta        = 2;
            cfg.ukfKappa       = 0;

            % PF tuning
            cfg.numParticles   = 500;

            % Initial state for trajectory generation
            cfg.initState      = [0; 6; 0; 1; 0.02];
        end

        function cfg = defaultTbdConfig()
        % DEFAULTTBDCONFIG  Return default TBD simulation parameters.
            % Scene
            cfg.gridSize       = [100, 100];
            cfg.numFrames      = 50;
            cfg.dt             = 1;

            % Target (constant-velocity model)
            cfg.initPos        = [15, 20];
            cfg.initVel        = [1.2, 0.8];
            cfg.targetRadius   = 3;
            cfg.amplitude      = 3.0;

            % Noise
            cfg.noiseStd       = 1.0;

            % DP-TBD
            cfg.maxSpeed       = 3;

            % PF-TBD
            cfg.numParticles   = 500;
            cfg.procNoisePos   = 2.0;
            cfg.procNoiseVel   = 0.5;
            cfg.procNoiseAmp   = 0.3;

            % Reproducibility
            cfg.rngSeed        = 2024;
        end

        %% ================================================================
        %                   DBT — MODEL GENERATION
        % =================================================================

        function [truthX, meas] = generateDbtScenario(cfg)
        % GENERATEDBTSCENARIO  Generate CT-model trajectory and radar measurements.
        %   truthX : [stateDim x numSteps]
        %   meas   : [measDim  x numSteps]
            nSteps = cfg.numSteps;
            state  = cfg.initState;

            truthX = zeros(cfg.stateDim, nSteps);
            meas   = zeros(cfg.measDim,  nSteps);

            R = cfg.measNoiseCov;
            for k = 1:nSteps
                F     = TargetTracker.ctDynamicMatrix(cfg.dt, state);
                state = F * state;
                truthX(:, k) = state;

                measNoise  = sqrtm(R) * randn(cfg.measDim, 1);
                meas(:, k) = [atan2(state(3), state(1)); ...
                              norm(state([1 3]))] + measNoise;
            end
        end

        %% ================================================================
        %                   DBT — KALMAN FILTERS
        % =================================================================

        % ---------- EKF ----------
        function [statePre, covarPre] = ekfPredict(stateUpd, covarUpd, cfg)
        % EKFPREDICT  Extended Kalman Filter prediction step.
            F        = TargetTracker.ctDynamicMatrix(cfg.dt, stateUpd);
            Qd       = cfg.procDriveMat * sqrtm(cfg.procNoiseCov) * cfg.procDriveMat';
            statePre = F * stateUpd;
            covarPre = F * covarUpd * F' + Qd;
        end

        function [stateUpd, covarUpd] = ekfUpdate(measZ, statePre, covarPre, cfg)
        % EKFUPDATE  Extended Kalman Filter update step.
            H       = TargetTracker.ctMeasJacobian(statePre);
            measPre = TargetTracker.ctMeasFunc(statePre);
            R       = cfg.measNoiseCov * cfg.measNoiseCov';
            S       = H * covarPre * H' + R;
            K       = covarPre * H' / S;
            stateUpd = statePre + K * (measZ - measPre);
            covarUpd = (eye(cfg.stateDim) - K * H) * covarPre;
        end

        % ---------- UKF ----------
        function [wSp, statePre, covarPre] = ukfPredict(stateUpd, covarUpd, cfg)
        % UKFPREDICT  Unscented Kalman Filter prediction step.
            [wSp, spPts] = TargetTracker.generateSigmaPoints(stateUpd, covarUpd, cfg);
            nSp      = size(spPts, 2);
            preSp    = zeros(cfg.stateDim, nSp);
            for i = 1:nSp
                F = TargetTracker.ctDynamicMatrix(cfg.dt, spPts(:, i));
                preSp(:, i) = F * spPts(:, i);
            end
            statePre = preSp * wSp(1, :)';
            Qd       = cfg.procDriveMat * sqrtm(cfg.procNoiseCov) * cfg.procDriveMat';
            covarPre = Qd;
            for i = 1:nSp
                d = preSp(:, i) - statePre;
                covarPre = covarPre + wSp(2, i) * (d * d');
            end
        end

        function [stateUpd, covarUpd] = ukfUpdate(measZ, statePre, covarPre, wSp, cfg)
        % UKFUPDATE  Unscented Kalman Filter update step.
            [~, spPts] = TargetTracker.generateSigmaPoints(statePre, covarPre, cfg);
            nSp    = size(spPts, 2);
            measSp = zeros(cfg.measDim, nSp);
            for i = 1:nSp
                measSp(:, i) = TargetTracker.ctMeasFunc(spPts(:, i));
            end
            measPre = measSp * wSp(1, :)';

            R  = cfg.measNoiseCov * cfg.measNoiseCov';
            Sz = R;
            Cxz = zeros(cfg.stateDim, cfg.measDim);
            for i = 1:nSp
                dz = measSp(:, i) - measPre;
                dx = spPts(:, i)  - statePre;
                Sz  = Sz  + wSp(2, i) * (dz * dz');
                Cxz = Cxz + wSp(2, i) * (dx * dz');
            end
            K        = Cxz / Sz;
            stateUpd = statePre + K * (measZ - measPre);
            covarUpd = covarPre - K * Sz * K';
        end

        % ---------- CKF ----------
        function [statePre, covarPre] = ckfPredict(stateUpd, covarUpd, cfg)
        % CKFPREDICT  Cubature Kalman Filter prediction step.
            [wCp, cpPts] = TargetTracker.generateCubaturePoints(stateUpd, covarUpd, cfg);
            nCp   = size(cpPts, 2);
            preCp = zeros(cfg.stateDim, nCp);
            for i = 1:nCp
                F = TargetTracker.ctDynamicMatrix(cfg.dt, cpPts(:, i));
                preCp(:, i) = F * cpPts(:, i);
            end
            statePre = preCp * wCp';
            Qd       = cfg.procDriveMat * sqrtm(cfg.procNoiseCov) * cfg.procDriveMat';
            covarPre = Qd - statePre * statePre';
            for i = 1:nCp
                covarPre = covarPre + wCp(i) * (preCp(:, i) * preCp(:, i)');
            end
        end

        function [stateUpd, covarUpd] = ckfUpdate(measZ, statePre, covarPre, cfg)
        % CKFUPDATE  Cubature Kalman Filter update step.
            [wCp, cpPts] = TargetTracker.generateCubaturePoints(statePre, covarPre, cfg);
            nCp    = size(cpPts, 2);
            measCp = zeros(cfg.measDim, nCp);
            for i = 1:nCp
                measCp(:, i) = TargetTracker.ctMeasFunc(cpPts(:, i));
            end
            measPre = measCp * wCp';

            R  = cfg.measNoiseCov * cfg.measNoiseCov';
            Sz = R - measPre * measPre';
            Cxz = -statePre * measPre';
            for i = 1:nCp
                Sz  = Sz  + wCp(i) * (measCp(:, i) * measCp(:, i)');
                Cxz = Cxz + wCp(i) * (cpPts(:, i)  * measCp(:, i)');
            end
            K        = Cxz / Sz;
            stateUpd = statePre + K * (measZ - measPre);
            covarUpd = covarPre - K * Sz * K';
        end

        % ---------- DBT Particle Filter ----------
        function [wPf, xPf] = pfInit(stateInit, covarInit, cfg)
        % PFINIT  Initialise particles from a Gaussian prior.
            nP  = cfg.numParticles;
            xPf = mvnrnd(stateInit, covarInit, nP)';
            wPf = ones(nP, 1) / nP;
        end

        function [wPre, xPre] = pfPredict(wUpd, xUpd, cfg)
        % PFPREDICT  Propagate particles through CT dynamic model.
            nP   = cfg.numParticles;
            Qd   = cfg.procDriveMat * sqrtm(cfg.procNoiseCov) * cfg.procDriveMat';
            xPre = zeros(cfg.stateDim, nP);
            for i = 1:nP
                F = TargetTracker.ctDynamicMatrix(cfg.dt, xUpd(:, i));
                xPre(:, i) = F * xUpd(:, i) + mvnrnd(zeros(1, cfg.stateDim), Qd)';
            end
            wPre = wUpd;
        end

        function [wUpd, xUpd] = pfUpdate(measZ, wPre, xPre, cfg)
        % PFUPDATE  Update particle weights using measurement likelihood.
            R = cfg.measNoiseCov * cfg.measNoiseCov';
            measPre = [atan2(xPre(3, :), xPre(1, :)); ...
                       sqrt(sum(xPre([1 3], :).^2, 1))];
            wUpd = wPre .* mvnpdf(measPre', measZ', R);
            wUpd = wUpd / sum(wUpd);
            xUpd = xPre;
        end

        function [wOut, xOut] = pfResample(wIn, xIn, cfg)
        % PFRESAMPLE  Systematic resampling for DBT particle filter.
            nP  = cfg.numParticles;
            idx = TargetTracker.systematicResample(wIn, nP);
            idx = idx(randperm(nP));
            wOut = ones(nP, 1) / nP;
            xOut = xIn(:, idx);
        end

        %% ================================================================
        %                   TBD — SCENARIO GENERATION
        % =================================================================

        function trueState = generateTbdTrajectory(cfg)
        % GENERATETBDTRAJECTORY  Constant-velocity target trajectory.
        %   trueState : [numFrames x 5] = [row, col, vRow, vCol, amp]
            nF = cfg.numFrames;
            trueState = zeros(nF, 5);
            trueState(1, :) = [cfg.initPos, cfg.initVel, cfg.amplitude];
            for t = 2:nF
                trueState(t, 1:2) = trueState(t-1, 1:2) + trueState(t-1, 3:4) * cfg.dt;
                trueState(t, 3:4) = trueState(t-1, 3:4);
                trueState(t, 5)   = cfg.amplitude;
            end
        end

        function [measData, psfKernel] = generateTbdMeasurement(trueState, cfg)
        % GENERATETBDMEASUREMENT  Simulate image-domain measurement frames.
        %   measData  : [rows x cols x numFrames]
        %   psfKernel : normalised Gaussian PSF
            gs = cfg.gridSize;  nF = cfg.numFrames;  r = cfg.targetRadius;

            % Gaussian PSF (peak-normalised)
            [kR, kC]  = meshgrid(-r:r, -r:r);
            psfSigma  = r / 2;
            psfKernel = exp(-(kR.^2 + kC.^2) / (2 * psfSigma^2));
            psfKernel = psfKernel / max(psfKernel(:));

            measData = zeros(gs(1), gs(2), nF);
            for t = 1:nF
                signal = zeros(gs);
                cRow = round(trueState(t, 1));
                cCol = round(trueState(t, 2));
                amp  = trueState(t, 5);
                for di = -r:r
                    for dj = -r:r
                        ri = cRow + di;  cj = cCol + dj;
                        if ri >= 1 && ri <= gs(1) && cj >= 1 && cj <= gs(2)
                            signal(ri, cj) = signal(ri, cj) + ...
                                amp * psfKernel(di + r + 1, dj + r + 1);
                        end
                    end
                end
                measData(:, :, t) = signal + cfg.noiseStd * randn(gs);
            end
        end

        %% ================================================================
        %                   TBD — DP-TBD
        % =================================================================

        function detMap = computeDetectionMap(measData, psfKernel, cfg)
        % COMPUTEDETECTIONMAP  Matched-filter detection map.
            nF = cfg.numFrames;  gs = cfg.gridSize;
            detMap = zeros(gs(1), gs(2), nF);
            for t = 1:nF
                detMap(:, :, t) = conv2(measData(:, :, t), psfKernel, 'same') ...
                                  / (cfg.noiseStd^2);
            end
        end

        function [estTrack, maxScore, valueFunc] = runDpTbd(measData, psfKernel, cfg)
        % RUNDPTBD  Dynamic-programming track-before-detect.
        %   estTrack  : [numFrames x 2] estimated positions [row, col]
        %   maxScore  : accumulated score at the final frame
        %   valueFunc : [rows x cols x numFrames] value function
            detMap  = TargetTracker.computeDetectionMap(measData, psfKernel, cfg);
            gs      = cfg.gridSize;
            nF      = cfg.numFrames;
            vMax    = cfg.maxSpeed;

            valueFunc = -inf(gs(1), gs(2), nF);
            backPtr   = zeros(gs(1), gs(2), nF, 2);
            valueFunc(:, :, 1) = detMap(:, :, 1);

            [dR, dC]   = meshgrid(-vMax:vMax, -vMax:vMax);
            nbOffsets  = [dR(:), dC(:)];
            nNb        = size(nbOffsets, 1);

            % Forward recursion
            for t = 2:nF
                for r = 1:gs(1)
                    for c = 1:gs(2)
                        bestScore = -inf;
                        bestPrev  = [0, 0];
                        for k = 1:nNb
                            pr = r - nbOffsets(k, 1);
                            pc = c - nbOffsets(k, 2);
                            if pr >= 1 && pr <= gs(1) && pc >= 1 && pc <= gs(2)
                                cs = valueFunc(pr, pc, t-1);
                                if cs > bestScore
                                    bestScore = cs;
                                    bestPrev  = [pr, pc];
                                end
                            end
                        end
                        if bestScore > -inf
                            valueFunc(r, c, t) = detMap(r, c, t) + bestScore;
                            backPtr(r, c, t, :) = bestPrev;
                        end
                    end
                end
            end

            % Backtrack
            finalVal = valueFunc(:, :, nF);
            [maxScore, maxIdx] = max(finalVal(:));
            [rEnd, cEnd] = ind2sub(gs, maxIdx);

            estTrack = zeros(nF, 2);
            estTrack(nF, :) = [rEnd, cEnd];
            cr = rEnd;  cc = cEnd;
            for t = nF:-1:2
                prev = squeeze(backPtr(cr, cc, t, :))';
                estTrack(t-1, :) = prev;
                cr = prev(1);  cc = prev(2);
            end
        end

        %% ================================================================
        %                   TBD — PF-TBD
        % =================================================================

        function [estState, posRmse, velRmse] = runPfTbd(measData, trueState, psfKernel, cfg)
        % RUNPFTBD  Particle-filter track-before-detect with log-likelihood weighting.
        %   estState : [numFrames x 5]
        %   posRmse  : [1 x numFrames]
        %   velRmse  : [1 x numFrames]
            nF       = cfg.numFrames;
            gs       = cfg.gridSize;
            nP       = cfg.numParticles;
            noiseStd = cfg.noiseStd;
            r        = cfg.targetRadius;
            dimState = 5;

            initSpread = [3, 3, 0.5, 0.5, 0.3];
            particles  = repmat(trueState(1, :), nP, 1) + ...
                         randn(nP, dimState) .* initSpread;
            particles(:, 5) = abs(particles(:, 5));

            logW     = zeros(nP, 1);
            estState = zeros(nF, dimState);
            posRmse  = zeros(1, nF);
            velRmse  = zeros(1, nF);

            for t = 1:nF
                % Prediction (CV + random walk on amplitude)
                particles(:, 1) = particles(:, 1) + particles(:, 3)*cfg.dt ...
                                   + cfg.procNoisePos * randn(nP, 1);
                particles(:, 2) = particles(:, 2) + particles(:, 4)*cfg.dt ...
                                   + cfg.procNoisePos * randn(nP, 1);
                particles(:, 3) = particles(:, 3) + cfg.procNoiseVel * randn(nP, 1);
                particles(:, 4) = particles(:, 4) + cfg.procNoiseVel * randn(nP, 1);
                particles(:, 5) = abs(particles(:, 5) + cfg.procNoiseAmp * randn(nP, 1));

                % Update (log-likelihood ratio)
                frame = measData(:, :, t);
                for ip = 1:nP
                    cRow = round(particles(ip, 1));
                    cCol = round(particles(ip, 2));
                    amp  = particles(ip, 5);
                    logLR = 0;
                    for di = -r:r
                        for dj = -r:r
                            ri = cRow + di;  cj = cCol + dj;
                            if ri >= 1 && ri <= gs(1) && cj >= 1 && cj <= gs(2)
                                hVal = amp * psfKernel(di + r + 1, dj + r + 1);
                                z    = frame(ri, cj);
                                logLR = logLR + (z * hVal - 0.5 * hVal^2) / noiseStd^2;
                            end
                        end
                    end
                    logW(ip) = logW(ip) + logLR;
                end

                % Normalise via log-sum-exp
                maxLogW = max(logW);
                w = exp(logW - maxLogW);
                w = w / sum(w);

                % Weighted state estimate
                estState(t, :) = w' * particles;

                % Systematic resampling
                idx       = TargetTracker.systematicResample(w, nP);
                particles = particles(idx, :);
                logW      = zeros(nP, 1);

                % Per-frame RMSE
                posRmse(t) = sqrt(sum((estState(t, 1:2) - trueState(t, 1:2)).^2));
                velRmse(t) = sqrt(sum((estState(t, 3:4) - trueState(t, 3:4)).^2));
            end
        end

        %% ================================================================
        %                   VISUALIZATION
        % =================================================================

        function plotDbtResults(truthX, meas, estEkf, estUkf, estCkf, estPf, ...
                               rmse, nSteps)
        % PLOTDBTRESULTS  Three-panel DBT comparison figure.
        %   rmse : struct with fields posEkf, posUkf, posCkf, posPf,
        %          velEkf, velUkf, velCkf, velPf (each 1 x nSteps).
            figure('Name', 'DBT Filter Comparison', 'Color', 'w', ...
                   'Position', [100, 100, 900, 750]);

            % Trajectory
            subplot(3, 1, 1);
            plot(truthX(1, :), truthX(3, :), 'k-.', 'DisplayName', 'True'); hold on;
            plot(meas(2, :) .* cos(meas(1, :)), ...
                 meas(2, :) .* sin(meas(1, :)), 'ro', 'DisplayName', 'Measurement');
            plot(estEkf(1, :), estEkf(3, :), 'c-v', 'DisplayName', 'EKF');
            plot(estUkf(1, :), estUkf(3, :), 'g-x', 'DisplayName', 'UKF');
            plot(estCkf(1, :), estCkf(3, :), 'b-*', 'DisplayName', 'CKF');
            plot(estPf(1, :),  estPf(3, :),  'm-s', 'DisplayName', 'PF');
            xlabel('x [m]'); ylabel('y [m]');
            legend('Location', 'northwest'); grid on; grid minor; hold off;

            % Position RMSE
            subplot(3, 1, 2);
            kk = 1:nSteps;
            plot(kk, rmse.posEkf, 'c.-', 'LineWidth', 1.5, 'DisplayName', 'EKF'); hold on;
            plot(kk, rmse.posUkf, 'g.-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
            plot(kk, rmse.posCkf, 'b.-', 'LineWidth', 1.5, 'DisplayName', 'CKF');
            plot(kk, rmse.posPf,  'm.-', 'LineWidth', 1.5, 'DisplayName', 'PF');
            xlabel('Time Step'); ylabel('Position RMSE [m]');
            legend('Location', 'northwest'); grid on; grid minor; hold off;

            % Velocity RMSE
            subplot(3, 1, 3);
            plot(kk, rmse.velEkf, 'c.-', 'LineWidth', 1.5, 'DisplayName', 'EKF'); hold on;
            plot(kk, rmse.velUkf, 'g.-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
            plot(kk, rmse.velCkf, 'b.-', 'LineWidth', 1.5, 'DisplayName', 'CKF');
            plot(kk, rmse.velPf,  'm.-', 'LineWidth', 1.5, 'DisplayName', 'PF');
            xlabel('Time Step'); ylabel('Velocity RMSE [m/s]');
            legend('Location', 'northwest'); grid on; grid minor; hold off;

            sgtitle('Detect-Before-Track: EKF / UKF / CKF / PF', 'FontSize', 13);
        end

        function plotTbdResults(trueState, dpTrack, pfState, measData, ...
                                posRmse, velRmse, dpScore, cfg)
        % PLOTTBDRESULTS  Comprehensive TBD comparison figure.
            nF = cfg.numFrames;
            figure('Name', 'TBD Algorithm Comparison', ...
                   'Position', [50, 50, 1500, 950], 'Color', 'w');

            % Row 1: snapshot frames
            snapIdx = unique(round(linspace(1, nF, 5)));
            snapIdx = snapIdx(1:min(5, end));
            for i = 1:length(snapIdx)
                tf = snapIdx(i);
                subplot(4, 5, i);
                imagesc(measData(:, :, tf)); colormap(gca, 'parula'); hold on;
                plot(trueState(tf, 2), trueState(tf, 1), ...
                     'r+', 'MarkerSize', 14, 'LineWidth', 2);
                title(sprintf('Frame %d', tf), 'FontSize', 9);
                axis equal tight; hold off;
            end

            % Rows 2-3 left: trajectory overlay
            subplot(4, 5, [6 7 8 11 12 13]);
            imagesc(mean(measData, 3)); colormap(gca, 'parula'); hold on;
            hT  = plot(trueState(:, 2), trueState(:, 1), 'g-o', 'LineWidth', 2, 'MarkerSize', 3);
            hDp = plot(dpTrack(:, 2),   dpTrack(:, 1),   'r--s','LineWidth', 2, 'MarkerSize', 4);
            hPf = plot(pfState(:, 2),   pfState(:, 1),   'b:d', 'LineWidth', 2, 'MarkerSize', 4);
            plot(trueState(1, 2),   trueState(1, 1),   'gp', 'MarkerSize', 16, 'MarkerFaceColor', 'g');
            plot(trueState(end, 2), trueState(end, 1), 'gh', 'MarkerSize', 16, 'MarkerFaceColor', 'g');
            legend([hT hDp hPf], {'True', 'DP-TBD', 'PF-TBD'}, 'Location', 'northwest', 'FontSize', 9);
            title(sprintf('Trajectory (DP score = %.1f)', dpScore));
            axis equal tight; hold off;

            % Row 2 right: position RMSE
            dpPosRmse = sqrt(sum((dpTrack - trueState(:, 1:2)).^2, 2));
            subplot(4, 5, [9 10]);
            plot(1:nF, dpPosRmse, 'r-', 'LineWidth', 1.5); hold on;
            plot(1:nF, posRmse,   'b-', 'LineWidth', 1.5);
            legend('DP-TBD', 'PF-TBD', 'Location', 'best');
            title('Position RMSE'); xlabel('Frame'); ylabel('pixels');
            grid on; hold off;

            % Row 3 right: velocity RMSE
            dpVelEst  = diff(dpTrack, 1, 1) / cfg.dt;
            dpVelRmse = sqrt(sum((dpVelEst - trueState(2:end, 3:4)).^2, 2));
            subplot(4, 5, [14 15]);
            plot(2:nF, dpVelRmse, 'r-', 'LineWidth', 1.5); hold on;
            plot(1:nF, velRmse,   'b-', 'LineWidth', 1.5);
            legend('DP-TBD (diff)', 'PF-TBD', 'Location', 'best');
            title('Velocity RMSE'); xlabel('Frame'); ylabel('pixels/frame');
            grid on; hold off;

            % Row 4 left: per-axis errors
            subplot(4, 5, [16 17]);
            plot(1:nF, dpTrack(:,1) - trueState(:,1), 'r-',  'LineWidth', 1); hold on;
            plot(1:nF, dpTrack(:,2) - trueState(:,2), 'r--', 'LineWidth', 1);
            plot(1:nF, pfState(:,1) - trueState(:,1), 'b-',  'LineWidth', 1);
            plot(1:nF, pfState(:,2) - trueState(:,2), 'b--', 'LineWidth', 1);
            legend('DP row', 'DP col', 'PF row', 'PF col', 'Location', 'best');
            title('Per-Axis Error'); xlabel('Frame'); ylabel('pixels');
            grid on; yline(0, 'k:'); hold off;

            % Row 4 middle: amplitude (PF)
            subplot(4, 5, [18 19]);
            plot(1:nF, pfState(:, 5), 'b-', 'LineWidth', 1.5); hold on;
            plot(1:nF, trueState(:, 5), 'g--', 'LineWidth', 1.5);
            legend('PF est.', 'True', 'Location', 'best');
            title('Amplitude (PF)'); xlabel('Frame'); ylabel('amplitude');
            grid on; hold off;

            % Row 4 right: summary
            subplot(4, 5, 20); axis off;
            txt = {sprintf('--- Summary ---'), ...
                   sprintf('Frames: %d', nF), ...
                   sprintf('Grid: %dx%d', cfg.gridSize), ...
                   sprintf('SNR ~ %.1f dB', 20*log10(cfg.amplitude/cfg.noiseStd)), ...
                   '', ...
                   sprintf('DP pos RMSE: %.2f px', mean(dpPosRmse)), ...
                   sprintf('PF pos RMSE: %.2f px', mean(posRmse)), ...
                   '', ...
                   sprintf('DP vel RMSE: %.2f',    mean(dpVelRmse)), ...
                   sprintf('PF vel RMSE: %.2f',    mean(velRmse)), ...
                   '', ...
                   sprintf('DP score: %.1f', dpScore)};
            text(0.05, 0.95, txt, 'VerticalAlignment', 'top', ...
                 'FontSize', 9, 'FontName', 'FixedWidth');

            sgtitle('Track-Before-Detect: DP vs Particle Filter', 'FontSize', 14);
        end

        %% ================================================================
        %                   SHARED UTILITIES
        % =================================================================

        function F = ctDynamicMatrix(T, x)
        % CTDYNAMICMATRIX  Constant turn-rate state transition matrix.
            w = x(5);
            F = [1  sin(w*T)/w        0  -(1-cos(w*T))/w  0; ...
                 0  cos(w*T)           0  -sin(w*T)        0; ...
                 0  (1-cos(w*T))/w     1   sin(w*T)/w      0; ...
                 0  sin(w*T)           0   cos(w*T)        0; ...
                 0  0                  0   0               1];
        end

        function z = ctMeasFunc(x)
        % CTMEASFUNC  Nonlinear range/azimuth measurement function.
            z = [atan2(x(3), x(1)); norm(x([1 3]))];
        end

        function H = ctMeasJacobian(x)
        % CTMEASJACOBIAN  Jacobian of range/azimuth measurement model.
            px = x(1);  py = x(3);
            r2 = px^2 + py^2;
            r  = sqrt(r2);
            H  = [-py/r2  0  px/r2  0  0; ...
                   px/r   0  py/r   0  0];
        end

        function L = cholPsd(A)
        % CHOLPSD  Robust Cholesky factorisation (SVD fallback).
            [~, flag] = chol(A);
            if flag == 0
                L = chol(A, 'lower');
            else
                [~, S, V] = svd(A);
                L = V * sqrt(S);
            end
        end

        function [wSp, spPts] = generateSigmaPoints(mu, P, cfg)
        % GENERATESIGMAPOINTS  Van der Merwe sigma points for UKF.
            n = cfg.stateDim;
            alpha = cfg.ukfAlpha;  beta = cfg.ukfBeta;  kappa = cfg.ukfKappa;
            lambda = alpha^2 * (n + kappa) - n;
            sqrtP  = TargetTracker.cholPsd(P * (n + lambda));

            spPts = zeros(n, 2*n + 1);
            spPts(:, 1) = mu;
            for i = 1:n
                spPts(:, i+1)   = mu + sqrtP(:, i);
                spPts(:, i+n+1) = mu - sqrtP(:, i);
            end

            wm = [lambda / (n + lambda), repmat(1 / (2*(n+lambda)), 1, 2*n)];
            wc = wm;
            wc(1) = wc(1) + (1 - alpha^2 + beta);
            wSp = [wm; wc];
        end

        function [wCp, cpPts] = generateCubaturePoints(mu, P, cfg)
        % GENERATECUBATUREPOINTS  Third-degree cubature points for CKF.
            n   = cfg.stateDim;
            nCp = 2 * n;
            xi  = sqrt(n) * [eye(n), -eye(n)];
            sqP = TargetTracker.cholPsd(P);

            cpPts = zeros(n, nCp);
            for i = 1:nCp
                cpPts(:, i) = mu + sqP * xi(:, i);
            end
            wCp = ones(1, nCp) / nCp;
        end

        function idx = systematicResample(weights, nPart)
        % SYSTEMATICRESAMPLE  Low-variance systematic resampling.
        %   Shared by both DBT-PF and TBD-PF.
            if nargin < 2
                nPart = length(weights);
            end
            cumW = cumsum(weights(:));
            idx  = zeros(nPart, 1);
            u1   = rand / nPart;
            j    = 1;
            for i = 1:nPart
                u = u1 + (i - 1) / nPart;
                while cumW(j) < u
                    j = j + 1;
                end
                idx(i) = j;
            end
        end

    end % methods (Static)
end % classdef
