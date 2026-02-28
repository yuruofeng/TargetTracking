function varargout = demoDbt(varargin)
% DEMODBT  Detect-Before-Track algorithm demonstration.
%   Demonstrates all four DBT filters (EKF, UKF, CKF, PF) on a
%   constant turn-rate target tracking scenario with radar measurements.
%
%   Usage:
%       demoDbt()                    % Run with default settings
%       demoDbt('mcRuns', 10)        % Run 10 Monte Carlo iterations
%       demoDbt('numSteps', 200)     % Use 200 time steps
%       demoDbt('saveResults', true) % Save results to file
%
%   Parameters (name-value pairs):
%       mcRuns      - Number of Monte Carlo runs (default: 1)
%       numSteps    - Number of time steps (default: 100)
%       dt          - Sampling interval (default: 1)
%       numParticles - Number of PF particles (default: 500)
%       saveResults - Save results to .mat file (default: false)
%       saveFile    - Output filename (default: 'dbt_results.mat')
%       showPlots   - Display plots (default: true)
%
%   Outputs:
%       results - Struct containing all estimates and RMSE data
%
%   Example:
%       results = demoDbt('mcRuns', 5, 'numSteps', 150);
%
%   Algorithms:
%       EKF - Extended Kalman Filter
%       UKF - Unscented Kalman Filter
%       CKF - Cubature Kalman Filter
%       PF  - Particle Filter (SIR)
%
%   See also: dbt.Config, dbt.EKF, dbt.UKF, dbt.CKF, dbt.ParticleFilter

    close all; clc;

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('   Detect-Before-Track (DBT) Algorithm Demonstration\n');
    fprintf('============================================================\n\n');

    testFolder = fileparts(mfilename('fullpath'));
    if isempty(testFolder)
        testFolder = pwd;
    end
    addpath(genpath(fileparts(testFolder)));

    p = inputParser;
    addParameter(p, 'mcRuns', 1, @isscalar);
    addParameter(p, 'numSteps', 100, @isscalar);
    addParameter(p, 'dt', 1, @isscalar);
    addParameter(p, 'numParticles', 500, @isscalar);
    addParameter(p, 'saveResults', false, @islogical);
    addParameter(p, 'saveFile', 'dbt_results.mat', @ischar);
    addParameter(p, 'showPlots', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    cfg = dbt.Config('numSteps', opts.numSteps, 'dt', opts.dt, ...
                     'numParticles', opts.numParticles);
    cfg.display();
    fprintf('\n');

    nK = cfg.numSteps;
    nD = cfg.stateDim;

    rmseAcc = struct(...
        'posEkf', zeros(opts.mcRuns, nK), 'velEkf', zeros(opts.mcRuns, nK), ...
        'posUkf', zeros(opts.mcRuns, nK), 'velUkf', zeros(opts.mcRuns, nK), ...
        'posCkf', zeros(opts.mcRuns, nK), 'velCkf', zeros(opts.mcRuns, nK), ...
        'posPf',  zeros(opts.mcRuns, nK), 'velPf',  zeros(opts.mcRuns, nK));

    elapsed = struct('ekf', 0, 'ukf', 0, 'ckf', 0, 'pf', 0);

    for iMc = 1:opts.mcRuns
        fprintf('MC Run %d/%d ... ', iMc, opts.mcRuns);

        scenario = dbt.Scenario(cfg);
        [truthX, meas] = scenario.generate();

        x0 = cfg.initState;
        P0 = blkdiag(10*eye(4), pi/90);

        ekf = dbt.EKF(cfg);
        ukf = dbt.UKF(cfg);
        ckf = dbt.CKF(cfg);
        pf  = dbt.ParticleFilter(cfg);

        [estEkf, ~, tEkf] = ekf.run(meas, x0, P0);
        [estUkf, ~, tUkf] = ukf.run(meas, x0, P0);
        [estCkf, ~, tCkf] = ckf.run(meas, x0, P0);
        [estPf, ~, tPf]   = pf.run(meas, x0, P0);

        elapsed.ekf = elapsed.ekf + tEkf;
        elapsed.ukf = elapsed.ukf + tUkf;
        elapsed.ckf = elapsed.ckf + tCkf;
        elapsed.pf  = elapsed.pf  + tPf;

        posIdx = [1 3];  velIdx = [2 4];
        for k = 1:nK
            rmseAcc.posEkf(iMc, k) = norm(estEkf(posIdx, k) - truthX(posIdx, k));
            rmseAcc.velEkf(iMc, k) = norm(estEkf(velIdx, k) - truthX(velIdx, k));
            rmseAcc.posUkf(iMc, k) = norm(estUkf(posIdx, k) - truthX(posIdx, k));
            rmseAcc.velUkf(iMc, k) = norm(estUkf(velIdx, k) - truthX(velIdx, k));
            rmseAcc.posCkf(iMc, k) = norm(estCkf(posIdx, k) - truthX(posIdx, k));
            rmseAcc.velCkf(iMc, k) = norm(estCkf(velIdx, k) - truthX(velIdx, k));
            rmseAcc.posPf(iMc, k)  = norm(estPf(posIdx, k) - truthX(posIdx, k));
            rmseAcc.velPf(iMc, k)  = norm(estPf(velIdx, k) - truthX(velIdx, k));
        end

        fprintf('Done. Time: EKF=%.3fs, UKF=%.3fs, CKF=%.3fs, PF=%.3fs\n', ...
                tEkf, tUkf, tCkf, tPf);
    end

    rmse = struct();
    fnames = fieldnames(rmseAcc);
    for i = 1:numel(fnames)
        rmse.(fnames{i}) = mean(rmseAcc.(fnames{i}), 1);
    end

    fprintf('\n=== Results Summary ===\n');
    fprintf('Algorithm  | Mean Pos RMSE | Mean Vel RMSE | Time [s]\n');
    fprintf('-----------|---------------|---------------|---------\n');
    fprintf('EKF        | %13.3f | %13.3f | %7.3f\n', mean(rmse.posEkf), mean(rmse.velEkf), elapsed.ekf);
    fprintf('UKF        | %13.3f | %13.3f | %7.3f\n', mean(rmse.posUkf), mean(rmse.velUkf), elapsed.ukf);
    fprintf('CKF        | %13.3f | %13.3f | %7.3f\n', mean(rmse.posCkf), mean(rmse.velCkf), elapsed.ckf);
    fprintf('PF         | %13.3f | %13.3f | %7.3f\n', mean(rmse.posPf), mean(rmse.velPf), elapsed.pf);
    fprintf('\n');

    if opts.showPlots
        fprintf('Generating visualizations...\n');
        
        estimates = {estEkf, estUkf, estCkf, estPf};
        labels = {'EKF', 'UKF', 'CKF', 'PF'};
        posRmse = {rmse.posEkf, rmse.posUkf, rmse.posCkf, rmse.posPf};
        velRmse = {rmse.velEkf, rmse.velUkf, rmse.velCkf, rmse.velPf};

        rmseStruct = struct();
        rmseStruct.posRmse = posRmse;
        rmseStruct.velRmse = velRmse;

        viz.Visualizer.plotComprehensiveDbt(truthX, meas, estimates, ...
            rmseStruct, labels, nK);

        viz.Visualizer.plotTimingComparison(...
            [elapsed.ekf, elapsed.ukf, elapsed.ckf, elapsed.pf] / opts.mcRuns, ...
            labels, 'Average Computation Time per Run');
    end

    results = struct(...
        'truthX', truthX, 'meas', meas, ...
        'estEkf', estEkf, 'estUkf', estUkf, 'estCkf', estCkf, 'estPf', estPf, ...
        'rmse', rmse, 'elapsed', elapsed, 'config', cfg);

    if opts.saveResults
        save(opts.saveFile, '-struct', 'results');
        fprintf('Results saved to %s\n', opts.saveFile);
    end

    fprintf('\n=== DBT Demo Complete ===\n\n');

    if nargout > 0
        varargout{1} = results;
    end
end
