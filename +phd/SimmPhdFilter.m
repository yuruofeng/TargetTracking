classdef SimmPhdFilter < handle
% PHD.SIMMPHDFILTER  SIMM-PHD Filter Class
%   Implements the Set-based Interacting Multiple Model Probability
%   Hypothesis Density (SIMM-PHD) filter.
%
%   SIMM-PHD vs IMM-PHD Differences:
%     - Input interaction uses max instead of sum for normalization
%     - Output interaction directly selects most likely model
%     - Probability update uses max for normalization
%
%   Properties:
%       config - Filter configuration parameters
%       model  - Motion model parameters
%       est    - Estimation results structure
%       miu    - Current model possibilities
%
%   Usage:
%       model = phd.Config();
%       filter = phd.SimmPhdFilter(model);
%       est = filter.run(meas);
%
%   Reference:
%       Yang, F., et al. "Multi-target tracking algorithm based on
%       set cardinality probability hypothesis density filtering,"
%       Journal of Electronics & Information Technology, 2014.
%
%   See also: phd.ImmPhdFilter, phd.SimmUtils, phd.Config

    properties
        config    % Filter configuration
        model     % Motion model
        est       % Estimation results
        miu       % Model possibilities
    end

    methods

        function obj = SimmPhdFilter(model, useAdaptive)
        % SIMMPHDFILTER  Create SIMM-PHD filter.
        %
        %   Inputs:
        %       model       - phd.Config configuration object
        %       useAdaptive - Use adaptive GM management (default: true)

            if nargin < 1
                model = phd.Config();
            end
            obj.model = model;
            obj.config.L_max = 100;
            obj.config.elim_threshold = 1e-4;
            obj.config.merge_threshold = 10;
            obj.config.P_G = 0.999;
            obj.config.gamma = chi2inv(obj.config.P_G, model.z_dim);
            obj.config.gate_flag = 1;
            
            if nargin < 2
                useAdaptive = true;
            end
            obj.config.useAdaptive = useAdaptive;
            obj.config.targetRatio = 10;
        end

        function est = run(obj, meas, truth)
        % RUN  Execute SIMM-PHD filtering.
        %
        %   Inputs:
        %       meas  - Measurement structure (with Z, K)
        %       truth - Truth structure (optional, for error computation)
        %
        %   Outputs:
        %       est - Estimation results structure

            K = meas.K;

            obj.est.W  = cell(K, 3);
            obj.est.X  = cell(K, 3);
            obj.est.P  = cell(K, 3);
            obj.est.N  = zeros(K, 3);
            obj.est.L  = cell(K, 3);
            obj.est.Lk = cell(K, 3);

            obj.est.IMMW = cell(K, 1);
            obj.est.IMMX = cell(K, 1);
            obj.est.IMMP = cell(K, 1);
            obj.est.IMMN = zeros(K, 1);
            obj.est.IMML = cell(K, 1);
            obj.est.filter = obj.config;

            wUpdate = eps;
            mUpdate = [0.1; 0; 0; 0.1; 0; 0];
            PUpdate = diag(ones(6, 1)).^2;
            LUpdate = 1;

            obj.miu = [1; 1; 1];  % SIMM uses possibilities (not probabilities)
            obj.est.miu = [];

            for k = 1:K
                [w0, m0, P0, cNorm] = phd.SimmUtils.inputInteraction(...
                    k, obj.model, obj.est, wUpdate, mUpdate, PUpdate, LUpdate, obj.miu);
                nGMs = length(w0{1});
                L0 = {nGMs; nGMs; nGMs};

                [w1,m1,P1,L1,lk1, w2,m2,P2,L2,lk2, w3,m3,P3,L3,lk3] = ...
                    phd.MultiModelFilter.run(k, obj.model, meas, obj.config, w0, m0, P0, L0);

                obj.est.W{k,1} = w1;  obj.est.W{k,2} = w2;  obj.est.W{k,3} = w3;
                obj.est.X{k,1} = m1;  obj.est.X{k,2} = m2;  obj.est.X{k,3} = m3;
                obj.est.P{k,1} = P1;  obj.est.P{k,2} = P2;  obj.est.P{k,3} = P3;
                obj.est.L{k,1} = L1;  obj.est.L{k,2} = L2;  obj.est.L{k,3} = L3;
                obj.est.Lk{k,1} = lk1;  obj.est.Lk{k,2} = lk2;  obj.est.Lk{k,3} = lk3;

                obj.est.miu = [obj.est.miu, obj.miu];
                obj.miu = phd.SimmUtils.updateProbability(k, obj.model, obj.est, cNorm, obj.miu);

                [wUpdate, mUpdate, PUpdate, LUpdate] = ...
                    phd.SimmUtils.outputInteraction(k, obj.model, obj.est, obj.miu);

                if obj.config.useAdaptive
                    gmConfig.baseElimThresh = obj.config.elim_threshold;
                    gmConfig.baseMergeThresh = obj.config.merge_threshold;
                    gmConfig.maxNum = obj.config.L_max;
                    gmConfig.targetRatio = obj.config.targetRatio;
                    [wUpdate, mUpdate, PUpdate] = phd.GaussMixture.adaptiveManage(...
                        wUpdate, mUpdate, PUpdate, gmConfig);
                else
                    [wUpdate, mUpdate, PUpdate] = phd.GaussMixture.manage(wUpdate, mUpdate, PUpdate, ...
                        obj.config.elim_threshold, obj.config.merge_threshold, obj.config.L_max);
                end
                LUpdate = length(wUpdate);

                idx = find(wUpdate > 0.5);
                for j = 1:length(idx)
                    nRepeat = round(wUpdate(idx(j)));
                    obj.est.IMMW{k} = [obj.est.IMMW{k}, repmat(wUpdate(idx(j)), [1, nRepeat])];
                    obj.est.IMMX{k} = [obj.est.IMMX{k}, repmat(mUpdate(:, idx(j)), [1, nRepeat])];
                    obj.est.IMMN(k) = obj.est.IMMN(k) + nRepeat;
                end
            end

            if nargin > 1 && ~isempty(truth)
                obj.computeError(truth);
            end

            est = obj.est;
        end

        function computeError(obj, truth)
        % COMPUTEERROR  Compute estimation error.

            K = length(obj.est.IMMX);
            for k = 1:K
                if ~isempty(obj.est.IMMX{k}) && ~isempty(truth.X{k})
                    obj.est.error_X{k} = obj.est.IMMX{k} - truth.X{k};
                else
                    if isempty(obj.est.IMMX{k})
                        obj.est.IMMX{k} = zeros(6, 1);
                    end
                    if ~isempty(truth.X{k})
                        obj.est.error_X{k} = obj.est.IMMX{k} - truth.X{k};
                    else
                        obj.est.error_X{k} = [];
                    end
                    if isempty(obj.est.IMMX{k}) || size(obj.est.IMMX{k}, 2) == 1
                        obj.est.IMMX{k} = [];
                    end
                end
            end
        end

    end
end
