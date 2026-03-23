classdef ImmUtils
% PHD.IMMUTILS  Interacting Multiple Model (IMM) Interaction Utilities
%   Provides input interaction, output interaction, and model probability
%   update for the IMM algorithm. Used for multi-model fusion in PHD filters.
%
%   Methods:
%       inputInteraction   - IMM input interaction
%       outputInteraction  - IMM output interaction
%       updateProbability  - IMM model probability update
%
%   Reference:
%       Bar-Shalom, Y., et al. "Estimation with Applications to
%       Tracking and Navigation," Wiley, 2001.
%
%   See also: phd.SimmUtils, phd.MultiModelFilter

    methods (Static)

        function [w0, m0, P0, cNorm] = inputInteraction(k, model, est, ...
                w, m, P, L, miu)
        % INPUTINTERACTION  IMM input interaction.
        %   Computes mixing probabilities based on transition matrix and model
        %   probabilities, then performs weighted mixing of previous states.
        %
        %   Inputs:
        %       k     - Current time step
        %       model - Model parameters (with Pmn transition matrix)
        %       est   - Estimation structure
        %       w,m,P,L - Current weights, means, covariances, component counts
        %       miu   - Model probability vector [M x 1]
        %
        %   Outputs:
        %       w0,m0,P0 - Cell arrays of mixed initial parameters per model
        %       cNorm    - Normalization constants

            M = model.M;
            mu = zeros(M, M);
            cNorm = zeros(M, 1);

            for j = 1:M
                cNorm(j) = sum(model.Pmn(:, j) .* miu);
            end

            for i = 1:M
                for j = 1:M
                    mu(i, j) = model.Pmn(i, j) * miu(i) / cNorm(j);
                end
            end

            if k == 1
                wM = {w, w, w};
                mM = {m, m, m};
                pM = {P, P, P};
            else
                wM = {est.W{k-1, 1}, est.W{k-1, 2}, est.W{k-1, 3}};
                mM = {est.X{k-1, 1}, est.X{k-1, 2}, est.X{k-1, 3}};
                pM = {est.P{k-1, 1}, est.P{k-1, 2}, est.P{k-1, 3}};
            end

            w0 = cell(M, 1);
            m0 = cell(M, 1);
            P0 = cell(M, 1);
            for j = 1:M
                w0{j} = wM{1} .* mu(1, j) + wM{2} .* mu(2, j) + wM{3} .* mu(3, j);
                m0{j} = mM{1} .* mu(1, j) + mM{2} .* mu(2, j) + mM{3} .* mu(3, j);
            end
            for j = 1:M
                P0{j} = mu(1, j) * (pM{1} + (mM{1} - m0{j}) * (mM{1} - m0{j})') ...
                       + mu(2, j) * (pM{2} + (mM{2} - m0{j}) * (mM{2} - m0{j})') ...
                       + mu(3, j) * (pM{3} + (mM{3} - m0{j}) * (mM{3} - m0{j})');
            end
        end

        function [wOut, mOut, POut, LOut] = outputInteraction(k, model, est, miu)
        % OUTPUTINTERACTION  IMM output interaction.
        %   Performs weighted fusion of filter results from all models.
        %
        %   Inputs:
        %       k     - Current time step
        %       model - Model parameters
        %       est   - Estimation structure
        %       miu   - Model probability vector
        %
        %   Outputs:
        %       wOut, mOut, POut - Fused Gaussian mixture parameters
        %       LOut             - Component count after fusion

            wAll = est.W;
            mAll = est.X;
            pAll = est.P;
            nGMs = cell2mat(est.L(k, 1));

            wOut = [];  mOut = [];  POut = [];

            for ll = 1:nGMs
                wJ = wAll{k, 1}(ll) * miu(1) + wAll{k, 2}(ll) * miu(2) + wAll{k, 3}(ll) * miu(3);
                mJ = mAll{k, 1}(:, ll) * miu(1) + mAll{k, 2}(:, ll) * miu(2) + mAll{k, 3}(:, ll) * miu(3);
                PJ = miu(1) * (pAll{k, 1}(:, :, ll) + (mAll{k, 1}(:, ll) - mJ) * (mAll{k, 1}(:, ll) - mJ)') ...
                   + miu(2) * (pAll{k, 2}(:, :, ll) + (mAll{k, 2}(:, ll) - mJ) * (mAll{k, 2}(:, ll) - mJ)') ...
                   + miu(3) * (pAll{k, 3}(:, :, ll) + (mAll{k, 3}(:, ll) - mJ) * (mAll{k, 3}(:, ll) - mJ)');

                wOut = [wOut; wJ]; %#ok<AGROW>
                mOut = [mOut, mJ]; %#ok<AGROW>
                POut = cat(3, POut, PJ);
            end
            LOut = length(wOut);
        end

        function miu = updateProbability(k, model, est, cNorm, miu)
        % UPDATEPROBABILITY  IMM model probability update.
        %   Updates model probabilities based on likelihoods and normalization constants.
        %
        %   Inputs:
        %       k     - Current time step
        %       model - Model parameters
        %       est   - Estimation structure (with likelihoods Lk)
        %       cNorm - Normalization constants
        %       miu   - Current model probabilities
        %
        %   Outputs:
        %       miu   - Updated model probabilities

            EPS_FLOOR = 1e-20;
            M = model.M;

            if isempty(est.Lk{k, 1}) || isempty(est.Lk{k, 2}) || isempty(est.Lk{k, 3})
                return;
            end

            lkVec = [est.Lk{k, 1}, est.Lk{k, 2}, est.Lk{k, 3}];
            if all(lkVec == 0)
                lkMin = EPS_FLOOR;
            else
                lkMin = min(lkVec(lkVec ~= 0));
            end

            lkLast = zeros(1, M);
            for ii = 1:M
                lkLast(ii) = est.Lk{k, ii};
                if lkLast(ii) == 0
                    lkLast(ii) = lkMin * EPS_FLOOR;
                end
            end

            cTotal = lkLast * cNorm;
            for ii = 1:M
                miu(ii, :) = lkLast(ii) * cNorm(ii) / cTotal;
            end
            miu = miu / sum(miu);
        end

    end
end
