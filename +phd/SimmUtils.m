classdef SimmUtils
% PHD.SIMMUTILS  Set-based Interacting Multiple Model (SIMM) Interaction Utilities
%   Provides input interaction, output interaction, and model possibility update
%   for the SIMM algorithm. Differences from ImmUtils:
%     - Input interaction uses max instead of sum for normalization
%     - Output interaction directly selects the most likely model
%     - Probability update uses max for normalization
%
%   Methods:
%       inputInteraction   - SIMM input interaction
%       outputInteraction  - SIMM output interaction
%       updateProbability  - SIMM model possibility update
%
%   Reference:
%       Yang, F., et al. "Multi-target tracking algorithm based on
%       set cardinality probability hypothesis density filtering,"
%       Journal of Electronics & Information Technology, 2014.
%
%   See also: phd.ImmUtils, phd.MultiModelFilter

    methods (Static)

        function [w0, m0, P0, cNorm] = inputInteraction(k, model, estS, ...
                w, m, P, L, miu)
        % INPUTINTERACTION  SIMM input interaction.
        %   Uses max operator for normalization (vs. sum in IMM).
        %
        %   Inputs:
        %       k     - Current time step
        %       model - Model parameters (with Sij transition possibility matrix)
        %       estS  - Estimation structure
        %       w,m,P,L - Current weights, means, covariances, component counts
        %       miu   - Model possibility vector [M x 1]
        %
        %   Outputs:
        %       w0,m0,P0 - Cell arrays of mixed initial parameters per model
        %       cNorm    - Normalization constants

            M = model.M;
            mu = zeros(M, M);
            cNorm = zeros(M, 1);

            for j = 1:M
                cNorm(j) = max(model.Sij(:, j) .* miu);
            end

            for i = 1:M
                for j = 1:M
                    mu(i, j) = model.Sij(i, j) * miu(i) / cNorm(j);
                end
            end

            if k == 1
                wM = {w, w, w};
                mM = {m, m, m};
                pM = {P, P, P};
            else
                wM = {estS.W{k-1, 1}, estS.W{k-1, 2}, estS.W{k-1, 3}};
                mM = {estS.X{k-1, 1}, estS.X{k-1, 2}, estS.X{k-1, 3}};
                pM = {estS.P{k-1, 1}, estS.P{k-1, 2}, estS.P{k-1, 3}};
            end

            w0 = cell(M, 1);
            m0 = cell(M, 1);
            P0 = cell(M, 1);
            for j = 1:M
                [~, lMax] = max(mu(:, j));
                w0{j} = wM{lMax};
                m0{j} = mM{lMax};
            end
            for j = 1:M
                [~, lIdx] = max(mu(:, j));
                P0{j} = pM{j} + (mM{j} - m0{lIdx}) * (mM{j} - m0{lIdx})';
            end
        end

        function [wOut, mOut, POut, LOut] = outputInteraction(k, model, estS, miu)
        % OUTPUTINTERACTION  SIMM output interaction.
        %   Directly selects the most likely model (vs. weighted fusion in IMM).
        %
        %   Inputs:
        %       k     - Current time step
        %       model - Model parameters
        %       estS  - Estimation structure
        %       miu   - Model possibility vector
        %
        %   Outputs:
        %       wOut, mOut, POut - Gaussian mixture from most likely model
        %       LOut             - Component count

            [~, jMax] = max(miu);
            wOut = estS.W{k, jMax};
            mOut = estS.X{k, jMax};
            POut = estS.P{k, jMax};
            LOut = length(wOut);
        end

        function miu = updateProbability(k, model, estS, cNorm, miu)
        % UPDATEPROBABILITY  SIMM model possibility update.
        %   Uses max for normalization (vs. sum in IMM).
        %
        %   Inputs:
        %       k     - Current time step
        %       model - Model parameters
        %       estS  - Estimation structure (with likelihoods Lk)
        %       cNorm - Normalization constants
        %       miu   - Current model possibilities
        %
        %   Outputs:
        %       miu   - Updated model possibilities

            EPS_FLOOR = 1e-20;
            M = model.M;

            if isempty(estS.Lk{k, 1}) || isempty(estS.Lk{k, 2}) || isempty(estS.Lk{k, 3})
                return;
            end

            lkVec = [estS.Lk{k, 1}, estS.Lk{k, 2}, estS.Lk{k, 3}];
            if all(lkVec == 0)
                lkMin = EPS_FLOOR;
            else
                lkMin = min(lkVec(lkVec ~= 0));
            end

            lkLast = zeros(1, M);
            for ii = 1:M
                lkLast(ii) = estS.Lk{k, ii};
                if lkLast(ii) == 0
                    lkLast(ii) = lkMin * EPS_FLOOR;
                end
            end

            cMax = max(lkLast .* cNorm');
            for ii = 1:M
                miu(ii, :) = lkLast(ii) * cNorm(ii) / cMax;
            end
            miu = miu / max(miu);
        end

    end
end
