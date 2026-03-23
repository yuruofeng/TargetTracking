classdef MultiModelFilter
% PHD.MULTIMODELFILTER  Multi-Model Joint PHD Filter
%   Coordinates prediction, gating, update, and likelihood computation across
%   CV/CT/CA models, expanding outputs to unified 6D state.
%
%   Methods:
%       run - Execute joint filtering for all three models at time k
%
%   Reference:
%       Vo, B.-N., & Ma, W.-K. (2006). The Gaussian mixture
%       probability hypothesis density filter. IEEE TSP.
%
%   See also: phd.CvFilter, phd.CtFilter, phd.CaFilter, phd.GaussMixture

    methods (Static)

        function [w1,m1,P1,L1,lk1, w2,m2,P2,L2,lk2, w3,m3,P3,L3,lk3] = ...
                run(k, model, meas, filter, w0, m0, P0, L0)
        % RUN  Execute joint filtering for all three models at time k.
        %
        %   Inputs:
        %       k      - Current time step
        %       model  - Model parameters
        %       meas   - Measurement data
        %       filter - Filter parameters
        %       w0,m0,P0,L0 - Interacted initial Gaussian mixture (cell arrays)
        %
        %   Outputs:
        %       w1~w3   - Updated weights (per model)
        %       m1~m3   - Updated means (unified 6D)
        %       P1~P3   - Updated covariances (unified 6x6)
        %       L1~L3   - Gaussian component counts
        %       lk1~lk3 - Model likelihoods

            %% ===== Prediction =====
            % CV
            [mPred1, PPred1] = phd.CvFilter.predict(model, m0{1}, P0{1});
            wPred1 = model.P_S * w0{1};
            wPred1 = cat(1, model.w_birth, wPred1);
            mPred1 = cat(2, model.m_birthCV, mPred1);
            PPred1 = cat(3, model.P_birthCV, PPred1);

            % CT
            [mPred2, PPred2] = phd.CtFilter.predict(model, m0{2}, P0{2});
            wPred2 = model.P_S * w0{2};
            wPred2 = cat(1, model.w_birth, wPred2);
            mPred2 = cat(2, model.m_birthCT, mPred2);
            PPred2 = cat(3, model.P_birthCT, PPred2);

            % CA
            [mPred3, PPred3] = phd.CaFilter.predict(model, m0{3}, P0{3});
            wPred3 = model.P_S * w0{3};
            wPred3 = cat(1, model.w_birth, wPred3);
            mPred3 = cat(2, model.m_birthCA, mPred3);
            PPred3 = cat(3, model.P_birthCA, PPred3);

            %% ===== Gating =====
            if filter.gate_flag
                Z1 = phd.CvFilter.gateMeas(meas.Z{k}, filter.gamma, model, mPred1, PPred1);
                Z2 = phd.CtFilter.gateMeas(meas.Z{k}, filter.gamma, model, mPred2, PPred2);
                Z3 = phd.CaFilter.gateMeas(meas.Z{k}, filter.gamma, model, mPred3, PPred3);
            else
                Z1 = meas.Z{k}; Z2 = meas.Z{k}; Z3 = meas.Z{k};
            end

            % Compute union of pairwise intersections (preserve information)
            Z1r = round(Z1, 5);
            Z2r = round(Z2, 5);
            Z3r = round(Z3, 5);
            D12 = intersect(Z1r', Z2r', 'rows', 'stable');
            D13 = intersect(Z1r', Z3r', 'rows', 'stable');
            D23 = intersect(Z2r', Z3r', 'rows', 'stable');
            zIntersect = unique([D12; D13; D23], 'rows', 'stable')';

            nMeas = size(zIntersect, 2);

            %% ===== Update =====
            % Missed detection terms
            w1 = model.Q_D * wPred1;  m1raw = mPred1;  P1raw = PPred1;  L1 = length(wPred1);
            w2 = model.Q_D * wPred2;  m2raw = mPred2;  P2raw = PPred2;  L2 = length(wPred2);
            w3 = model.Q_D * wPred3;  m3raw = mPred3;  P3raw = PPred3;  L3 = length(wPred3);

            if nMeas > 0
                % CV update
                [qz1, mT1, PT1] = phd.CvFilter.update(zIntersect, model, mPred1, PPred1);
                for ell = 1:nMeas
                    wT = model.P_D * wPred1(:) .* qz1(:, ell);
                    wT = wT ./ (model.lambda_c * model.pdf_c + sum(wT));
                    w1 = cat(1, w1, wT);
                    m1raw = cat(2, m1raw, mT1(:, :, ell));
                    P1raw = cat(3, P1raw, PT1);
                end
                L1 = length(w1);

                % CT update
                [qz2, mT2, PT2] = phd.CtFilter.update(zIntersect, model, mPred2, PPred2);
                for ell = 1:nMeas
                    wT = model.P_D * wPred2(:) .* qz2(:, ell);
                    wT = wT ./ (model.lambda_c * model.pdf_c + sum(wT));
                    w2 = cat(1, w2, wT);
                    m2raw = cat(2, m2raw, mT2(:, :, ell));
                    P2raw = cat(3, P2raw, PT2);
                end
                L2 = length(w2);

                % CA update
                [qz3, mT3, PT3] = phd.CaFilter.update(zIntersect, model, mPred3, PPred3);
                for ell = 1:nMeas
                    wT = model.P_D * wPred3(:) .* qz3(:, ell);
                    wT = wT ./ (model.lambda_c * model.pdf_c + sum(wT));
                    w3 = cat(1, w3, wT);
                    m3raw = cat(2, m3raw, mT3(:, :, ell));
                    P3raw = cat(3, P3raw, PT3);
                end
                L3 = length(w3);
            end

            %% ===== Likelihood =====
            lk1 = phd.CvFilter.computeLikelihood(model, zIntersect, mPred1, PPred1, wPred1);
            lk2 = phd.CtFilter.computeLikelihood(model, zIntersect, mPred2, PPred2, wPred2);
            lk3 = phd.CaFilter.computeLikelihood(model, zIntersect, mPred3, PPred3, wPred3);

            %% ===== Gaussian Mixture Management =====
            [w1, m1raw, P1raw] = phd.GaussMixture.manage(w1, m1raw, P1raw, ...
                filter.elim_threshold, filter.merge_threshold, filter.L_max);
            [w2, m2raw, P2raw] = phd.GaussMixture.manage(w2, m2raw, P2raw, ...
                filter.elim_threshold, filter.merge_threshold, filter.L_max);
            [w3, m3raw, P3raw] = phd.GaussMixture.manage(w3, m3raw, P3raw, ...
                filter.elim_threshold, filter.merge_threshold, filter.L_max);
            L1 = length(w1);
            L2 = length(w2);
            L3 = length(w3);

            %% ===== Align Gaussian Component Counts =====
            maxGMs = max([L1, L2, L3]);
            [w1, m1raw, P1raw] = phd.MultiModelFilter.padComponents(w1, m1raw, P1raw, maxGMs);
            [w2, m2raw, P2raw] = phd.MultiModelFilter.padComponents(w2, m2raw, P2raw, maxGMs);
            [w3, m3raw, P3raw] = phd.MultiModelFilter.padComponents(w3, m3raw, P3raw, maxGMs);

            %% ===== Expand to Unified 6D =====
            % CV: 4D -> 6D
            m1 = phd.MathUtils.expandState4to6(m1raw);
            P1 = phd.MathUtils.expandCov4to6(P1raw);

            % CT: Remove omega to get 4D -> 6D
            m2no = m2raw([1, 2, 3, 4], :);
            P2no = P2raw(1:4, 1:4, :);
            m2 = phd.MathUtils.expandState4to6(m2no);
            P2 = phd.MathUtils.expandCov4to6(P2no);

            % CA: Already 6D
            m3 = m3raw;
            P3 = P3raw;
        end

    end

    methods (Static, Access = private)

        function [w, m, P] = padComponents(w, m, P, targetNum)
        % PADCOMPONENTS  Pad with zero-weight components to target count.

            nCurr = length(w);
            if nCurr < targetNum
                nAdd = targetNum - nCurr;
                w = [w; zeros(nAdd, 1)];
                m = [m, zeros(size(m, 1), nAdd)];
                P = cat(3, P, zeros(size(m, 1), size(m, 1), nAdd));
            end
        end

    end
end
