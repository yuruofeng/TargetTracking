classdef CtFilter
% PHD.CTFILTER  Coordinated Turn (CT) Model PHD Filter
%   Provides EKF-based prediction, update, likelihood computation, and gating
%   for the CT motion model, including nonlinear state transition and observation.
%
%   State vector: [x, vx, y, vy, omega] (5 dimensions)
%       x, y   - Position coordinates
%       vx, vy - Velocity components
%       omega  - Turn rate (rad/s)
%
%   Methods:
%       predict          - EKF prediction for CT model
%       update           - EKF update (batch processing)
%       computeLikelihood - Compute weighted likelihood for CT model
%       gateMeas         - Ellipsoidal gating for CT model
%       genNewState      - Nonlinear state transition
%       genObservation   - Observation function
%       predictMat       - EKF prediction Jacobian
%       updateMat        - EKF update Jacobian
%
%   Reference:
%       Bar-Shalom, Y., et al. "Estimation with Applications to
%       Tracking and Navigation," Wiley, 2001.
%
%   See also: phd.CvFilter, phd.CaFilter, phd.MultiModelFilter

    methods (Static)

        function [mPred, PPred] = predict(model, m, P)
        % PREDICT  EKF prediction for CT model.
        %   Converts 6D unified state to 5D (including omega) for prediction.
        %
        %   Inputs:
        %       model - Model parameter struct
        %       m     - 6D mean matrix [6 x nComps]
        %       P     - 6D covariance stack [6 x 6 x nComps]
        %
        %   Outputs:
        %       mPred - 5D predicted mean [5 x nComps]
        %       PPred - 5D predicted covariance [5 x 5 x nComps]

            if isempty(m)
                mPred = [];
                PPred = zeros(5, 5, 0);
                return;
            end

            nComps = size(m, 2);

            rows4 = m([1, 2, 4, 5], :);
            omegaRow = model.omega * ones(1, nComps);
            m5 = [rows4; omegaRow];

            [~, P4] = phd.MathUtils.reduceState6to4(m, P);
            P5 = zeros(5, 5, nComps);
            for ii = 1:nComps
                P5(:, :, ii) = blkdiag(P4(:, :, ii), 1);
            end

            mPred = zeros(size(m5));
            PPred = zeros(size(P5));
            for ii = 1:nComps
                mT = phd.CtFilter.genNewState(model, m5(:, ii), 'noiseless');
                [Fekf, Gekf] = phd.CtFilter.predictMat(model, m5(:, ii));
                PT = Gekf * model.Q_CT * Gekf' + Fekf * P5(:, :, ii) * Fekf';
                PT = (PT + PT') / 2;
                mPred(:, ii) = mT;
                PPred(:, :, ii) = PT;
            end
        end

        function [qzUpdate, mUpdate, PUpdate] = update(z, model, m, P)
        % UPDATE  EKF update for CT model (batch processing).
        %   Updates in 4D space (excluding omega), then appends omega.
        %
        %   Inputs:
        %       z     - Measurement matrix [2 x nMeas]
        %       model - Model parameters
        %       m     - 5D predicted mean [5 x nComps]
        %       P     - 5D predicted covariance [5 x 5 x nComps]
        %
        %   Outputs:
        %       qzUpdate - Measurement likelihood [nComps x nMeas]
        %       mUpdate  - 5D updated mean [5 x nComps x nMeas]
        %       PUpdate  - 5D updated covariance [5 x 5 x nComps]

            if isempty(m) || isempty(z)
                qzUpdate = [];
                mUpdate = [];
                PUpdate = zeros(5, 5, 0);
                return;
            end

            nComps = size(m, 2);
            nMeas = size(z, 2);

            qzUpdate = zeros(nComps, nMeas);
            mUpdate = zeros(model.x_dimCT, nComps, nMeas);
            PUpdate = zeros(model.x_dimCT, model.x_dimCT, nComps);

            Hno = [1 0 0 0; 0 0 1 0];

            for ii = 1:nComps
                mNo = m([1, 2, 3, 4], ii);
                PNo = P(1:4, 1:4, ii);
                muNo = Hno * mNo;

                S = model.R_CT + Hno * PNo * Hno';
                S = (S + S') / 2;
                S = phd.MathUtils.ensurePositiveDefinite(S, 1e-10);

                Vs = chol(S);
                detS = prod(diag(Vs))^2;
                iS = inv(Vs' * Vs); %#ok<MINV>
                K = PNo * Hno' * iS;

                nu = z - repmat(muNo, [1, nMeas]);
                qzUpdate(ii, :) = exp(-0.5 * size(z, 1) * log(2*pi) ...
                    - 0.5 * log(detS) - 0.5 * dot(nu, iS * nu))';
                mTno = repmat(mNo, [1, nMeas]) + K * nu;

                mT = [mTno; repmat(model.omega, 1, nMeas)];
                mUpdate(:, ii, :) = mT;

                Rfull = [25 0 0; 0 25 0; 0 0 1];
                Sfull = Rfull + model.H_CT * P(:, :, ii) * model.H_CT';
                Sfull = (Sfull + Sfull') / 2;
                Sfull = phd.MathUtils.ensurePositiveDefinite(Sfull, 1e-10);
                iFull = inv(chol(Sfull)' * chol(Sfull)); %#ok<MINV>
                Kfull = P(:, :, ii) * model.H_CT' * iFull;
                PUpdate(:, :, ii) = (eye(5) - Kfull * model.H_CT) * P(:, :, ii);
            end
        end

        function lk = computeLikelihood(model, Z, mPred, PPred, wPred)
        % COMPUTELIKELIHOOD  Compute weighted likelihood for CT model.
        %   Computes likelihood in 4D space (excluding omega).

            if isempty(mPred) || isempty(Z)
                lk = 0;
                return;
            end

            Hno = [1 0 0 0; 0 0 1 0];
            mNo = mPred([1, 2, 3, 4], :);
            PNo = PPred(1:4, 1:4, :);

            nGM = size(mNo, 2);
            nMeas = size(Z, 2);
            totalLik = 0;

            for ll = 1:nGM
                Hm = Hno * mNo(:, ll);
                S = Hno * PNo(:, :, ll) * Hno' + model.R_CT;
                S = (S + S') / 2;
                S = phd.MathUtils.ensurePositiveDefinite(S, 1e-6);

                Vs = chol(S);
                detS = prod(diag(Vs))^2;
                invS = inv(S); %#ok<MINV>

                likVals = zeros(1, nMeas);
                for zz = 1:nMeas
                    v = Z(:, zz) - Hm;
                    logLik = -0.5 * (model.z_dim * log(2*pi) + log(detS) + v' * invS * v);
                    likVals(zz) = exp(logLik);
                end
                totalLik = totalLik + wPred(ll) * sum(likVals);
            end
            lk = totalLik;
        end

        function zGate = gateMeas(z, gamma, model, m, P)
        % GATEMEAS  Ellipsoidal gating for CT model (using EKF observation Jacobian).

            nMeas = size(z, 2);
            if nMeas == 0
                zGate = [];
                return;
            end

            nComps = size(m, 2);
            validIdx = [];
            for jj = 1:nComps
                [Hekf, Uekf] = phd.CtFilter.updateMat(model, m(:, jj));
                Sj = Uekf * model.R_CT * Uekf' + Hekf * P(:, :, jj) * Hekf';
                Sj = (Sj + Sj') / 2;
                Sj = phd.MathUtils.ensurePositiveDefinite(Sj, 1e-10);

                Vs = chol(Sj);
                invSqrtSj = inv(Vs); %#ok<MINV>
                zPred = phd.CtFilter.genObservation(model, m(:, jj), zeros(size(model.D_CT, 2), 1));
                nu = z - repmat(zPred, [1, nMeas]);
                dist = sum((invSqrtSj' * nu).^2);
                validIdx = union(validIdx, find(dist < gamma));
            end
            zGate = z(:, validIdx);
        end

        function X = genNewState(model, Xd, V)
        % GENNEWSTATE  Nonlinear state transition for CT model.
        %
        %   Inputs:
        %       Xd - 5D state [x vx y vy omega]'
        %       V  - Noise vector or 'noise'/'noiseless' string
        %
        %   Outputs:
        %       X  - Transitioned 5D state

            if ~isnumeric(V)
                if strcmp(V, 'noise')
                    V = model.B_CT * randn(size(model.B_CT, 2), size(Xd, 2));
                elseif strcmp(V, 'noiseless')
                    V = zeros(size(model.B_CT, 1), size(Xd, 2));
                end
            end

            if isempty(Xd)
                X = [];
                return;
            end

            X = zeros(size(Xd));
            L = size(Xd, 2);
            T = model.T;
            omega = Xd(5, :);
            tol = 1e-10;

            sinOT = sin(omega * T);
            cosOT = cos(omega * T);
            a = T * ones(1, L);
            b = zeros(1, L);
            idx = find(abs(omega) > tol);
            a(idx) = sinOT(idx) ./ omega(idx);
            b(idx) = (1 - cosOT(idx)) ./ omega(idx);

            X(1, :) = Xd(1, :) + a .* Xd(2, :) - b .* Xd(4, :);
            X(2, :) = cosOT .* Xd(2, :) - sinOT .* Xd(4, :);
            X(3, :) = b .* Xd(2, :) + Xd(3, :) + a .* Xd(4, :);
            X(4, :) = sinOT .* Xd(2, :) + cosOT .* Xd(4, :);
            X(5, :) = Xd(5, :);
            X = X + model.G_CT * V;
        end

        function Z = genObservation(model, X, W)
        % GENOBSERVATION  Observation function for CT model.
        %
        %   Inputs:
        %       X - 5D state
        %       W - Noise vector or 'noise'/'noiseless'
        %
        %   Outputs:
        %       Z - Observation vector

            if ~isnumeric(W)
                if strcmp(W, 'noise')
                    W = model.D_CT * randn(size(model.D_CT, 2), size(X, 2));
                elseif strcmp(W, 'noiseless')
                    W = zeros(size(model.D_CT, 1), size(X, 2));
                end
            end

            if isempty(X)
                Z = [];
            else
                Zfull = model.H_CT * X;
                Z = Zfull([1, 2], :) + W;
            end
        end

        function [F, G] = predictMat(model, muOld)
        % PREDICTMAT  EKF prediction Jacobian for CT model.

            tol = 1e-6;
            omega = muOld(5);
            T = model.T;
            sinOT = sin(omega * T);
            cosOT = cos(omega * T);

            a = T; b = 0;
            if abs(omega) > tol
                a = sinOT / omega;
                b = (1 - cosOT) / omega;
            end
            A = [1 a 0 -b; 0 cosOT 0 -sinOT; 0 b 1 a; 0 sinOT 0 cosOT];

            c = 0; d = T^2 / 2;
            if abs(omega) > tol
                c = (omega * T * cosOT - sinOT) / omega^2;
                d = (omega * T * sinOT - 1 + cosOT) / omega^2;
            end
            dA = [0 c 0 -d; 0 -T * sinOT 0 -T * cosOT; ...
                  0 d 0 c;  0 T * cosOT 0 -T * sinOT];

            F = [A, dA * muOld(1:4); zeros(1, 4), 1];
            G = model.G_CT;
        end

        function [H, U] = updateMat(model, mu)
        % UPDATEMAT  EKF update Jacobian for CT model.

            p = mu([1, 3], :);
            mag = p(1)^2 + p(2)^2;
            sqrtMag = sqrt(mag);
            H = [p(2) / mag,      0, -p(1) / mag,     0, 0; ...
                 p(1) / sqrtMag,  0,  p(2) / sqrtMag, 0, 0];
            U = eye(2);
        end

    end
end
