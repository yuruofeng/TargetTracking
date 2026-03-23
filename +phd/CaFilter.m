classdef CaFilter
% PHD.CAFILTER  Constant Acceleration (CA) Model PHD Filter
%   Provides EKF-based prediction, update, likelihood computation, and gating
%   for the CA motion model.
%
%   State vector: [x, vx, ax, y, vy, ay] (6 dimensions)
%       x, y   - Position coordinates
%       vx, vy - Velocity components
%       ax, ay - Acceleration components
%
%   Methods:
%       predict          - EKF prediction for CA model
%       update           - EKF update (batch processing)
%       computeLikelihood - Compute weighted likelihood for CA model
%       gateMeas         - Ellipsoidal gating for CA model
%
%   Reference:
%       Bar-Shalom, Y., et al. "Estimation with Applications to
%       Tracking and Navigation," Wiley, 2001.
%
%   See also: phd.CvFilter, phd.CtFilter, phd.MultiModelFilter

    methods (Static)

        function [mPred, PPred] = predict(model, m, P)
        % PREDICT  EKF prediction for CA model.
        %   Performs linear prediction directly in 6D state space.
        %
        %   Inputs:
        %       model - Model parameter struct
        %       m     - 6D mean matrix [6 x nComps]
        %       P     - 6D covariance stack [6 x 6 x nComps]
        %
        %   Outputs:
        %       mPred - 6D predicted mean [6 x nComps]
        %       PPred - 6D predicted covariance [6 x 6 x nComps]

            if isempty(m)
                mPred = [];
                PPred = zeros(6, 6, 0);
                return;
            end

            nComps = size(m, 2);
            mPred = zeros(size(m));
            PPred = zeros(size(P));
            for ii = 1:nComps
                mPred(:, ii) = model.F_CA * m(:, ii);
                PPred(:, :, ii) = model.G_CA * model.Q_CA * model.G_CA' ...
                              + model.F_CA * P(:, :, ii) * model.F_CA';
                PPred(:, :, ii) = (PPred(:, :, ii) + PPred(:, :, ii)') / 2;
            end
        end

        function [qzUpdate, mUpdate, PUpdate] = update(z, model, m, P)
        % UPDATE  EKF update for CA model (batch processing).
        %
        %   Inputs:
        %       z     - Measurement matrix [2 x nMeas]
        %       model - Model parameters
        %       m     - Predicted mean [6 x nComps]
        %       P     - Predicted covariance [6 x 6 x nComps]
        %
        %   Outputs:
        %       qzUpdate - Measurement likelihood [nComps x nMeas]
        %       mUpdate  - Updated mean [6 x nComps x nMeas]
        %       PUpdate  - Updated covariance [6 x 6 x nComps]

            if isempty(m) || isempty(z)
                qzUpdate = [];
                mUpdate = [];
                PUpdate = zeros(6, 6, 0);
                return;
            end

            nComps = size(m, 2);
            nMeas = size(z, 2);

            qzUpdate = zeros(nComps, nMeas);
            mUpdate = zeros(model.x_dimCA, nComps, nMeas);
            PUpdate = zeros(model.x_dimCA, model.x_dimCA, nComps);

            for ii = 1:nComps
                mu = model.H_CA * m(:, ii);
                S = model.R_CA + model.H_CA * P(:, :, ii) * model.H_CA';
                S = (S + S') / 2;
                S = phd.MathUtils.ensurePositiveDefinite(S, 1e-6);

                Vs = chol(S);
                detS = prod(diag(Vs))^2;
                iS = inv(Vs' * Vs); %#ok<MINV>
                K = P(:, :, ii) * model.H_CA' * iS;

                nu = z - repmat(mu, [1, nMeas]);
                qzUpdate(ii, :) = exp(-0.5 * size(z, 1) * log(2*pi) ...
                    - 0.5 * log(detS) - 0.5 * dot(nu, iS * nu))';
                mUpdate(:, ii, :) = repmat(m(:, ii), [1, nMeas]) + K * nu;
                PUpdate(:, :, ii) = (eye(size(P, 1)) - K * model.H_CA) * P(:, :, ii);
            end
        end

        function lk = computeLikelihood(model, Z, mPred, PPred, wPred)
        % COMPUTELIKELIHOOD  Compute weighted likelihood for CA model.

            if isempty(mPred) || isempty(Z)
                lk = 0;
                return;
            end

            nGM = size(mPred, 2);
            nMeas = size(Z, 2);
            totalLik = 0;

            for ll = 1:nGM
                Hm = model.H_CA * mPred(:, ll);
                S = model.H_CA * PPred(:, :, ll) * model.H_CA' + model.R_CA;
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
        % GATEMEAS  Ellipsoidal gating for CA model.

            nMeas = size(z, 2);
            if nMeas == 0
                zGate = [];
                return;
            end

            nComps = size(m, 2);
            validIdx = [];
            for jj = 1:nComps
                Sj = model.R_CA + model.H_CA * P(:, :, jj) * model.H_CA';
                Sj = (Sj + Sj') / 2;
                Sj = phd.MathUtils.ensurePositiveDefinite(Sj, 1e-6);

                Vs = chol(Sj);
                invSqrtSj = inv(Vs); %#ok<MINV>
                nu = z - model.H_CA * repmat(m(:, jj), [1, nMeas]);
                dist = sum((invSqrtSj' * nu).^2);
                validIdx = union(validIdx, find(dist < gamma));
            end
            zGate = z(:, validIdx);
        end

    end
end
