classdef CvFilter
% PHD.CVFILTER  Constant Velocity (CV) Model PHD Filter
%   Provides Kalman filter-based prediction, update, likelihood computation,
%   and gating for the CV motion model.
%
%   State vector: [x, vx, y, vy] (4 dimensions)
%       x, y   - Position coordinates
%       vx, vy - Velocity components
%
%   Methods:
%       predict          - Kalman prediction for CV model
%       update           - Kalman update (batch processing for multiple measurements)
%       computeLikelihood - Compute weighted likelihood for CV model
%       gateMeas         - Ellipsoidal gating for CV model
%
%   Reference:
%       Bar-Shalom, Y., et al. "Estimation with Applications to
%       Tracking and Navigation," Wiley, 2001.
%
%   See also: phd.CtFilter, phd.CaFilter, phd.MultiModelFilter

    methods (Static)

        function [mPred, PPred] = predict(model, m, P)
        % PREDICT  Kalman prediction for CV model.
        %   Reduces 6D unified state to 4D for prediction, output is 4D.
        %
        %   Inputs:
        %       model - Model parameter struct
        %       m     - 6D mean matrix [6 x nComps]
        %       P     - 6D covariance stack [6 x 6 x nComps]
        %
        %   Outputs:
        %       mPred - 4D predicted mean [4 x nComps]
        %       PPred - 4D predicted covariance [4 x 4 x nComps]

            if isempty(m)
                mPred = [];
                PPred = zeros(4, 4, 0);
                return;
            end

            nComps = size(m, 2);
            [m4, P4] = phd.MathUtils.reduceState6to4(m, P);

            mPred = zeros(size(m4));
            PPred = zeros(size(P4));
            for ii = 1:nComps
                mPred(:, ii) = model.F_CV * m4(:, ii);
                PPred(:, :, ii) = model.Q_CV + model.F_CV * P4(:, :, ii) * model.F_CV';
            end
        end

        function [qzUpdate, mUpdate, PUpdate] = update(z, model, m, P)
        % UPDATE  Kalman update for CV model (batch processing).
        %
        %   Inputs:
        %       z     - Measurement matrix [zDim x nMeas]
        %       model - Model parameters
        %       m     - Predicted mean [4 x nComps]
        %       P     - Predicted covariance [4 x 4 x nComps]
        %
        %   Outputs:
        %       qzUpdate - Measurement likelihood [nComps x nMeas]
        %       mUpdate  - Updated mean [4 x nComps x nMeas]
        %       PUpdate  - Updated covariance [4 x 4 x nComps]

            if isempty(m) || isempty(z)
                qzUpdate = [];
                mUpdate = [];
                PUpdate = zeros(4, 4, 0);
                return;
            end

            nComps = size(m, 2);
            nMeas = size(z, 2);

            qzUpdate = zeros(nComps, nMeas);
            mUpdate = zeros(model.x_dimCV, nComps, nMeas);
            PUpdate = zeros(model.x_dimCV, model.x_dimCV, nComps);

            for ii = 1:nComps
                [qz, mT, PT] = phd.CvFilter.kalmanUpdateSingle(...
                    z, model.H_CV, model.R_CV, m(:, ii), P(:, :, ii));
                qzUpdate(ii, :) = qz;
                mUpdate(:, ii, :) = mT;
                PUpdate(:, :, ii) = PT;
            end
        end

        function lk = computeLikelihood(model, Z, mPred, PPred, wPred)
        % COMPUTELIKELIHOOD  Compute weighted likelihood for CV model.
        %
        %   Inputs:
        %       model - Model parameters
        %       Z     - Gated measurement set
        %       mPred - Predicted mean
        %       PPred - Predicted covariance
        %       wPred - Predicted weights
        %
        %   Outputs:
        %       lk - Scalar likelihood value

            if isempty(mPred) || isempty(Z)
                lk = 0;
                return;
            end

            lk = phd.CvFilter.computeWeightedLikelihood(...
                model.H_CV, model.R_CV, model.z_dim, Z, mPred, PPred, wPred);
        end

        function zGate = gateMeas(z, gamma, model, m, P)
        % GATEMEAS  Ellipsoidal gating for CV model.
        %   Retains measurements within the gating region.
        %
        %   Inputs:
        %       z     - Measurement matrix [2 x nMeas]
        %       gamma - Gating threshold (chi2inv(P_G, zDim))
        %       model - Model parameters
        %       m     - Predicted mean [4 x nComps]
        %       P     - Predicted covariance [4 x 4 x nComps]
        %
        %   Outputs:
        %       zGate - Gated measurement matrix

            if isempty(m)
                zGate = z;
                return;
            end

            zGate = phd.CvFilter.ellipsoidalGate(...
                z, gamma, model.H_CV, model.R_CV, m, P);
        end

    end

    methods (Static, Access = private)

        function [qz, mT, PT] = kalmanUpdateSingle(z, H, R, m, P)
        % KALMANUPDATESINGLE  Kalman update for single Gaussian component.

            mu = H * m;
            S = R + H * P * H';
            S = (S + S') / 2;
            S = phd.MathUtils.ensurePositiveDefinite(S, 1e-6);

            Vs = chol(S);
            detS = prod(diag(Vs))^2;
            iS = inv(Vs' * Vs); %#ok<MINV>
            K = P * H' * iS;

            nu = z - repmat(mu, [1, size(z, 2)]);
            qz = exp(-0.5 * size(z, 1) * log(2*pi) - 0.5 * log(detS) ...
                  - 0.5 * dot(nu, iS * nu))';
            mT = repmat(m, [1, size(z, 2)]) + K * nu;
            PT = (eye(size(P)) - K * H) * P;
        end

        function lk = computeWeightedLikelihood(H, R, zDim, Z, mPred, PPred, wPred)
        % COMPUTEWEIGHTEDLIKELIHOOD  Weighted Gaussian likelihood summation.

            nGM = size(mPred, 2);
            nMeas = size(Z, 2);
            totalLik = 0;

            for ll = 1:nGM
                Hm = H * mPred(:, ll);
                S = H * PPred(:, :, ll) * H' + R;
                S = (S + S') / 2;
                S = phd.MathUtils.ensurePositiveDefinite(S, 1e-6);

                Vs = chol(S);
                detS = prod(diag(Vs))^2;
                invS = inv(S); %#ok<MINV>

                likVals = zeros(1, nMeas);
                for zz = 1:nMeas
                    v = Z(:, zz) - Hm;
                    logLik = -0.5 * (zDim * log(2*pi) + log(detS) + v' * invS * v);
                    likVals(zz) = exp(logLik);
                end
                totalLik = totalLik + wPred(ll) * sum(likVals);
            end
            lk = totalLik;
        end

        function zGate = ellipsoidalGate(z, gamma, H, R, m, P)
        % ELLIPSOIDALGATE  Ellipsoidal gating based on Mahalanobis distance.

            nMeas = size(z, 2);
            if nMeas == 0
                zGate = [];
                return;
            end

            nComps = size(m, 2);
            validIdx = [];
            for jj = 1:nComps
                Sj = R + H * P(:, :, jj) * H';
                Sj = (Sj + Sj') / 2;
                Sj = phd.MathUtils.ensurePositiveDefinite(Sj, 1e-6);

                Vs = chol(Sj);
                invSqrtSj = inv(Vs); %#ok<MINV>
                nu = z - H * repmat(m(:, jj), [1, nMeas]);
                dist = sum((invSqrtSj' * nu).^2);
                validIdx = union(validIdx, find(dist < gamma));
            end
            zGate = z(:, validIdx);
        end

    end
end
