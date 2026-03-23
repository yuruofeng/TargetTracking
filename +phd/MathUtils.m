classdef MathUtils
% PHD.MATHUTILS  Mathematical Utilities for PHD Filters
%   Provides common mathematical functions including coordinate conversion,
%   matrix positive definitization, and state dimension transformation.
%
%   Methods:
%       ensurePositiveDefinite - Fix non-positive definite matrices
%       cartToPolar            - Cartesian to polar coordinate conversion
%       polarToCart            - Polar to Cartesian coordinate conversion
%       expandState4to6        - Expand 4D state to 6D unified state
%       expandCov4to6          - Expand 4x4 covariance to 6x6
%       reduceState6to4        - Reduce 6D state/covariance to 4D
%
%   See also: phd.GaussMixture, utils.FilterUtils

    methods (Static)

        function P = ensurePositiveDefinite(P, epsilon)
        % ENSUREPOSITIVEDEFINITE  Fix non-positive definite matrices.
        %   Replaces non-positive eigenvalues with epsilon using eigendecomposition.
        %
        %   Inputs:
        %       P       - Matrix to fix
        %       epsilon - Replacement threshold (default: 1e-6)
        %
        %   Outputs:
        %       P - Fixed positive definite matrix

            if nargin < 2
                epsilon = 1e-6;
            end

            if any(isnan(P(:))) || any(isinf(P(:)))
                P(isnan(P)) = 0;
                P(isinf(P)) = realmax;
                P = (P + P') / 2;
            end

            try
                [V, D] = eig(P);
            catch
                P = P + epsilon * eye(size(P));
                [V, D] = eig(P);
            end

            eigVals = diag(D);
            eigVals(eigVals <= 0 | isnan(eigVals) | isinf(eigVals)) = epsilon;

            P = V * diag(eigVals) * V';
            P = (P + P') / 2;
        end

        function z = cartToPolar(m)
        % CARTTOPOLAR  Cartesian to polar coordinate conversion.
        %
        %   Inputs:
        %       m - State matrix [x; y; ...]
        %
        %   Outputs:
        %       z - Polar coordinates [theta; range]

            p = m([1, 2], :);
            z(1, :) = atan2(p(1, :), p(2, :));
            z(2, :) = sqrt(sum(p.^2));
        end

        function z = polarToCart(zPol)
        % POLARTOCART  Polar to Cartesian coordinate conversion.
        %
        %   Inputs:
        %       zPol - Polar coordinates [theta; range]
        %
        %   Outputs:
        %       z    - Cartesian coordinates [x; y]

            z(1, :) = zPol(2, :) .* sin(zPol(1, :));
            z(2, :) = zPol(2, :) .* cos(zPol(1, :));
        end

        function mExp = expandState4to6(m4)
        % EXPANDSTATE4TO6  Expand 4D state [x vx y vy] to 6D [x vx 0 y vy 0].
        %   Used for conversion between CV/CT states and unified 6D state.
        %
        %   Inputs:
        %       m4 - 4D state matrix [4 x nComps]
        %
        %   Outputs:
        %       mExp - 6D state matrix [6 x nComps]

            if isempty(m4)
                mExp = [];
                return;
            end
            xPart = [m4([1, 2], :); zeros(1, size(m4, 2))];
            yPart = [m4([3, 4], :); zeros(1, size(m4, 2))];
            mExp = [xPart; yPart];
        end

        function PExp = expandCov4to6(P4stack)
        % EXPANDCOV4TO6  Expand 4x4 covariance to 6x6 covariance.
        %   Expands [x,vx,y,vy] structure to [x,vx,ax,y,vy,ay] with zero acceleration variance.
        %
        %   Inputs:
        %       P4stack - 4x4xnComps covariance stack
        %
        %   Outputs:
        %       PExp - 6x6xnComps covariance stack

            nComps = size(P4stack, 3);
            if nComps == 0
                PExp = zeros(6, 6, 0);
                return;
            end
            PExp = zeros(6, 6, nComps);
            for ii = 1:nComps
                P4 = P4stack(:, :, ii);
                P6 = zeros(6, 6);
                P6([1, 2], [1, 2]) = P4([1, 2], [1, 2]);
                P6([1, 2], [4, 5]) = P4([1, 2], [3, 4]);
                P6([4, 5], [1, 2]) = P4([3, 4], [1, 2]);
                P6([4, 5], [4, 5]) = P4([3, 4], [3, 4]);
                PExp(:, :, ii) = P6;
            end
        end

        function [m4, P4] = reduceState6to4(m6, P6)
        % REDUCESTATE6TO4  Reduce 6D state/covariance to 4D.
        %   Extracts [x vx y vy] components and corresponding covariance submatrix.
        %
        %   Inputs:
        %       m6 - 6D state matrix [6 x nComps]
        %       P6 - 6x6xnComps covariance stack
        %
        %   Outputs:
        %       m4 - 4D state matrix [4 x nComps]
        %       P4 - 4x4xnComps covariance stack

            if isempty(m6)
                m4 = [];
                P4 = zeros(4, 4, 0);
                return;
            end
            m4 = [m6([1, 2], :); m6([4, 5], :)];
            nComps = size(P6, 3);
            P4 = zeros(4, 4, nComps);
            for ii = 1:nComps
                tp = P6(:, :, ii);
                rows = [tp([1, 2], :); tp([4, 5], :)];
                P4(:, :, ii) = [rows(:, [1, 2]), rows(:, [4, 5])];
            end
        end

    end
end
