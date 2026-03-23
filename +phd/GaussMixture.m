classdef GaussMixture
% PHD.GAUSSMIXTURE  Gaussian Mixture Management Utilities
%   Provides pruning, merging, and capping operations for Gaussian mixture
%   components used in PHD filters.
%
%   Methods:
%       prune              - Remove components with weights below threshold
%       merge              - Merge components within Mahalanobis distance threshold
%       cap                - Keep only top N components by weight
%       manage             - Execute prune -> merge -> cap in sequence
%       adaptiveManage     - Adaptive management with dynamic thresholds
%       computeAdaptiveThresholds - Compute adaptive pruning/merging thresholds
%
%   Reference:
%       Vo, B.-N., & Ma, W.-K. (2006). The Gaussian mixture
%       probability hypothesis density filter. IEEE TSP.
%
%   See also: phd.MultiModelFilter, phd.Config

    methods (Static)

        function [wNew, xNew, PNew] = prune(w, x, P, threshold)
        % PRUNE  Remove Gaussian components with weights below threshold.
        %
        %   Inputs:
        %       w         - Weight vector [nComps x 1]
        %       x         - Mean matrix [dim x nComps]
        %       P         - Covariance stack [dim x dim x nComps]
        %       threshold - Pruning threshold
        %
        %   Outputs:
        %       wNew, xNew, PNew - Pruned Gaussian mixture parameters

            idx = find(w > threshold);
            wNew = w(idx);
            xNew = x(:, idx);
            PNew = P(:, :, idx);
        end

        function [wNew, xNew, PNew] = merge(w, x, P, threshold)
        % MERGE  Merge Gaussian components within Mahalanobis distance threshold.
        %   For each component with largest weight, merge nearby components.
        %   Uses pseudo-inverse with regularization for numerical stability.
        %
        %   Inputs:
        %       w         - Weight vector [nComps x 1]
        %       x         - Mean matrix [dim x nComps]
        %       P         - Covariance stack [dim x dim x nComps]
        %       threshold - Merging threshold (squared Mahalanobis distance)
        %
        %   Outputs:
        %       wNew, xNew, PNew - Merged Gaussian mixture parameters

            nComps = length(w);
            xDim = size(x, 1);
            remaining = 1:nComps;

            if all(w == 0) || nComps == 0
                wNew = []; xNew = []; PNew = [];
                return;
            end

            wNew = [];  xNew = [];  PNew = [];
            wCopy = w;
            regEpsilon = 1e-10;

            while ~isempty(remaining)
                [~, jj] = max(wCopy);
                jj = jj(1);
                Pj = P(:, :, jj);
                PjReg = Pj + regEpsilon * eye(size(Pj));
                iPj = pinv(PjReg);

                mergeIdx = [];
                for ii = remaining
                    diff = x(:, ii) - x(:, jj);
                    dist = diff' * iPj * diff;
                    if dist <= threshold
                        mergeIdx = [mergeIdx, ii]; %#ok<AGROW>
                    end
                end

                if isempty(mergeIdx)
                    wNew = [wNew; wCopy(jj)]; %#ok<AGROW>
                    xNew = [xNew, x(:, jj)]; %#ok<AGROW>
                    PNew = cat(3, PNew, P(:, :, jj));
                    remaining = setdiff(remaining, jj);
                    wCopy(jj) = -1;
                else
                    wMerged = sum(w(mergeIdx));
                    wMat = repmat(w(mergeIdx)', [xDim, 1]);
                    xMerged = sum(wMat .* x(:, mergeIdx), 2);

                    wMat3 = reshape(w(mergeIdx), [1, 1, length(mergeIdx)]);
                    wMat3 = repmat(wMat3, [xDim, xDim, 1]);
                    PMerged = sum(wMat3 .* P(:, :, mergeIdx), 3);

                    xMerged = xMerged / wMerged;
                    PMerged = PMerged / wMerged;
                    PMerged = (PMerged + PMerged') / 2;

                    wNew = [wNew; wMerged]; %#ok<AGROW>
                    xNew = [xNew, xMerged]; %#ok<AGROW>
                    PNew = cat(3, PNew, PMerged);
                    remaining = setdiff(remaining, mergeIdx);
                    wCopy(mergeIdx) = -1;
                end
            end
        end

        function [wNew, xNew, PNew] = cap(w, x, P, maxNumber)
        % CAP  Truncate to maxNumber components with highest weights.
        %   Renormalizes weights to preserve total weight.
        %
        %   Inputs:
        %       w         - Weight vector [nComps x 1]
        %       x         - Mean matrix [dim x nComps]
        %       P         - Covariance stack [dim x dim x nComps]
        %       maxNumber - Maximum number of components to keep
        %
        %   Outputs:
        %       wNew, xNew, PNew - Capped Gaussian mixture parameters

            if length(w) > maxNumber
                [~, idx] = sort(w, 1, 'descend');
                wNew = w(idx(1:maxNumber));
                wNew = wNew * (sum(w) / sum(wNew));
                xNew = x(:, idx(1:maxNumber));
                PNew = P(:, :, idx(1:maxNumber));
            else
                wNew = w;
                xNew = x;
                PNew = P;
            end
        end

        function [w, x, P] = manage(w, x, P, elimThresh, mergeThresh, maxNum)
        % MANAGE  Execute prune -> merge -> cap in sequence.
        %   Convenience function for complete Gaussian mixture management.
        %
        %   Inputs:
        %       w, x, P      - Gaussian mixture parameters
        %       elimThresh   - Pruning threshold
        %       mergeThresh  - Merging threshold
        %       maxNum       - Maximum number of components
        %
        %   Outputs:
        %       w, x, P - Managed Gaussian mixture parameters

            [w, x, P] = phd.GaussMixture.prune(w, x, P, elimThresh);
            [w, x, P] = phd.GaussMixture.merge(w, x, P, mergeThresh);
            [w, x, P] = phd.GaussMixture.cap(w, x, P, maxNum);
        end

        function [w, x, P, threshInfo] = adaptiveManage(w, x, P, config)
        % ADAPTIVEMANAGE  Adaptive Gaussian mixture management with dynamic thresholds.
        %   Automatically adjusts pruning/merging thresholds based on:
        %     - Current component count vs target estimate
        %     - Weight distribution statistics
        %     - Covariance condition numbers
        %
        %   Inputs:
        %       w, x, P - Gaussian mixture parameters
        %       config  - Configuration struct with fields:
        %                 .baseElimThresh  - Base elimination threshold (default: 1e-4)
        %                 .baseMergeThresh - Base merge threshold (default: 10)
        %                 .maxNum          - Maximum components (default: 100)
        %                 .targetRatio     - Target components per estimated target (default: 10)
        %
        %   Outputs:
        %       w, x, P     - Managed Gaussian mixture parameters
        %       threshInfo  - Struct with computed thresholds (for debugging)

            if nargin < 4
                config = struct();
            end
            
            if ~isfield(config, 'baseElimThresh')
                config.baseElimThresh = 1e-4;
            end
            if ~isfield(config, 'baseMergeThresh')
                config.baseMergeThresh = 10;
            end
            if ~isfield(config, 'maxNum')
                config.maxNum = 100;
            end
            if ~isfield(config, 'targetRatio')
                config.targetRatio = 10;
            end
            
            threshInfo = struct();
            nComps = length(w);
            
            if nComps == 0
                threshInfo.elimThresh = config.baseElimThresh;
                threshInfo.mergeThresh = config.baseMergeThresh;
                threshInfo.maxNum = config.maxNum;
                return;
            end
            
            nTargets = sum(w > 0.5);
            if nTargets < 1
                nTargets = 1;
            end
            
            [elimThresh, mergeThresh, maxNum] = phd.GaussMixture.computeAdaptiveThresholds(...
                w, nComps, nTargets, config);
            
            threshInfo.elimThresh = elimThresh;
            threshInfo.mergeThresh = mergeThresh;
            threshInfo.maxNum = maxNum;
            threshInfo.nTargets = nTargets;
            
            [w, x, P] = phd.GaussMixture.prune(w, x, P, elimThresh);
            [w, x, P] = phd.GaussMixture.merge(w, x, P, mergeThresh);
            [w, x, P] = phd.GaussMixture.cap(w, x, P, maxNum);
        end

        function [elimThresh, mergeThresh, maxNum] = computeAdaptiveThresholds(...
                w, nComps, nTargets, config)
        % COMPUTEADAPTIVETHRESHOLDS  Compute adaptive thresholds for GM management.
        %
        %   Adaptive strategy:
        %     - When components >> targets: aggressive pruning/merging
        %     - When components ~ targets: conservative management
        %     - Max components scale with estimated target count
        %
        %   Inputs:
        %       w        - Weight vector
        %       nComps   - Current component count
        %       nTargets - Estimated number of targets
        %       config   - Base configuration parameters
        %
        %   Outputs:
        %       elimThresh  - Adaptive elimination threshold
        %       mergeThresh - Adaptive merge threshold
        %       maxNum      - Adaptive maximum component count

            compRatio = nComps / max(nTargets * config.targetRatio, 1);
            
            wSorted = sort(w, 'descend');
            if length(wSorted) >= 3
                top3sum = sum(wSorted(1:min(3, length(wSorted))));
                weightSkew = wSorted(1) / max(top3sum, eps);
            else
                weightSkew = 0.5;
            end
            
            if compRatio > 3.0
                scaleX = 2.5;
                mergeScale = 0.5;
            elseif compRatio > 2.0
                scaleX = 1.5;
                mergeScale = 0.7;
            elseif compRatio > 1.5
                scaleX = 1.2;
                mergeScale = 0.85;
            else
                scaleX = 1.0;
                mergeScale = 1.0;
            end
            
            elimThresh = config.baseElimThresh * scaleX;
            mergeThresh = config.baseMergeThresh * mergeScale;
            
            adaptiveMax = round(nTargets * config.targetRatio);
            maxNum = min(max(adaptiveMax, 20), config.maxNum);
        end

        function nTargets = extractTargets(w, threshold)
        % EXTRACTTARGETS  Extract target number estimate from Gaussian mixture.
        %   Counts components with weights exceeding threshold.
        %
        %   Inputs:
        %       w         - Weight vector [nComps x 1]
        %       threshold - Extraction threshold (default: 0.5)
        %
        %   Outputs:
        %       nTargets - Estimated number of targets

            if nargin < 2
                threshold = 0.5;
            end
            nTargets = sum(w > threshold);
        end

        function [states, weights] = extractState(w, x, threshold)
        % EXTRACTSTATE  Extract state estimates from Gaussian mixture.
        %   Extracts components with weights exceeding threshold as target states.
        %
        %   Inputs:
        %       w         - Weight vector [nComps x 1]
        %       x         - Mean matrix [dim x nComps]
        %       threshold - Extraction threshold (default: 0.5)
        %
        %   Outputs:
        %       states  - Extracted state matrix [dim x nTargets]
        %       weights - Corresponding weight vector

            if nargin < 3
                threshold = 0.5;
            end
            idx = find(w > threshold);
            if isempty(idx)
                states = [];
                weights = [];
            else
                states = x(:, idx);
                weights = w(idx);
            end
        end

    end
end
