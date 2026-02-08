# Target Tracking Algorithms

MATLAB implementation of single-target tracking algorithms covering both **Detect-Before-Track (DBT)** and **Track-Before-Detect (TBD)** approaches.

## Project Structure

| File | Description |
|---|---|
| `TargetTracker.m` | Unified static class containing all tracking algorithms, scenario generators, and visualization utilities. |
| `mainDemo.m` | Single entry-point demo script with configurable mode selection, Monte Carlo support, and optional data export. |
| `README.md` | This file. |

## Quick Start

```matlab
>> mainDemo
```

Edit the **User Options** section at the top of `mainDemo.m` to control behaviour:

```matlab
demoMode   = 'both';    % 'both' | 'dbt' | 'tbd'
mcRuns     = 1;         % Monte Carlo iterations for DBT filters
saveData   = false;     % set true to export results to .mat
saveFile   = 'tracking_results.mat';
```

## Algorithms

### Detect-Before-Track (DBT)

Four Bayesian filters operating on a **Constant Turn-Rate** dynamic model with radar range/azimuth measurements:

- **EKF** — Extended Kalman Filter (first-order linearisation)
- **UKF** — Unscented Kalman Filter (sigma-point transform)
- **CKF** — Cubature Kalman Filter (third-degree cubature rule)
- **PF** — Bootstrap Particle Filter with systematic resampling

### Track-Before-Detect (TBD)

Two algorithms operating on raw image-domain measurements (Gaussian PSF target in AWGN):

- **DP-TBD** — Dynamic Programming: exhaustive grid search with O(N²K) complexity
- **PF-TBD** — Particle Filter: sequential Monte Carlo with log-likelihood weighting, O(NK) complexity

## API Reference

All functionality is accessed via static methods on `TargetTracker`:

```matlab
% Configuration
dbtCfg = TargetTracker.defaultDbtConfig();
tbdCfg = TargetTracker.defaultTbdConfig();

% DBT scenario
[truthX, meas] = TargetTracker.generateDbtScenario(dbtCfg);

% DBT filters
[xPre, pPre]      = TargetTracker.ekfPredict(x, P, dbtCfg);
[xUpd, pUpd]      = TargetTracker.ekfUpdate(z, xPre, pPre, dbtCfg);
[wSp, xPre, pPre] = TargetTracker.ukfPredict(x, P, dbtCfg);
[xUpd, pUpd]      = TargetTracker.ukfUpdate(z, xPre, pPre, wSp, dbtCfg);
[xPre, pPre]      = TargetTracker.ckfPredict(x, P, dbtCfg);
[xUpd, pUpd]      = TargetTracker.ckfUpdate(z, xPre, pPre, dbtCfg);

% TBD algorithms
trueState = TargetTracker.generateTbdTrajectory(tbdCfg);
[measData, psf] = TargetTracker.generateTbdMeasurement(trueState, tbdCfg);
[track, score, V] = TargetTracker.runDpTbd(measData, psf, tbdCfg);
[est, pRmse, vRmse] = TargetTracker.runPfTbd(measData, trueState, psf, tbdCfg);
```

## Requirements

- MATLAB R2020b or later
- Signal Processing Toolbox
- Statistics and Machine Learning Toolbox

## License

Provided for academic and research purposes. Please cite appropriately if used in publications.
