# Target Tracking Algorithms

MATLAB implementation of single-target tracking algorithms covering both **Detect-Before-Track (DBT)** and **Track-Before-Detect (TBD)** approaches with a modular, package-based architecture.

## Project Structure

```
TargetTracking/
├── +dbt/                          # Detect-Before-Track Module
│   ├── BaseFilter.m               # Abstract filter interface
│   ├── KalmanFilterBase.m         # Kalman filter base class
│   ├── EKF.m                      # Extended Kalman Filter
│   ├── UKF.m                      # Unscented Kalman Filter
│   ├── CKF.m                      # Cubature Kalman Filter
│   ├── MotionModelEKF.m           # Unified motion model EKF
│   ├── ParticleFilter.m           # Particle Filter (SIR)
│   ├── IMM.m                      # Interacting Multiple Model
│   ├── Config.m                   # DBT configuration
│   ├── ConfigIMM.m                # IMM configuration
│   ├── ConfigManeuver.m           # Maneuver configuration
│   ├── MotionModelConfig.m        # Motion model configuration
│   ├── Scenario.m                 # Trajectory/measurement generation
│   ├── ScenarioManeuver.m         # Maneuver scenario generation
│   └── FilterFactory.m            # Filter factory
├── +tbd/                          # Track-Before-Detect Module
│   ├── BaseTbd.m                  # Abstract TBD interface
│   ├── DpTbd.m                    # Dynamic Programming TBD
│   ├── PfTbd.m                    # Particle Filter TBD
│   ├── Config.m                   # TBD configuration
│   ├── Scenario.m                 # Image-domain scenario
│   └── TbdFactory.m               # TBD factory
├── +utils/                        # Shared Utilities
│   ├── FilterUtils.m              # Cholesky, sigma points, resampling
│   └── MeasurementModel.m         # CT/CV models, PSF
├── +viz/                          # Visualization Module
│   └── Visualizer.m               # Colorblind-friendly plotting
├── demos/                         # Demonstration Scripts
│   ├── demoDbt.m                  # DBT demonstration
│   ├── demoTbd.m                  # TBD demonstration
│   ├── demoManeuver.m             # Maneuver tracking demo
│   └── demoAll.m                  # Complete suite demo
├── tests/                         # Unit Tests
│   ├── TestDbtFilters.m           # DBT filter tests
│   ├── TestTbdAlgorithms.m        # TBD algorithm tests
│   ├── TestManeuverTracking.m     # Maneuver tracking tests
│   ├── TestVisualization.m        # Visualization tests
│   └── runAllTests.m              # Test runner
├── docs/                          # Documentation
├── config/                        # Configuration files
│   └── ProjectConfig.m            # Project configuration
├── resources/                     # Resource files
├── startup.m                      # Project initialization
├── .gitignore                     # Git ignore rules
└── README.md                      # This file
```

## Quick Start

### Initialize Project
```matlab
>> cd TargetTracking
>> startup()                       % Initialize paths and show info
```

### Run Complete Demo
```matlab
>> run('demos/demoAll.m')
```

### Run Individual Demos
```matlab
>> run('demos/demoDbt.m')          % DBT algorithms
>> run('demos/demoTbd.m')          % TBD algorithms
>> run('demos/demoManeuver.m')     % Maneuvering target tracking
```

### Run Tests
```matlab
>> run('tests/runAllTests.m')
```

## Algorithms

### Detect-Before-Track (DBT)

| Filter | Description | State Dim | Complexity |
|--------|-------------|-----------|------------|
| **EKF** | Extended Kalman Filter (Jacobian) | 5 (CT) | O(n²) |
| **UKF** | Unscented Kalman Filter (sigma points) | 5 (CT) | O(n³) |
| **CKF** | Cubature Kalman Filter (cubature points) | 5 (CT) | O(n³) |
| **MotionModelEKF** | Unified EKF (CV/CA/CT/Singer/CS) | 4-6 | O(n²) |
| **ParticleFilter** | Bootstrap PF with systematic resampling | 5 (CT) | O(Np×n) |
| **IMM** | Interacting Multiple Model | 4-6 | O(M×n³) |

### Track-Before-Detect (TBD)

| Algorithm | Description | State Dim | Complexity |
|-----------|-------------|-----------|------------|
| **DP-TBD** | Dynamic Programming grid search | 2 (pos) | O(N²×K) |
| **PF-TBD** | Particle Filter with log-likelihood | 5 (pos,vel,amp) | O(Np×K) |

## API Reference

### Factory Pattern (Recommended)

```matlab
% Create filters using factory
filter = dbt.FilterFactory.create('EKF-CV', 'dt', 1.0, 'q', 0.1);
imm = dbt.FilterFactory.createDefaultIMM({'CV', 'CA', 'CT'});

% Create TBD algorithms using factory
dp = tbd.TbdFactory.create('DP');
pf = tbd.TbdFactory.create('PF');
```

### DBT Module

```matlab
% Configuration
cfg = dbt.Config('numSteps', 100, 'dt', 1);

% Scenario generation
scenario = dbt.Scenario(cfg);
[truthX, meas] = scenario.generate();

% Motion model EKF (supports CV, CA, CT, Singer, CS)
mmConfig = dbt.MotionModelConfig('CT');
ekf = dbt.MotionModelEKF(mmConfig);
[states, covars] = ekf.run(meas, x0, P0);

% Classic CT-model EKF
ekf = dbt.EKF(cfg);
[states, covars] = ekf.run(meas, x0, P0);
```

### TBD Module

```matlab
% Configuration
cfg = tbd.Config('numFrames', 50, 'gridSize', [100, 100]);

% Scenario generation
scenario = tbd.Scenario(cfg);
[~, trueState, measData, psfKernel] = scenario.generate();

% DP-TBD
dp = tbd.DpTbd(cfg);
dp.run(measData, psfKernel);
[track, score] = dp.getResults();

% PF-TBD
pf = tbd.PfTbd(cfg);
pf.run(measData, trueState(1,:), psfKernel);
[posRmse, velRmse] = pf.computeRmse(trueState);
```

### Visualization

```matlab
% Colorblind-friendly palette (CUD colors)
viz.Visualizer.COLORS.blue    % #0077BB
viz.Visualizer.COLORS.orange  % #EE7733
viz.Visualizer.COLORS.cyan    % #33BBEE
viz.Visualizer.COLORS.green   % #228833

% Plot functions
viz.Visualizer.plotDbtTrajectory(truthX, meas, estimates, labels)
viz.Visualizer.plotRmseComparison(rmseData, labels, 'Position RMSE')
viz.Visualizer.plotManeuverComparison(results, configs)
viz.Visualizer.plotTbdTrajectory(trueState, estimates, labels)
```

## Motion Models

The `MotionModelEKF` supports multiple motion models:

| Model | State Vector | Description |
|-------|--------------|-------------|
| **CV** | [x, vx, y, vy] | Constant Velocity |
| **CA** | [x, vx, ax, y, vy, ay] | Constant Acceleration |
| **CT** | [x, vx, y, vy, ω] | Coordinated Turn |
| **Singer** | [x, vx, ax, y, vy, ay] | Singer Acceleration Model |
| **CS** | [x, vx, ax, y, vy, ay] | Current Statistical Model |

```matlab
% Create motion model EKF
cfg = dbt.MotionModelConfig('CV', 'dt', 1.0, 'q', 0.1);
ekf = dbt.MotionModelEKF(cfg);

% Use with IMM
imm = dbt.FilterFactory.createDefaultIMM({'CV', 'CA', 'CT'});
```

## Design Patterns

The project uses the following design patterns:

| Pattern | Implementation | Purpose |
|---------|---------------|---------|
| **Strategy** | BaseFilter, BaseTbd | Interchangeable algorithms |
| **Factory** | FilterFactory, TbdFactory | Unified object creation |
| **Template Method** | KalmanFilterBase | Common algorithm structure |
| **Abstract Base Class** | BaseFilter, BaseTbd | Define interfaces |

## Demo Parameters

### demoDbt Options
| Parameter | Default | Description |
|-----------|---------|-------------|
| `mcRuns` | 1 | Monte Carlo iterations |
| `numSteps` | 100 | Time steps |
| `dt` | 1 | Sampling interval [s] |
| `numParticles` | 500 | PF particles |
| `saveResults` | false | Save to .mat file |
| `showPlots` | true | Display plots |

### demoTbd Options
| Parameter | Default | Description |
|-----------|---------|-------------|
| `gridSize` | [100, 100] | Image dimensions |
| `numFrames` | 50 | Number of frames |
| `snr` | 9.5 | Target SNR [dB] |
| `numParticles` | 500 | PF particles |
| `motionModel` | 'CV' | Motion model type |
| `saveResults` | false | Save to .mat file |
| `animate` | false | Show animation |

## Requirements

- MATLAB R2020b or later
- Signal Processing Toolbox (optional)
- Statistics and Machine Learning Toolbox (optional)

## References

1. He You. *Radar Data Processing and Applications* (Chinese)
2. Van der Merwe, R. (2004). *Sigma-Point Kalman Filters*
3. Arasaratnam, I. (2009). *Cubature Kalman Filters*
4. Barniv, Y. (1985). *Dynamic Programming for Dim Target Detection*
5. Salmond, D. J. (2001). *Track-Before-Detect Techniques*
6. Bar-Shalom, Y. (2001). *Estimation with Applications to Tracking and Navigation*

## License

Provided for academic and research purposes. Please cite appropriately if used in publications.
