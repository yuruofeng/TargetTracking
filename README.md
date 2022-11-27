# TargetTracking
This is a matlab value class for single target Bayesian filter, consisting of Kalman Filter, Extended Kalman Filter, Unscented Kalman Filter, Cubature Kalman Filter
and Particle Filter. Constant Turn Rate Model is used as the dynamic model and the sensor can provide radial distance and corresponding azimuth of the target.

dynamic model and the sensor model can be modified in file _SingleTargetFilter.m_.

**P.S.** the resampling algorithm for particle filter is a copy from a open source project.
