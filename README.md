# MRS open_vins Core

Metapackage containing submodules, launch files, config files, scripts, and other files necessary for running the MRS UAV system with open_vins state estimation.

> :warning: **Attention please: This README needs work.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.

## Package mrs_open_vins_core

This package is a minimal runnable OpenVINS example. It does not have the other nodes yet - imu filter, republisher and some estimators. Configs were taken from the ROS1 package (estimator_plugin.yaml, kalibr_imu_chain.yaml and kalibr_imucam_chain.yaml).

## Submodules

| Repository                                                                                  |
|---------------------------------------------------------------------------------------------|
| [open_vins](https://github.com/ctu-mrs/open_vins)                                           |
| [mrs_open_vins_estimator_plugin](https://github.com/ctu-mrs/mrs_open_vins_estimator_plugin) |
| [mrs_vins_republisher](https://github.com/ctu-mrs/mrs_vins_republisher)                     |
| [mrs_vins_imu_filter](https://github.com/ctu-mrs/mrs_vins_imu_filter)                       |
