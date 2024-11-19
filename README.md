# LUGOT

This is the source code of our paper. If you find this code useful in your research, please consider citing our work:

LUGOT: LiDAR SLAM and UWB Fusion for Robust Global Given Object Tracking

## Run

### Instructions for Running the Program

1. **Main Function**  
   Directly run the `main.m` function, which initializes parameters and invokes all other functional programs.  
   Warnings encountered during runtime do not affect the results and can be safely ignored.  

### Step-by-Step Guide

1. **Coordinate Transformation**  
   Execute `uwb_to_map0.m` to transform the coordinate system.  
   Set the `SLAM_FLAG` parameter within this script to select results from different SLAM systems.  

2. **Load Hyperparameters**  
   Load the `hyper_parameter.mat` file to import necessary hyperparameters.

3. **Filtering**  
   Run `LUGOT.slx` to perform *KF filtering.

4. **Smoothing Spline Fitting**  
   Run `smoothing_spline_fit.m` to fit the UWB trajectory using a smoothing spline.

6. **Process Noise Calculation**  
   Run `calc_process_noise.m` to calculate process noise.

7. **Visualization**  
   Execute `plot_resault.m` to visualize the filtering results.

8. **Statistical Analysis**  
   Run `plot_statistics.m` to generate statistical data visualizations, including various RMSE metrics.

## Version info
- Version: 1.2.0
- Release Date: 2024-11-19
- Notes:
  - modify for 3rd review.