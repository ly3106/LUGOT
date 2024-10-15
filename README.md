# LUGOT

This is the source code of our paper. If you find this code useful in your research, please consider citing our work:

LUGOT: LiDAR SLAM and UWB Fusion for Robust Global Given Object Tracking

## version info
- Version: 1.1.0
- Release Date: 2024-09-27
- Notes:
  - modify for 2nd review.

## Run
直接运行main函数，其会对参数进行初始化并调用所有其他功能程序
运行过程中出现的警告不影响结果，可以忽略

部分步骤说明：
1. 运行`uwb_to_map10.m`，转换坐标系，设置其中的`SLAM_FLAG`参数，选择不同的SLAM的结果
2. 加载`hyper_parameter.mat`,加载超参数
2. 运行`LUGOT.slx`，进行滤波
3. 运行`plot_resault.m`,可视化滤波结果