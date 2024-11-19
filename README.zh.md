# LUGOT

这个项目是我们论文的源代码。如果您发现此代码对您的研究有用，请考虑引用我们的工作：

《LUGOT: LiDAR and UWB Fusion for Global Given Object Tracking under Mobile Anchors》

（LUGOT：LiDAR SLAM和UWB融合以实现稳健的全局给定对象跟踪）

## 运行
直接运行`main.m`函数，其会对参数进行初始化并调用所有其他功能程序。
运行过程中出现的警告不影响结果，可以忽略。

### 部分步骤说明：
1. 运行`uwb_to_map0.m`，转换坐标系，设置其中的`SLAM_FLAG`参数，选择不同的SLAM的结果；
2. 加载`hyper_parameter.mat`,加载超参数
3. 运行`LUGOT.slx`，进行滤波；
4. 运行`smoothing_spline_fit.m`，使用平滑样条曲线（smoothing spline）拟合UWB轨迹；
5. 运行`calc_process_noise.m`，计算过程噪声；
6. 运行`plot_resault.m`,可视化滤波结果；
7. 运行`plot_statistics.m'`，绘制统计学录播效果数据，如各种RMSE。

## 版本信息
- 版本：1.2.0
- 发行日期：2024-11-19
- 说明：
  - 论文3审修改。