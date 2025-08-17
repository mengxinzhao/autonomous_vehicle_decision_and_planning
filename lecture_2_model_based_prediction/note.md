## Notes
### Overview
1. prediction system eval metrics. FDE: final displacement error,  ADE: average displacement error
2. prediction system  architecture usual view
```
   objects                                      - trajectory/intention, multimodel, uncertainty
   static road graph   --> prediction model --> - various objects, car truck etc
   other informations                           - scenarios: urban highspeed, parking etc
```
3. multimodel: simple explanation is subject to multiple gaussian distribution
### Prediction system 
- 简单运动模型 定速预测，定曲率预测
- 结合场景先验知识：基于手工特征标注（基于车辆行为特征， 对车辆意图进行分类）
   - Example：预测lane change。 
   手工标记距离目标车道横向距离，横向速度，道路实现，虚线，然后train SVM 做预测。model input： 历史lateral position，转向角，速度。output：next tick 0 - 不去该车道，  1-去该车道。 Appolo 5.0 用MLP预测障碍车会选择哪一条车道，输出车道线的概率
   - Example：预测intersection（Appollo）
   以障碍车朝向为参考方向，将intersection 划分成12 个扇区；记录每个扇形区域是否有离开该路口的车道；将问题转化为12个分类问题
   - limitation：标记好坏直接影响预测
- hidden markov HMM
- model based trajectory prediction *
核心思想是通过短时预测（简单的定速 定曲率） 结合地图contrl point 补全长时间预测。注意点是此时的轨迹生成是轻量化的planner，不是规控中full blown的 搜索+采样+优化。 典型算法有：

   - 预判行车意图后，抽取地图control point 用高阶曲线（如bezier）生成轨迹 [Trajectory generator for autonomous vehicles in urban environments](https://inria.hal.science/hal-00789760/document)
   - 结合状态信息，航向角，速度，加速度等 作为待拟合路径曲线的初始值和约束条件。利用曲线系数，根据时间间隔对曲线采样，得到的采样点即为拟合后的路径点（这个和工作紧密相关）[Randomized Bidirectional B-Spline Parameterization Motion Planning](https://ieeexplore.ieee.org/abstract/document/7274361) no free access on ieee. similar paper [Planning Motion Trajectories for Mobile Robots Using Splines](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf)
   - Epsilon: Intention prediction network github https://github.com/HKUST-Aerial-Robotics/EPSILON. paper in the lecture repo
      - 有没有可能把这个搬到仿真里去给agent用？
      - 讲课老师强调forward simulation；用idm 生成前向仿真 提供前向交互的各种可能性；如果前向simulation 是基于特定轨迹曲线，那交互的选项就会减小。



