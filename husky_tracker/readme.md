# 无人机自动驾驶软件系列 E06: 简单全局目标追踪

前几讲中，我们讲述了如何设置仿真环境，如何通过python控制无人机，如何使用双目进行深度估计。本讲中，我会简要介绍如何通过 [GAAS-Object-Tracking
](https://github.com/generalized-intelligence/GAAS-Object-Tracking.git) 模块进行目标跟踪。注意，此实现比较简单，实际情况下需要使用更为复杂的方法。

>请注意，该全局跟踪是的一个模块，无法独立使用。

请移步到我们的 Gitbook 主页继续阅读[详细教程](https://gaas.gitbook.io/)

# HUSKY模型
husky模型下载参考，https://github.com/TaarLab/husky_joy

# 打开新的终端，开启全局追踪算法

cd (GAAS-Object-Tracking_PATH)/KCF
./bin/startRosService

此时是没有任何输出的，算法正在等待通过 ROS Service 给算法设置起始位置。
GAAS 提供了一个小的 GUI 程序来快速设置初始坐标点。使用如下命令即可运行：

python set_init.py

运行 set_init.py ，在弹出的界面中画一个矩形框住目标，然后在键盘上点击 s 将目标初始位置通过 ROS Service 传递给全局追踪算法。

开始全局追踪
python track_and_move.py
启动小车控制插件：

> 在追踪过程中，请勿关闭追踪算法。如果你觉得需要重新追踪物体，请重启追踪算法，并运行 set_init.py 重新设置物体起始坐标。
