# SVO
Among the solutions for mono/stereo with/without IMU, SVO is fast, consistent & does not refix its path (leading to sudden teleportations). It does not implement SLAM, which eliminates the computational overhead. It's a bit old but seems to be what we need (the svo_pro used is modern however, with good documentation & support for Ubuntu 18 & 20).

SVO can possibly be used on both the front and bottom camera, giving us two streams of velocity odometry.

# Installation
https://github.com/uzh-rpg/rpg_svo_pro_open

SVO will not be able to be installed directly after nr22-software cloning. See clone & compile section in the above repo to see how to install it, or run the following commands in this parent folder (`svo`).

For [building on ARM](https://github.com/uzh-rpg/rpg_svo/wiki/Installation:-General-for-ARM-processors), set `export ARM_ARCHITECTURE=True` beforehand. See [here](https://github.com/uzh-rpg/rpg_svo_pro_open/issues/9#issuecomment-954076175) on how to fix `fast-neon` flag issues - checkout the `test/aarch64-compilation` branch instead.

```
vcs-import < ./rpg_svo_pro_open/dependencies.yaml
touch minkindr/minkindr_python/CATKIN_IGNORE
# vocabulary for place recognition
cd rpg_svo_pro_open/svo_online_loopclosing/vocabularies && ./download_voc.sh
cd ../../..
catkin build
```

When `catkin build`ing, some packages might fail and pending packages are abandoned afterwards. You can rerun `catkin build` multiple times or inspect why packages fail and fix the issues.

# Testing
After installing, see [doc/frontend/visual_frontend.md](https://github.com/uzh-rpg/rpg_svo_pro_open/blob/master/doc/frontend/visual_frontend.md) and [doc/frontend/frontend_fla.md](https://github.com/uzh-rpg/rpg_svo_pro_open/blob/master/doc/frontend/frontend_fla.md) to how to get it running from .bag files.

The modified repo `nyuad-robosub/rpg_svo_pro_open` allows you to run stereo SVO without the IMU (refer to commits **7d4ebbc** and **19a5d6f** for exact changes.) This can be tested using the same commands in [frontend_fla.md](https://github.com/uzh-rpg/rpg_svo_pro_open/blob/master/doc/frontend/frontend_fla.md) but with `fla_stereo.launch`:

```
roslaunch svo_ros fla_stereo.launch
rosbag play fla_stereo_imu.bag
```

# Configuration files
There are 2 config files in SVO: calibration for the camera data & parameters controlling how SVO works. They are stored in the svo_ros/param folder.

## Calibration
[See here](https://github.com/uzh-rpg/rpg_svo_pro_open/blob/master/doc/calibration.md) to understand where the calibration values go. It seems distortion values d0, d1, d2, d3 corresponds to k1, k2, p1, p2 - if the `plump_bob` model is used, I think it can go up to 5 parameters with the addition of k3 at the end.

The depthai calibration json provides a 14-element array for distortion coeffs. After tracing depthai calibration functions ([this](https://github.com/luxonis/depthai/blob/41b95a9e225562fcbb4f5815b0767afa2167d79d/calibrate.py#L525) to [this](https://github.com/luxonis/depthai/blob/41b95a9e225562fcbb4f5815b0767afa2167d79d/depthai_helpers/calibration_utils.py#L287) to [this](https://github.com/luxonis/depthai/blob/41b95a9e225562fcbb4f5815b0767afa2167d79d/depthai_helpers/calibration_utils.py#L427)), line 452 leads to: https://docs.opencv.org/4.x/d9/d6a/group__aruco.html where "distCoeffs	is (k1,k2,p1,p2[,k3[,k4,k5,k6],[s1,s2,s3,s4]])". 8 values are provided by depthai, which is up to k6.

## Parameters



## Other considered solutions
Benchmarkings:
https://rpg.ifi.uzh.ch/docs/ICRA18_Delmerico.pdf
https://arxiv.org/pdf/2007.11898.pdf
https://www.reddit.com/r/computervision/comments/s863pj/slam_vs_visual_odometry_approaches/
https://robotics.stackexchange.com/questions/10728/how-do-monocular-visual-odometry-algorithms-work

https://vision.in.tum.de/research/vslam/vi-dso
https://github.com/VladyslavUsenko/basalt-mirror
https://github.com/JakobEngel/dso
http://www.cvlibs.net/software/libviso/
https://github.com/avisingh599/mono-vo

https://github.com/HKUST-Aerial-Robotics/VINS-Mono
https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/euroc.gif
