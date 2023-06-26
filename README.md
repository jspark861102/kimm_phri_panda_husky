# kimm_phri_panda_husky
KIMM pHRI application with Padna Robot Arm & Husky Mobile Platform

## 1. Prerequisites
### 1.1 Robot controller
```bash
git clone https://github.com/jspark861102/kimm_qpoases.git -b melodic
git clone https://github.com/jspark861102/kimm_hqp_controller_phri.git -b melodic
git clone https://github.com/jspark861102/kimm_trajectory_smoother.git -b melodic
```

### 1.2 Robot model and simulator
```bash
git clone https://github.com/jspark861102/franka_ros.git #my used version (0.8.1)
git clone https://github.com/jspark861102/husky.git -b melodic-devel #my used version (0.4.10)
git clone https://github.com/jspark861102/robotiq_2finger_grippers.git
git clone https://github.com/jspark861102/kimm_robots_description.git -b melodic
git clone https://github.com/jspark861102/kimm_mujoco_ros.git -b melodic
```

### 1.3 Unknown obejct parameter estimation
```bash
git clone https://github.com/jspark861102/kimm_object_estimation.git
git clone https://github.com/jspark861102/kimm_phri_msgs.git
```

### 1.4 pHRI task
```bash
git clone https://github.com/jspark861102/kimm_phri_panda_husky.git
git clone https://github.com/jspark861102/kimm_phri_viz.git
```

## 2. Run
### 2.1 Simulation
```bash
# Simulation 
roslaunch kimm_phri_panda_husky ns1_phri_simulation.launch
```

### 2.1 Real Robot
```bash
# object aware impedance controller for human-robot collaborative transportation
roslaunch kimm_phri_panda_husky ns1_phri_real.launch
```
