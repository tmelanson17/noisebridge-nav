# OS2D detector

ROS2 node to run One-Stage One-Shot Object Detection (OS2D) on turtlebot4 camera feed. 

# Installation

- Create conda environment: `conda create -n ros-env python=3.10`
- Activate conda environment: `conda activate ros-env`
- install pip: `conda install pip`
- Install torch
    - GPU: `conda install pytorch torchvision torchaudio pytorch-cuda=11.7 -c pytorch -c nvidia`
    - CPU: `conda install pytorch torchvision torchaudio cpuonly -c pytorch`
- Install cv2: `pip install opencv-python`
- Install misc dependencies: `conda install -c conda-forge yacs=0.1.6`
- Install colcon: `pip install -U colcon-common-extensions`

# Build & run
- Build using colcon:
    ```
    cd ros_ws
    colcon build --packages-select os2d_detector
    ```
- Run ROS2 node: `ros2 run os2d_detector detector_node`


# Credit

os2d code is taken from here: https://github.com/aosokin/os2d

```
@inproceedings{
    osokin20os2d,
    title = {{OS2D}: One-Stage One-Shot Object Detection by Matching Anchor Features},
    author = {Anton Osokin and Denis Sumin and Vasily Lomakin},
    booktitle = {proceedings of the European Conference on Computer Vision (ECCV)},
    year = {2020} 
}
```