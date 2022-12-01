# 544_lab3

## Prerequistes

Please install these prerequistes:
```
pip3 install opencv-python
pip3 install numpy
```

## How to build

- clone the repo such that the folder `interfaces/` and `lab3/` are located inside your `src/` directory of your ros2 workspace
    - e.g your directory might look like:
    ```
    ros2_ws/
      build/
      install/
      launch/
      src/
        interfaces/
        lab3/
    ```
- cd into your workspace root (e.g. `ros2_ws/`) and then the run the commands in order:
    - `colcon build --packages-select interfaces`
    - `colcon build --packages-select lab3`
    - `. install/setup.bash`

## How to run

### Simulation

1. Follow the lab instructions to open Gazebo and RViz for the OGM

2. `ros2 run lab3 controller` will create the controller node. This can be activated all time since it only continuously updates the current pose of the robot until a service request is put it. 

3. `ros2 run lab3 client <start_x> <start_y> <end_x> <end_y>` will start the client node, which will send the start and end position as a Point request to the controller. The controller will perform A* on these start and end points on a pre-generated OGM, and run a controller on each node to reach the target destination. Client ends when the target has been reached. If the Gazebo has just been loaded, a sample client command that works would be: `ros2 run lab3 client 1 -0.9 3 3 `

For the first run, the controller node will generate the map and the metadata as `ScaledMap.png` and `scaledmap.json`. These can be loaded in by the controller to run the controller without relying on RViz. After you ran the above steps, to run in offline map mode:

1. In `lab3/lab3/kp_controller.py` ln 16, change to `OFFLINE_MAP=True`
2. Ensure that on the same file, ln 61 and 72, it points to the correct path of the metadata and ogm image
3. Re run `colcon build --packages-select lab3` from the ros2 workspace

