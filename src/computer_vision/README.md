# Setup

1. (Optional) If you are planning on training the YOLO model, copy `.roboflow_key.example` to `.roboflow_key`, get your Roboflow key and write it into the file
2. Get camera calibration and parameter files from the [RobotArm repository](https://github.com/OxRAMSociety/RobotArm) or calibrate the camera by yourself. Put the params file into `computer_vision/computer_vision/cam_params.yaml`.

## TODO: add documentation for how to calibrate the camera
TODO: calibrate camera
TODO: optimise params

# Usage
## Debugging
To start the camera node directly, run `ros2 run usb_cam usb_cam_node_exe --ros-args --params-file cam_params.yaml`

To view camera output, run `rqt`, select "Plugins/visualization/Image view" and select the correct topic
