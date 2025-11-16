# Setup

1. (Optional) If you are planning on training the YOLO model, copy `.roboflow_key.example` to `.roboflow_key`, get your Roboflow key and write it into the file
2. Get camera calibration and parameter files and put them into `config/cam_params.yaml` and `config/camera_info.yaml`. This can be done in 2 ways:
- Copy over the templates `config/cam_params_template.yaml` and `config/camera_info_template.yaml`. This is done by default in `install.py`.
- Calibrate the camera by yourself (See the section below).

## TODO: add documentation for how to calibrate the camera
TODO: calibrate camera
TODO: optimise params

# Usage
## Running the nodes
`board_detection` node doesn't use any extra parameters: just do `ros2 run computer_vision board_detection`

## Debugging
To start the camera node directly, run `ros2 run usb_cam usb_cam_node_exe --ros-args --params-file config/cam_params.yaml`

To simulate a camera with an image, run `ros2 run stub_scripts compressed_image_publisher --ros-args -p input_path:="test/test_images/chessboards/1.jpeg"`, where the image path can be replaced for other images

To view camera output, run `rqt`, select "Plugins/visualization/Image view" and select the correct topic
