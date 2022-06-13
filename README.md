# Open Robotics Darknet ROS

This is a ROS 2 wrapper around [darknet](https://pjreddie.com/darknet), an open source neural network framework.

![Example image with bounding boxes created using darknet and the yolov3-tiny network](doc/example_darknet_yolov3-tiny.png)

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone --recurse-submodules https://github.com/mgonzs13/openrobotics_darknet_ros.git
$ cd ~/ros2_ws
$ rosdep install --from-paths src -r -y --ignore-src
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select darknet_vendor # -DDARKNET_CUDA=Off -DDARKNET_OPENCV=Off
$ colcon build
```

## DetectorNode

This node can run object detectors like [YOLOv3](https://pjreddie.com/darknet/yolo/) on images.

### Subscribers

- `images` (type `sensor_msgs/msg/Image`) - Input mages to feed to the detector

### Publishers

- `detections` (type `vision_msgs/msg/Detection2DArray`) - Objects detected in an image (if any)

### Services

- `enable` (type `std_srvs::srv::SetBool`) - Enable/disable darknet detector

### Parameters

- `network.config` - a path to a file describing a darknet detector network
- `network.weights` - a path to a file with weights for the given network
- `network.class_names` - a path to a file with names of classes the network can detect (1 per line)
- `detection.threshold` - Minimum probability of a detection to be published
- `detection.nms_threshold` - Non-maximal Suppression threshold - controls filtering of overlapping boxes

## Example

Download `YOLOv3-tiny`.

```shell
$ wget https://raw.githubusercontent.com/pjreddie/darknet/f86901f6177dfc6116360a13cc06ab680e0c86b0/cfg/yolov3-tiny.cfg
$ wget https://pjreddie.com/media/files/yolov3-tiny.weights
$ wget https://raw.githubusercontent.com/pjreddie/darknet/c6afc7ff1499fbbe64069e1843d7929bd7ae2eaa/data/coco.names
```

### Node

Save the following as `detector_node_params.yaml`

```yaml
/**:
  ros__parameters:
    network:
      config: "./yolov3-tiny.cfg"
      weights: "./yolov3-tiny.weights"
      class_names: "./coco.names"
    detection:
      enable: true
      threshold: 0.25
      nms_threshold: 0.45
```

```shell
$ ros2 run darknet_ros detector_node --ros-args --params-file detector_node_params.yaml
```

### Launch

```shell
$ ros2 launch darknet_bringup darknet.launch.py network_config:=yolov3-tiny.cfg weights:=yolov3-tiny.weights class_names:=coco.names
```
