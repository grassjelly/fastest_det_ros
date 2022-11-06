## yolo_fastest_ros
Colconized [YoloFastest](https://github.com/dog-qiuqiu/Yolo-Fastest) Python library for easier inference deployment on a ROS2 system.

## Installation

    pip install torch torchvision
    cd <your_colcon_ws>
    rosdep install --from-paths src -i -r -y
    colcon build

## Usage

Import inference from yolo_fastest_ros and call the `predict` function.
    import cv2
    from yolo_fastest_ros.inference import YoloFastest

    model = YoloFastest()

    image = cv2.imread('test.jpg')
    classes, bounding_boxes = model.predict(image)
