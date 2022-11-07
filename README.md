## fastest_det_ros
Colconized [FastestDet](https://github.com/dog-qiuqiu/FastestDet) Python library for easier inference deployment on a ROS2 system.

## Installation

    pip install torch torchvision
    cd <your_colcon_ws>
    rosdep install --from-paths src -i -r -y
    colcon build

## Usage

Import inference from fastest_det_ros and call the `predict` function.
    import cv2
    from fastest_det_ros.inference import FastestDet

    model = FastestDet()

    image = cv2.imread('test.jpg')
    classes, bounding_boxes = model.predict(image)
