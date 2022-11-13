## fastest_det_ros
Colconized [FastestDet](https://github.com/dog-qiuqiu/FastestDet) Python library for easier inference deployment on a ROS2 system.

## Installation

    pip install torch torchvision
    cd <your_colcon_ws>/src
    git clone --recursive https://github.com/grassjelly/fastest_det_ros
    cd ..
    rosdep install --from-paths src -i -r -y
    colcon build

## Usage

Import inference from fastest_det_ros and call the `predict` function.

    import cv2
    from fastest_det_ros.inference import FastestDet

    detector = FastestDet(
        weights=<path to .pth>,
        imsgz=[352,352], #height, width,
        thresh=0.5,
        device='gpu'
    )

    image = cv2.imread('test.jpg')
    classes, bounding_boxes, confidence = detector.predict(image)

## Troubleshooting Guide

- TLS Memory Block Error:

        export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/torch/lib/../../torch.libs/libgomp-d22c30c5.so.1.0.0

- Numpy Not Available

        pip install numpy --upgrade