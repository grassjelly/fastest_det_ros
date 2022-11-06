#!/usr/bin/env python
import sys
import os
import cv2
import time
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]/'yolo_fastest'  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import torch
from model.detector import Detector
from utils.utils import load_datafile, handel_preds, non_max_suppression
from ament_index_python.packages import get_package_share_directory


lib_folder = get_package_share_directory('yolo_fastest_ros') + '/yolo_fastest_ros/yolo_fastest' 


class YoloFastest:
    def __init__(
            self,
            weights=lib_folder + '/modelzoo/coco2017-0.241078ap-model.pth',  # model.pt path(s)
            data=lib_folder + '/data/coco.data',
            imgsz=[352, 352],  # inference size (height, width)
            conf_thres=0.4,
            iou_thres=0.5,
            device=''
    ):
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.img_size = imgsz

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.labels = []
        with open(lib_folder + '/data/coco.names', 'r') as f:
            for line in f.readlines():
                self.labels.append(line.strip())
        
        self.model = Detector(
            len(self.labels), 
            3, 
            True
        ).to(self.device)

        self.model.load_state_dict(torch.load(weights, map_location=self.device))

        self.config={}
        self.config['width'] = imgsz[1]
        self.config['height'] = imgsz[0]
        self.config['anchor_num'] = 3
        self.config['anchors'] = [12.64, 19.39, 37.88, 51.48, 55.71, 138.31, 126.91, 78.23, 131.57, 214.55, 279.92, 258.87]

        #sets the module in eval node
        self.model.eval()
        
    def predict(self, im):
        res_img = cv2.resize(im, (self.config['width'], self.config['height']), interpolation = cv2.INTER_LINEAR) 
        img = res_img.reshape(1, self.config['height'], self.config['width'], 3)
        img = torch.from_numpy(img.transpose(0,3, 1, 2))
        img = img.to(self.device).float() / 255.0

        preds = self.model(img)
     
        output = handel_preds(preds, self.config, self.device)
        output_boxes = non_max_suppression(
            output, 
            conf_thres=self.conf_thres, 
            iou_thres=self.iou_thres
        )

        h, w, _ = res_img.shape
        scale_h, scale_w = h / self.config['height'], w / self.config['width']

        for box in output_boxes[0]:
            box = box.tolist()
        
            obj_score = box[4]
            category = self.labels[int(box[5])]

            x1, y1 = int(box[0] * scale_w), int(box[1] * scale_h)
            x2, y2 = int(box[2] * scale_w), int(box[3] * scale_h)
            print(x1)

        #     cv2.rectangle(ori_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        #     cv2.putText(ori_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)	
        #     cv2.putText(ori_img, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)

        # cv2.imwrite("test_result.png", ori_img)
        

        