#!/usr/bin/env python
import os
import sys
import time
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]/'fastest_det' #Python repo
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import cv2
import torch
from utils.tool import *
from module.detector import Detector
from ament_index_python.packages import get_package_share_directory

lib_folder = get_package_share_directory('fastest_det_ros') + '/fastest_det_ros/fastest_det' 

class FastestDet:
    def __init__(
            self,
            weights=lib_folder + '/weights/weight_AP05:0.253207_280-epoch.pth',  # model.pt path(s)
            data=lib_folder + '/data/coco.data',
            imgsz=[352, 352],  # inference size (height, width)
            thresh=0.4,
            device='cpu'
    ):
        self.thresh = thresh
        self.img_size = imgsz

        if device == 'cpu':
            self.device = torch.device('cpu')
        else:
            if torch.cuda.is_available():
                self.device = torch.device('cuda')
            else:
                self.device = torch.device('cpu')     

        self.labels = []
        with open(lib_folder + '/configs/coco.names', 'r') as f:
            for line in f.readlines():
                self.labels.append(line.strip())
        
        self.model = Detector(
            len(self.labels), 
            True
        ).to(self.device)

        self.model.load_state_dict(torch.load(weights, map_location=self.device))

        self.config={}
        self.config['width'] = imgsz[1]
        self.config['height'] = imgsz[0]
    
        self.model.eval()
        
    def predict(self, im):
        res_img = cv2.resize(im, (self.config['width'], self.config['height']), interpolation = cv2.INTER_LINEAR) 
        img = res_img.reshape(1, self.config['height'], self.config['width'], 3)
        img = torch.from_numpy(img.transpose(0,3, 1, 2))
        img = img.to(self.device).float() / 255.0

        preds = self.model(img)
     
        output = handle_preds(preds, self.device, self.thresh)

        h, w, _ = im.shape
        scale_h, scale_w = h / self.config['height'], w / self.config['width']

        classes = []
        bounding_boxes = []
        confidence = []
        object_indices = []

        for box in output[0]:
            sub_box = []
            box = box.tolist()
            obj_score = box[4]
            obj_index = int(box[5])
            category = self.labels[obj_index]

            x1, y1 = int(box[0] * w), int(box[1] * h)
            x2, y2 = int(box[2] * w), int(box[3] * h)
            
            sub_box.append((x1, y1))
            sub_box.append((x2, y2))

            bounding_boxes.append(sub_box)
            confidence.append(obj_score)
            classes.append(category)
            object_indices.append(obj_index)

        return classes, object_indices, bounding_boxes, confidence

        

        