# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Validate a trained YOLOv5 model accuracy on a custom dataset

Usage:
    $ python path/to/val.py --data coco128.yaml --weights yolov5s.pt --img 640
"""

import argparse
import json
import os
import sys
from pathlib import Path
from threading import Thread

import numpy as np
import torch
from tqdm import tqdm

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = ROOT.relative_to(Path.cwd())  # relative

from models.yolo import Model
from models.experimental import attempt_load
from utils.datasets import create_dataloader
from utils.general import coco80_to_coco91_class, check_dataset, check_img_size, check_requirements, \
    check_suffix, check_yaml, box_iou, non_max_suppression, scale_coords, xyxy2xywh, xywh2xyxy, set_logging, \
    increment_path, colorstr, print_args
from utils.metrics import ap_per_class, ConfusionMatrix
from utils.plots import output_to_target, plot_images, plot_val_study
from utils.torch_utils import select_device, time_sync
from utils.callbacks import Callbacks
from utils.general import check_yaml, check_file
from PIL import Image
from torchvision import transforms

preprocess = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

class Inference:

    def __init__(self):
        self.load_model()

    def load_model(self):

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")  
        config = 'yolov5s.yaml'
        # configfile = check_file(config)
        # self.model = Model(configfile).to(self.device)
        weights = ROOT / 'yolov5s.pt'
        self.model = torch.load(weights)['model']
        self.model.to(self.device)
        self.model.float()
        self.model.eval()

    def predict(self, input):
        with torch.no_grad():
            prediction = self.model(input)
            return prediction
        
inference = Inference()

img = Image.open('/media/psf/Home/Downloads/Software_bois.jpg')
img = preprocess(img).unsqueeze(0)
prediction = inference.predict(img)
# print(prediction)



