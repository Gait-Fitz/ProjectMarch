import sys
from pathlib import Path
import pyrealsense2 as rs
import numpy as np
from matplotlib import cm
import cv2
import torch
import torchvision
from torch import nn
from torch import Tensor
from torchvision import transforms
from PIL import Image
from models.experimental import attempt_load
from utils.torch_utils import select_device

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = ROOT.relative_to(Path.cwd())

from detect import run

weights = ROOT / 'yolov5s.pt'
device = select_device('')
model = attempt_load(weights, map_location=device).eval()

# Define preprocess transformations
preprocess = transforms.Compose([
    # transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    frame_index = 0

    while True:
        # Wait for next color image
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        if True:
            pil_image = Image.fromarray(np.uint8(color_image))
            image = preprocess(pil_image).unsqueeze(0)
            with torch.no_grad():
                output = model(image)[0]
                print(output)
            exit()
            probabilities = torch.nn.functional.softmax(output[0], dim=0)

            # Show top categories per image
            top5_prob, top5_catid = torch.topk(probabilities, 1)
            for i in range(top5_prob.size(0)):
                print(categories[top5_catid[i]], top5_prob[i].item())
                # print(categories[top5_catid[i]])

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        # cv2.waitKey(1)
    
        frame_index += 1

finally:
    # Stop streaming
    pipeline.stop()



