# -*- coding: utf-8 -*-
import pdb
import cv2
import glob
import time
import numpy as np

import orbslam

data_path = "./extern/ORB_SLAM3/data/MH_01_easy"
timestamp_path = "./extern/ORB_SLAM3/Examples/Monocular/EuRoC_TimeStamps/MH01.txt"
data_name = "dataset-MH01_mono"

images = []
for filename in glob.glob(data_path + "/mav0/cam0/data/*"):
    # images.append(cv2.imread(filename, cv2.IMREAD_UNCHANGED))
    images.append(cv2.imread(filename))

timestamps = np.loadtxt(timestamp_path)
timestamps = timestamps / 1e9
nImages = len(timestamps)
timesTrack = []

s = orbslam.System("extern/ORB_SLAM3/Vocabulary/ORBvoc.txt",
                   "extern/ORB_SLAM3/Examples/Monocular/EuRoC.yaml")

for i in range(nImages):
    timesTrack.append(s.step(images[i], timestamps[i]))
    # TODO 时间控制
