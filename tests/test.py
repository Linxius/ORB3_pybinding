# -*- coding: utf-8 -*-

# import cmake_example as m

# assert m.__version__ == "0.0.1"
# assert m.add(1, 2) == 3
# assert m.subtract(1, 2) == -1

import cv2
import orbslam
import glob
import numpy as np
data_path = "extern/ORB_SLAM3/data/MH_01_easy"
timestamp_path = "extern/ORB_SLAM3/Examples/Monocular/EuRoC_TimeStamps/MH01.txt"
data_name = "dataset-MH01_mono"

s = orbslam.System("extern/ORB_SLAM3/Vocabulary/ORBvoc.txt",
                   "extern/ORB_SLAM3/Examples/Monocular/EuRoC.yaml")
i = cv2.imread('i.jpg')
r = s.step(i)
# images = []
# for filename in glob.glob(data_path + "/mav0/cam0/data"):
#     images.append(cv2.imread(filename))

# timestamps = np.loadtxt(timestamp_path)
# nImages = len(images)
# vTimesTrack = []
