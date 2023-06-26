import cv2
import numpy as np
import os
import glob

current_dir = os.getcwd()
filepath = os.path.join(current_dir, 'data_output', 'sweeps', 'CAM_DRONE_200', '*.jpg')

img_arr = []

for filename in glob.glob(filepath):
    img = cv2.imread(filename)
    h, w, layers = img.shape
    size = (w, h)
    img_arr.append(img)

out = cv2.VideoWriter('compression_test.avi', 0, 1, size)

for i in range(len(img_arr)):
    out.write(img_arr[i])
out.release()

