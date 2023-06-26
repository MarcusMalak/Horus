# from tqdm import tqdm
import numpy as np
import time
import subprocess
import os
import sys
import glob
import signal
import win32api
import win32process
import psutil

# name='CarlaUE4-Win64-Shipping.exe'

carla_path = os.path.normpath(os.path.join(os.getcwd(), "../../Carla/CarlaUE4.exe"))
process = subprocess.Popen(carla_path)
time.sleep(15)

# subprocess.Popen("taskkill /f /im/ CarlaUE4-Win64-Shipping.exe")

for proc in psutil.process_iter():
    if "CarlaUE4-Win64-Shipping.exe" in proc.name():
        print('terminating')
        proc.terminate()
        break
process.wait()



# process = win32process.CreateProcess(
#     carla_path,
#     None, None, None, 0, win32process.CREATE_NO_WINDOW, None, None, win32process.STARTUPINFO())
#
# time.sleep(15)
#
# process_handle = process[0]
# process_id = process[2]
#
# win32api.TerminateProcess(process_handle, -1)
# win32api.CloseHandle(process_handle)


# scenes = {
#     'scene1': 1,
#     'scene2': 2,
#     'scene3': 3,
#     'scene4': 4,
#     'scene5': 5,
#     'scene6': 6,
#     'scene7': 7,
#     'scene8': 8,
#     'scene9': 9,
#     'scene10': 10
# }
# outer_pbar = tqdm(total=len(scenes), desc='Scenes', position=0)
#
# for scene, num in scenes.items():
#     inner_pbar = tqdm(total=10, desc=f'{scene} frame', position=1, leave=True)
#     # print(f'\nNEW SCENE: {scene}')
#     clock = 0
#
#     while True:
#         clock += 1
#         inner_pbar.update(1)
#         time.sleep(2)
#         if clock == 10:
#             inner_pbar.close()
#             print(f'done')
#             break
#     outer_pbar.update(1)
#
# outer_pbar.close()
