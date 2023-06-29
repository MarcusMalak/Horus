# DATASET CREATION CARLA
The scripts in this folder are used to export the dataset from Carla into the nuScenes format. 

Please refer to the Carla documentation first to install the simulation: https://carla.readthedocs.io/

Also see nuScenes documentation for more info on dataset format: https://www.nuscenes.org/nuscenes

## How to use
All sensor and data export configuration can be modified in main_export.py.

- Run main_export.py
- Run data_blocks/utils/update_log_fix.py (quick fix for bug with log.json)
- Run get_lidar_num.py (Adds number of lidar points to annotations)

## Tips
- Make sure to modify splits.py file the nuScenes library as the scene numbers for the train and val set are hardcoded.
