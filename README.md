# Horus

## Introduction
This repository introduces the Horus dataset, a novel dataset designed specifically for evaluating the integration of drones into autonomous vehicle (AV) pipelines for object detection tasks. With the absence of suitable datasets for this scenario, the Horus dataset aims to fill this gap and provide a valuable resource for researchers and developers in the field. Leveraging the Carla simulation environment, Horus offers a cost-effective alternative to real-world data acquisition, enabling comprehensive evaluations.

## Dataset Overview
The Horus dataset comsists of 500 scenes that cover a wide range of weather conditions and feature diverse object categories. It includes sensor data captured by a lidar sensor mounted on a vehicle and cameras mounted on drones at different heights. Additionally, depth cameras on both the ego vehicle and drones assess object visibility within the scenes. This rich dataset facilitates the evaluation of incorporating drones into object detection, specifically addressing the challenge of sensor fusion between the drone and vehicle from various perspectives.

## Key Findings
Experimental results demonstrate the effectiveness of the approaches explored in the dataset. By integrating the drone camera into the AV pipeline, significant improvements in object detection performance are observed compared to the baseline system relying solely on lidar. The unobstructed birds-eye-view provided by the drone camera proves highly valuable, enabling the detection of vehicles at distances of up to 400m. However, limitations in pedestrian detection start to emerge above a height of 50m. These findings highlight the potential benefits of incorporating drones in AV pipelines, enhancing object detection efficiency and accuracy in real-world applications.

## Paper
[link to paper?]

![drone_heights](https://github.com/MarcusMalak/Horus/assets/94542107/8b8cff28-4f2a-4342-bf41-8adf0db58d46)
