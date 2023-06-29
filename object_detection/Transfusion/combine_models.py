import torch

img_backbone = "/home/marcus/MasterThesis/TransFusion/mask_rcnn_r50_nuim.pth"
lidar_backbone = "/home/marcus/MasterThesis/TransFusion/checkpoints/transfusion_L_horus300_ep20.pth"

img = torch.load(img_backbone, map_location='cpu')
pts = torch.load(lidar_backbone, map_location='cpu')
new_model = {"state_dict": pts["state_dict"]}
for k,v in img["state_dict"].items():
    if 'backbone' in k or 'neck' in k:
        new_model["state_dict"]['img_'+k] = v
        print('img_'+k)
torch.save(new_model, "fusion_model_horus300_rcnn_50.pth")