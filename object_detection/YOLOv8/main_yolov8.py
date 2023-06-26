from ultralytics import YOLO

# Load the model
model = YOLO('yolov8n.pt')
# model = YOLO("./runs/detect/CAM_DRONE_100/weights/best.pt")

if __name__ == '__main__':
   # Training.
   results = model.train(
      data=r'C:\Users\malak\Documents\MasterThesis\MasterThesis\datasets\mini_set_3_working\YOLO\CAM_DRONE_50\data\mini_set_3_yolo.yaml',
      imgsz=1280,
      epochs=20,
      batch=4,
      name='CAM_DRONE_50',
      amp=False
      # device='cpu'
   )

   # # validate results
   # model.val(data="mini_set_10_p1_yolo.yaml", conf=0.5, save=True)

   # results = model.predict(source='../generate_dataset/convert2yolo/mini_set_10_p1_yolo/test/images',
   #                         save=True)

