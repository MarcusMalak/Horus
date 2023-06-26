from PIL import Image
import os

current_dir = os.getcwd()
filepath = os.path.join(current_dir, 'image_testing', 'scene01_1678640209.jpg')


def compress(filepath):
    image = Image.open(filepath)
    image = image.convert('RGB')
    image_path_out = os.path.join(current_dir, 'image_testing', 'image_compressed_100.jpg')
    image.save(image_path_out, optimize=True, quality=100)
    return


compress(filepath)
