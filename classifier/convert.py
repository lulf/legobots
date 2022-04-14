from PIL import Image
import numpy as np
import os

def load_image(infilename):
    """This function loads an image into memory when you give it
       the path of the image
    """
    img = Image.open(infilename)
    img.load()
    data = np.asarray(img, dtype="float32")
    return data


def create_npy_from_image(images_folder, output_name, num_images, image_dim):
    """Loops through the images in a folder and saves all of them
       as a numpy array in output_name
    """
    image_matrix = np.empty((num_images, image_dim, image_dim, 3), dtype=np.float32)
    for i, filename in enumerate(os.listdir(images_folder)):
        if filename.endswith(".jpg"):
            data = load_image(images_folder + filename)
            image_matrix[i] = data
        else:
            continue
    np.save(output_name, image_matrix)


