from PIL import Image

from numpy import asarray

# load the image

image = Image.open('00018178.png')

# convert image to numpy array

data = asarray(image)

print(data)
