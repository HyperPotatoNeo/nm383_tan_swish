import os
import cv2
import numpy as np
import yaml
import shapefile
import pngcanvas
import lxml
import xml.etree.ElementTree as ET
from bs4 import BeautifulSoup

# Open shapefile with Python Shapefile Library
shapefile_name = 'map' # e.g. england_oa_2001
r = shapefile.Reader(shapefile_name)

# Determine bounding box x and y distances and then calculate an xyratio
# that can be used to determine the size of the generated PNG file. A xyratio
# of greater than one means that PNG is to be a landscape type image whereas
# an xyratio of less than one means the PNG is to be a portrait type image.
xdist = r.bbox[2] - r.bbox[0]
ydist = r.bbox[3] - r.bbox[1]
xyratio = xdist/ydist
image_max_dimension = 600 # Change this to desired max dimension of generated PNG
if (xyratio >= 1):
    iwidth  = image_max_dimension
    iheight = int(image_max_dimension/xyratio)
else:
    iwidth  = int(image_max_dimension/xyratio)
    iheight = image_max_dimension

# Iterate through all the shapes within the shapefile and draw polyline
# representations of them onto the PNGCanvas before saving the resultant canvas
# as a PNG file
xratio = iwidth/xdist
yratio = iheight/ydist
pixels = []
c = pngcanvas.PNGCanvas(iwidth,iheight)
for shape in r.shapes():
    for x,y in shape.points:
        px = int(iwidth - ((r.bbox[2] - x) * xratio))
        py = int((r.bbox[3] - y) * yratio)
        pixels.append([px,py])
    c.polyline(pixels)
    pixels = []
f = file("%s.png" % shapefile_name,"wb")
f.write(c.dump())
f.close()

# Create a world file
wld = file("%s.pgw" % shapefile_name, "w")
wld.write("%s\n" % (xdist/iwidth))
wld.write("0.0\n")
wld.write("0.0\n")
wld.write("-%s\n" % (ydist/iheight))
wld.write("%s\n" % r.bbox[0])
wld.write("%s\n" % r.bbox[3])
wld.close



class MapFiller:

    def fill(self, img):
        image = img.copy()
        
        th, im_th = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((7,7), np.uint8) 
        im_th = cv2.dilate(im_th, kernel, iterations=1)
        
        h, w = im_th.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        im_floodfill = im_th.copy()
        cv2.floodFill(im_floodfill, mask, (0,0), 255)
        cv2.floodFill(im_floodfill, mask, (0, h-1), 255)
        cv2.floodFill(im_floodfill, mask, (w-1, 0), 255)
        cv2.floodFill(im_floodfill, mask, (w-1, h-1), 255)

        cv2.imwrite("result.png", im_floodfill)

        inverted_image = cv2.bitwise_not(im_floodfill)
        cv2.imwrite(os.path.expanduser("~/.gazebo/models/shape_map/materials/textures/shape_map.png"), inverted_image)

        model_path = os.path.expanduser("~/.gazebo/models/shape_map/model.sdf")

        tree = ET.parse(model_path)
        root = tree.getroot()

        for child in root.iter("size"):
            child.text = "{} {}".format(w/40, h/40)

        output_str = BeautifulSoup(ET.tostring(root), "xml").prettify()
        with open(model_path, "w") as f:
            f.write(output_str)



filler = MapFiller()
img = cv2.imread("map.png", cv2.IMREAD_GRAYSCALE)
filler.fill(img)

resolution = 0.02

doc = {
    'origin': [-img.shape[1]*resolution/2, -img.shape[0]*resolution/2, 0.0],
    'free_thresh': 0.196,
    'occupied_thresh': 0.65,
    'negate': 1,
    'image': 'result.png',
    'resolution': resolution
}

with open("map.yaml", "wr") as file:
    documents = yaml.dump(doc, file)

