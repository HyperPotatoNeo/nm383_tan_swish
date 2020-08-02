import os
import cv2
import numpy as np

ROOT_PATH = "."
DATA_PATH = "data/test_cases/"

class MapFiller:

    def fill(self, img, id):
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

        
        # cv2.imwrite("dilate"+str(id)+".png", im_th)
        # for row in range(h):
        #     if im_th[row, 0] == 255:
        #         cv2.floodFill(im_th, None, (0, row), 0)
        #     if im_th[row, w-1] == 255:
        #         cv2.floodFill(im_th, None, (w-1, row), 0)
        # cv2.imwrite("rowflood"+str(id)+".png", im_th)
        # for col in range(w):
        #     if im_th[0, col] == 255:
        #         cv2.floodFill(im_th, None, (col, 0), 0)
        #     if im_th[h-1, col] == 255:
        #         cv2.floodFill(im_th, None, (col, h-1), 0)
        
        # holes = im_th.copy()
        # cv2.floodFill(holes, None, (0, 0), 255)

        # holes = cv2.bitwise_not(holes)
        # im_th = cv2.bitwise_or(im_th, holes)

        cv2.imwrite("./outputs/Contours"+str(id)+".png", im_floodfill)

        

def main():
    filler = MapFiller()
    counter = 0
    for x in os.walk(os.path.join(ROOT_PATH, DATA_PATH)):
        if os.path.exists(os.path.join(x[0], "map.png")):
            print(os.path.join(x[0], "map.png"))
            img = cv2.imread(os.path.join(x[0], "map.png"), cv2.IMREAD_GRAYSCALE)
            counter += 1
            filler.fill(img, counter)
            


if __name__ == "__main__" :
    main()