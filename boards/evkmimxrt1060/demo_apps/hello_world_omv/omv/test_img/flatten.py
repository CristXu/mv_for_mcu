import cv2 as cv 
import numpy as np 

img = cv.imread("rects.jpg")
img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
cv.imshow("img", img)
cv.waitKey(0)
img_flat = img.flatten()

binary = False
col = 16#img.shape[1]
string = '#include "stdint.h"\nuint8_t rect_grayscale[] = {\n\t'
for i, value in enumerate(img_flat):
    if binary:
        value = 0 if value >= 240 else 1
    string += "%d, "%value
    if (i + 1) % col == 0:
        string += '\n\t'
string += "};\n"
with open("rects.c", "w") as f:
    print(string, file=f)
    f.close()
