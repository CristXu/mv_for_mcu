import cv2 as cv 
import numpy as np 
import argparse 

if __name__ == "__main__":
    parser = argparse.ArgumentParser() 
    parser.add_argument('prefix', type=str)
    parser.add_argument('-t', '--img_type', default='jpg', type=str)
    parser.add_argument('-s', '--size', type=int)
    args, unknown = parser.parse_known_args() 

    prefix = args.prefix
    img_type = args.img_type
    new_size = args.size

    print(prefix, img_type)
    img = cv.imread("%s.%s"%(prefix, img_type))
    img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    if new_size:
        img = cv.resize(img, (new_size, new_size))
    cv.imshow("img", img)
    cv.waitKey(0)
    img_flat = img.flatten()

    binary = False
    col = 16#img.shape[1]
    string = '#include "stdint.h"\nuint8_t %s_grayscale[] = {\n\t'%prefix
    for i, value in enumerate(img_flat):
        if binary:
            value = 0 if value >= 240 else 1
        string += "%d, "%value
        if (i + 1) % col == 0:
            string += '\n\t'
    string += "};\n"
    with open("%s.c"%prefix, "w") as f:
        print(string, file=f)
        f.close()
