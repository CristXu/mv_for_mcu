import argparse 
import numpy as np 
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', default = 'test.bin', type=str)
    args, unknown = parser.parse_known_args()
    f = args.file
    name = f.split('.')[0]

    data = np.fromfile(f, dtype='uint8')
    data_0 = data[0:len(data):2]
    data_1 = data[1:len(data):2]
    
    data_0 = data_0[..., np.newaxis]
    data_1 = data_1[..., np.newaxis]

    data_new = np.hstack([data_1, data_0]).ravel()
    data_new.tofile(name + '_swap.bin')