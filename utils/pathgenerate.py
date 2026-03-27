import pickle
import numpy as np


def pkl2npy(pkl_path, npy_path):
    egoPath = pickle.load(open(pkl_path, "rb"))
    lst = list(map(lambda x:x[:2],egoPath))
    l1 = list(map(lambda x:x[0],lst))
    l2 = list(map(lambda x:x[1],lst))
    matrix = np.array([l1,l2])
    np.save(npy_path, matrix)