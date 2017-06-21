#!/usr/bin/env python

import argparse
import cv2
import numpy as np
import matplotlib.pyplot as plt

def run_grabcut(img, rect):
    mask = np.zeros(img.shape[:2], np.uint8)
    bg_model = np.zeros((1,65), np.float64)
    fg_model = np.zeros((1,65), np.float64)
    cv2.grabCut(img, mask, rect, bg_model, fg_model, 5, cv2.GC_INIT_WITH_RECT)
    mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
    img = img * mask2[:,:,np.newaxis]
    plt.imshow(img)
    plt.colorbar()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-q', '--query', required=True)
    parser.add_argument('-r', '--rect', nargs='+', type=int, required=True)
    args = vars(parser.parse_args())

    # Get image
    img_name = args['query']
    img = cv2.imread(img_name)

    # Get rect
    rect = tuple(args["rect"])
    run_grabcut(img, rect)
    

