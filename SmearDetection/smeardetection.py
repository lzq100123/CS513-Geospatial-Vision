## author: Zhiquan Li

import numpy as np
import cv2
from os import (listdir,path,makedirs)


def smearDection(dir_path):
    folders = listdir(dir_path)
    output_folder = './result'
    if not path.exists(output_folder):
        makedirs(output_folder)
    para = {'cam_0':110,'cam_1':175,'cam_2':127,'cam_3':127,'cam_5':220} # threshold parameters
    for folder in folders: # process images in each cam folder
        bin_threshold = para[folder]
        new_path = dir_path + folder + '/'
        files = listdir(new_path)
        pre_img = cv2.imread(new_path + files[0])
        pre_img = cv2.cvtColor(pre_img,cv2.COLOR_BGR2GRAY)
        acc = np.zeros((pre_img.shape[0],pre_img.shape[1]),dtype = np.uint8)
        for indx in range(1,len(files),305): # a serial of images subtraction and different addition for each image
            cur_img = cv2.imread(new_path + files[indx])
            cur_img = cv2.cvtColor(cur_img,cv2.COLOR_BGR2GRAY)
            sub = cv2.subtract(pre_img,cur_img)
            acc = cv2.add(acc,sub)
            pre_img = cur_img
        ret,bin = cv2.threshold(acc,bin_threshold,255,cv2.THRESH_BINARY_INV) 
        kernel = np.ones((3,3),dtype = np.uint8)
        erode = cv2.erode(bin,kernel,iterations = 1)
        kernel = np.ones((3,3),dtype = np.uint8)
        dilate = cv2.dilate(erode, kernel, iterations = 1)
        _,contours, hierarchy = cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > 619 and cv2.contourArea(c) < 20000] # contours selection
        img = cv2.imread(new_path + files[100]) # result example
        _ = cv2.drawContours(img,contours,-1,(0,255,0),3) # mark smear in example image
        cv2.imwrite(output_folder + '/' + folder + '_' + files[0],img)

def main():
    dir_path = './sample_drive/'
    smearDection(dir_path)

if __name__ == '__main__':
    main()