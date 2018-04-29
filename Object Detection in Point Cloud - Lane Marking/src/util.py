#author: Zhiquan Li
from math import (radians, cos, acos, sin, asin, sqrt, exp, pi,degrees,atan)
import numpy as np
from sklearn import linear_model


def haversine_dist(p1, p2, r = 6371):
    lon1, lat1, lon2, lat2 = map(radians,[p1[0], p1[1], p2[0], p2[1]])
    dlon = lon1 - lon2
    dlat = lat1 - lat2
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    return c * r * 1000

def proj_point(p, traj): # each point is narray.
    seg_vec = traj[1] - traj[0]
    seg_p_vec = p - traj[0]
    f = np.dot(seg_vec, seg_p_vec)
    d = np.dot(seg_vec, seg_vec)
    if f < 0 or f > d:
        return np.array([])
    f = f / d
    return traj[0] + f * seg_vec


def file_reader(path):
    with open(path,'r') as f:
        data = []
        for line in f:
            data_list = line.split(" ")
            data.append([float(data_list[0]), float(data_list[1]), float(data_list[2]), float(data_list[3][:-1])])
    return data

def angle_caculation(x, y):
    lx = np.sqrt(x.dot(x))
    ly = np.sqrt(y.dot(y))
    cos_angle = x.dot(y) / (lx * ly)
    angle = np.arccos(cos_angle)
    return angle * 360 / 2 / np.pi

def hough_transform(points):
    size = len(points)
    inst = {}
    for idx in range(0,size):
        for indx in range(idx + 1, size):
            if points[idx][0] != points[indx][0]:
                intersect_point = []
                if (points[idx][1] - points[indx][1]) == 0:
                    theta = round(np.pi / 2,2) 
                    r = round(points[idx][1],2)
                    intersect_point = (theta,r)
                else:
                    theta = atan((points[indx][0] - points[idx][0]) / (points[idx][1] - points[indx][1]))
                    r = points[idx][0] * cos(theta) + points[idx][1] * sin(theta)
                    intersect_point = (theta,r)
                if intersect_point not in inst:
                    inst[intersect_point] = [1,[points[idx],points[indx]]]
                elif points[indx] not in inst[intersect_point][1]:
                    inst[intersect_point][0] += 1
                    inst[intersect_point][1].append(points[indx])
    return inst

def RANSAC(X, Y):
    ransac = linear_model.RANSACRegressor()
    ransac.fit(X, Y)
    line_X = np.arange(X.min(), X.max(),(X.max()-X.min())/100)[:, np.newaxis]
    line_Y_ransac = ransac.predict(line_X)
    return [line_X, line_Y_ransac]