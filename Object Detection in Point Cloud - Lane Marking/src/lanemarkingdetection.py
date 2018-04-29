#author: Zhiquan Li
from math import (radians, cos, acos, sin, asin, sqrt, exp, pi,degrees,atan)
import numpy as np
from util import (haversine_dist, proj_point, angle_caculation, hough_transform, RANSAC)
from sklearn import linear_model


#discard points far away to road
#get elevation points for road elevation evaluation
def road_surface_detection(plat, plon, palt, pit, trajectory, threshold_road = 20, threshold_traj = 5):
    road_points = []
    new_road_points = []
    elevation_points = []
    scan_data = {}
    for i in range(0,len(plat)):
        point = np.array([plat[i], plon[i]])
        point_on_traj = proj_point(point, trajectory)
        if len(point_on_traj) != 0:
            dis = round(haversine_dist(point, point_on_traj),2)
            if dis < threshold_road: # select point near by road
                road_points.append([plat[i],plon[i],palt[i],pit[i]])  
            if dis < threshold_traj: #select points for average elevation calculation
                elevation_points.append([plat[i],plon[i],palt[i],pit[i]])          
    xs,ys,zs,i = np.array(road_points).T
    x,y,z,fi = np.array(elevation_points).T
    e0 = np.sum(z) / len(z) # average elevation
    # select points which belongs to road elevation
    for j in range(0,len(zs)):
        if zs[j] > e0 - 0.3 and zs[j] <= e0 + 0.3:
            new_road_points.append([xs[j],ys[j],zs[j],i[j]])
    return new_road_points

# generated scan line
def scan_line_generator(new_road_points, trajectory):
    scan_line= {}
    for p in new_road_points:
        point = np.array([p[0], p[1]])
        point_on_traj = proj_point(point, trajectory)
        dis = round(haversine_dist(point,trajectory[0]),2)
        vx = trajectory[1] - trajectory[0]
        vy = point - trajectory[0]
        angle = angle_caculation(vx,vy)
        v_dis = round(haversine_dist(point, point_on_traj),2)
        if (point_on_traj[0] - point[0]) > 0: #right side
            if dis not in scan_line:
                scan_line[dis] = [([p[0],p[1],p[2],p[3]], v_dis, angle)]
            else:
                scan_line[dis].append(([p[0],p[1],p[2],p[3]], v_dis, angle))
        elif (point_on_traj[0] - point[0]) < 0:
            if -dis not in scan_line: #left side
                scan_line[-dis] = [([p[0],p[1],p[2],p[3]], v_dis, angle)]
            else:
                scan_line[-dis].append(([p[0],p[1],p[2],p[3]], v_dis, angle))
    return scan_line

#select road boundary
def boundary_selection(scan_line):
    for key in scan_line:
        lens = len(scan_line[key])
        val = -1
        if lens <= 6:
            scan_line[key] = sorted(scan_line[key],key = lambda x :x[2])
        else:
            for i in range(0,lens - 5):
                sum_elv = 0
                avg = 0
                sorted_scan_line = sorted(scan_line[key][i:i+10],key = lambda x :x[2])
                for sl in sorted_scan_line:
                    avg += sl[0][2]
                avg /= 6
                for sl in sorted_scan_line:
                    sum_elv += (sl[0][2] - avg) ** 2
                if val == -1:
                    val = sqrt(sum_elv /  6)
                elif val != sqrt(sum_elv /  6):
                    scan_line[key] = sorted_scan_line[:i + 5]
                    break
    return scan_line

#
def lane_marking_refinement(intensity, traj_direction):
    ist = hough_transform(intensity)
    hough_points = sorted(ist.items(), key = lambda d:d[1][0],reverse = True)[1:341]
    refine_list = []
    for hp in hough_points:
        line_direction = np.array(hp[1][1][len(hp[1][1]) - 1]) - np.array(hp[1][1][0])
        if angle_caculation(traj_direction, line_direction) < 5:
            refine_list.append(hp)
    #calculate mean of each line
    for ref in refine_list:
        mx, my = np.array(ref[1][1]).T
        mx = np.sum(mx) / len(mx)
        my = np.sum(my) / len(my)
        ref[1].append(np.array([mx,my]))
    return refine_list



# intensity selection
def intensity_selection(scan_line, threshold = 15):
    intensity = []
    for key in scan_line:
        for sl in scan_line[key]:
            if sl[0][3] > threshold:
                intensity.append([round(sl[0][0],6),round(sl[0][1],6)])
    return intensity

def RANSAC(X, Y):
    ransac = linear_model.RANSACRegressor()
    ransac.fit(X, Y)
    line_X = np.arange(X.min(), X.max(),(X.max()-X.min())/100)[:, np.newaxis]
    line_Y_ransac = ransac.predict(line_X)
    return [line_X, line_Y_ransac]

#refinement of candidate lane marking
#using distance between two lines and voting number
def lane_marking_selection(refine_list, traj_direction):
    line_cluster = []
    check = []
    for idx in range(0,len(refine_list)):
        line = []
        idx_line = refine_list[idx]
        if idx_line not in check:
            line.append(idx_line)
            check.append(idx_line)
            for indx in range(0,len(refine_list)):
                if indx == idx:
                    continue
                indx_line = refine_list[indx]
                if indx_line in check:
                    continue
                if idx_line[1][2][1] > indx_line[1][2][1]:
                    mean_vector = idx_line[1][2] - indx_line[1][2]
                else:
                    mean_vector = indx_line[1][2] - idx_line[1][2]
                dis = haversine_dist(idx_line[1][2],indx_line[1][2])
                angle = abs(angle_caculation(traj_direction, mean_vector))
                if angle < 5 or dis < 1:
                    line.append(indx_line)
                    check.append(indx_line)
            line_cluster.append(line)
    return line_cluster

# 
def lane_marking_generation(line_cluster, threshold = 8.241385):
    new_lc = []
    for lc in line_cluster:
        sorted_line = sorted(lc[0][1][1],key = lambda x :x[0])
        point = sorted_line[0]
        segments = []
        segment = [point]
        for idx in range(1, len(sorted_line)):
            dis = haversine_dist(np.array(sorted_line[idx]),np.array(point))
            if dis <= threshold:
                segment.append(sorted_line[idx])
            else:
                segments.append(segment)
                segment = [sorted_line[idx]]
            point = sorted_line[idx]
        segments.append(segment)
        new_lc.append(segments) 
    ls = []
    for nlc in new_lc:
        for nl in nlc:
            if len(nl) < 2:
                continue
            X, Y = np.array(nl).T
            X = np.array([[z] for z in X])
            ls.append(RANSAC(X, Y))
    return ls

