#author: Zhiquan Li

import sys
from matplotlib import pyplot as plt
import numpy as np
from lanemarkingdetection import (road_surface_detection, scan_line_generator, boundary_selection, intensity_selection, lane_marking_refinement, lane_marking_selection, lane_marking_generation )
from util import file_reader
import time

def main():
    if len(sys.argv) == 3:
        paths = [sys.argv[1],sys.argv[2]]
    else:
        print('please give point cloud file and vehicle trajectory')
        return
    plat, plon, palt, pit =  np.array(file_reader(paths[0])).T
    trajlat, trajlon, trajalt, trajit =  np.array(file_reader(paths[1])).T
    trajectory = np.array([[trajlat[0], trajlon[0]],[trajlat[-1], trajlon[-1]]])
    traj_direction = trajectory[1] - trajectory[0]
    
    new_road_points = road_surface_detection(plat, plon, palt, pit, trajectory)
    scan_line = scan_line_generator(new_road_points, trajectory)
    scan_line = boundary_selection(scan_line)
    intensity = intensity_selection(scan_line)
    refine_list = lane_marking_refinement(intensity, traj_direction)
    line_cluster = lane_marking_selection(refine_list, traj_direction)
    ls = lane_marking_generation(line_cluster)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(plat, plon, ',')
    for l in ls:
        ax.plot(l[0],l[1],color = 'black',linestyle = '-')
    plt.savefig('result.png')
    plt.show()

if __name__ == '__main__':
    start = time.time()
    main()  
    print('Time costs: ',time.time() - start)