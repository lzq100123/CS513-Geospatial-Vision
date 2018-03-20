## author: Zhiquan Li

import matplotlib.pyplot as plt
import numpy as np
from math import radians, cos, sin, asin, sqrt, exp, pi,degrees,atan
import csv
import sys

def getProbeInfo(path):
    pbinfo = []
    with open(path,'r') as f:
        reader = csv.reader(f)
        for row in reader:
            sid, dt, sc, lat, lon, alt, spd, hd = row
            dt = timeConverter(dt)
            tod = None
            lp = None
            linkid = None
            cpset = []
            mp = None
            pbinfo.append([np.array([float(lon),float(lat)]), [sid,dt,sc,alt,spd,hd], cpset, tod, lp,linkid,mp])
    preid = pbinfo[0][1][0]
    start = 0
    for indx in range(1,len(pbinfo)):
        curid = pbinfo[indx][1][0]
        if curid != preid:
            end = indx - 1
            pbinfo[start][3] = haversineDist(pbinfo[start][0],pbinfo[end][0])
            preid = curid
            start = indx
        if indx == len(pbinfo) - 1:
            end = indx
            pbinfo[start][3] = haversineDist(pbinfo[start][0],pbinfo[end][0])  
    return pbinfo 

def timeConverter(time):
    timesp = time.split()
    time = timesp[1]
    time = time.split(':')
    if timesp[2] == 'PM':
        time[0] = int(time[0]) + 12
    mins = int(time[0]) * 60
    secs = (mins + int(time[1])) * 60
    secs = secs + int(time[2])
    return secs 

def getLinkInfo(path):
    lkinfo = []
    with open(path,'r') as f:
        reader = csv.reader(f)
        for row in reader:
            lid, lens, dot, sp,slp = row[0],row[3],row[5],row[14],row[16]
            shape = []
            relv = None
            slop = None
            for idx, s in enumerate(sp.split('|')):
                lat, lon, elv = s.split('/')
                if idx == 0 and len(elv) != 0:
                    relv = float(elv)
                shape.append(np.array([float(lon), float(lat)]))
            if len(slp) != 0:
                count = 0
                sp = 0
                for s in slp.split('|'):
                    tmp = s.split('/')
                    sp += float(tmp[1])
                    count += 1
                slop = sp / count
            lkinfo.append([shape,[lid,lens,dot],relv,slop])    
    return lkinfo    


def haversineDist(p1, p2, r = 6371):
    lon1, lat1, lon2, lat2 = map(radians,[p1[0], p1[1], p2[0], p2[1]])
    dlon = lon1 - lon2
    dlat = lat1 - lat2
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    return c * r * 1000

def projPoint(p, lkp): # each point is narray.
    seg_vec = lkp[1] - lkp[0]
    seg_p_vec = p - lkp[0]
    f = np.dot(seg_vec, seg_p_vec)
    if f < 0: 
        return lkp[0]
    d = np.dot(seg_vec, seg_vec)
    if f > d:
        return lkp[1]
    f = f / d
    return lkp[0] + f * seg_vec

def getCandiPoint(p,lkinfo, lens, threshold = 20):
    candpoint = []
    lklist = []
    lkid = None
    for lk in lkinfo:
        min_dist = sys.maxsize
        cp = None
        pdist = haversineDist(p, lk[0][0])
        if pdist <= lens:
            for indx in range(len(lk[0]) - 1): # for each segment in the link
                lkps = [lk[0][indx],lk[0][indx + 1]]
                cpoint = projPoint(p,lkps)
                dist = haversineDist(p,cpoint)
                if dist < threshold and dist < min_dist:
                    min_dist = dist
                    cp = cpoint
            if cp is not None:
                candpoint.append(cp)
                lklist.append(lk[0])
                lkid = lk[1][0]
    return candpoint, lklist, lkid


def obsProb(p, cp, mu = 0, deta = 20):
    return (1 / (sqrt(2 * pi) * deta)) * exp((haversineDist(p, cp) - mu) ** 2 / (2 * deta ** 2))

def transProb(pdist, cp1, cp2):
    d = haversineDist(cp1,cp2)
    if d == 0:
        d = 1000
    return pdist / d

def findMatchedSeq(points):
    f = []
    pre = {}
    # find the first point with non empty candidate points set and initialization
    indx = 0
    while(len(points[indx][2]) == 0 and indx < len(points)):
        indx += 1
    if indx == len(points): # return none if all points have no candiate points set
        return None
    
    prepoint = points[indx]
    temp_prob = 1
    max_in_f = 0 #tracking the max value index in f
    for i, cp in enumerate(points[indx][2]): #initialize the candidate points in first point
        obp = obsProb(points[indx][0],cp)
        f.append((cp,obp))
        if f[max_in_f][1] < obp:
            max_in_f = i
    
    indx_f = 0
    t_indx = len(f) - 1
    #start with the second point
    for idx in range(indx, len(points)):
        lens = len(f)
        pdist = haversineDist(points[idx][0],prepoint[0])
        if len(points[idx][2]) == 0: #skip points with no candidate point
            continue
        for cp in points[idx][2]:
            max_val = -sys.maxsize - 1
            max_cand = None
            for i in range(indx_f, lens):
                tmp = f[i][1] + obsProb(points[idx][0], cp) * transProb(pdist, cp, f[i][0]) * temp_prob
                if tmp > max_val:
                    max_val = tmp
                    max_cand = f[i][0]
            pstr = ','.join([str(cp[0]),str(cp[1])])
            pre[pstr] = max_cand
            f.append((cp, max_val))
            t_indx += 1
            if f[max_in_f][1] < max_val:
                max_in_f = t_indx
        indx_f = lens
        prepoint = points[idx]
    c = f[max_in_f][0]
    rlist = []
    vname = ','.join([str(c[0]),str(c[1])])
    for p in points[: :-1]:
        if len(p[2]) != 0:
            rlist.append(c)
            p[6] = c
            vname = ','.join([str(c[0]), str(c[1])])
            c = pre[vname]
        
    rlist.reverse()
    return rlist


def STMatching(probes,lkinfo):
    start = 0
    end = 0
    sid = probes[0][1][0]
    mplist = []
    for p in probes:
        p[2], p[4], p[5]= getCandiPoint(p[0],lkinfo,probes[0][3])
        if end == len(probes) - 1:
            mplist.append(findMatchedSeq(probes[start:end + 1]))
            continue
        if sid != p[1][0]:
            sid = p[1][0]
            mplist.append(findMatchedSeq(probes[start:end]))
            start = end
        end += 1
    return mplist

def computeAndEvalSlope(pbinfo,lkinfo):
    slopedict = {}
    lkdict = {}
    rst = []
    for p in pbinfo:
        if p[5] not in slopedict.keys():
            slopedict[p[5]] = []
        if p[6] is not None:
            slopedict[p[5]].append((p[6],p[1][3]))
    for lk in lkinfo:
        if lk[1][0] not in lkdict:
            lkdict[lk[1][0]] = {'refnode':lk[0][0],'altitude':lk[2],'slope':lk[3]}
    for key in slopedict.keys():      
        if key is not None and lkdict[key]['altitude'] is not None and lkdict[key]['slope'] is not None:
            length = len(slopedict[key])
            sum = 0
            slope = lkdict[key]['slope']
            for mp, alt in slopedict[key]:
                d = haversineDist(mp, lkdict[key]['refnode'])
                if d == 0:
                    continue
                sum += degrees(atan((float(alt) - lkdict[key]['altitude']) / d))
            cslope = sum / length
            deviation = abs(cslope - slope)
            rst.append([key,cslope,slope,deviation])
    return rst

def writeToDisk(path,data):
    with open(path,'w') as f:
        writer = csv.writer(f)
        for dt in data:
            writer.writerow(dt)

def main():
    probefilepath = './probe_data_map_matching/Partition6467ProbePoints_test.csv'
    linkfilepath = './probe_data_map_matching/Partition6467LinkData.csv'
    output_slopeinfo = './slopeinfo.csv'
    output_matchedpoint_sequence = './output_matchedpoint_sequence.csv'
    print('Processing probe data...')
    pbinfo = getProbeInfo(probefilepath)
    print('Processing link data...')
    lkinfo = getLinkInfo(linkfilepath)
    print('Map matching...')
    m = STMatching(pbinfo,lkinfo)
    print('Map matching done!')
    writeToDisk(output_matchedpoint_sequence,m)
    print('matched points sequence file create complete!')
    print('slope computation and evaluation...')
    rst = computeAndEvalSlope(pbinfo,lkinfo)
    print('slope computation and evaluation done!')
    writeToDisk(output_slopeinfo,rst)
    print('slopeinfo file create complete!')

if __name__ == '__main__':
    main()