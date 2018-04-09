import math
import numpy as np
import urllib.request
import json
from PIL import Image
from _io import BytesIO


API_KEY = 'Akp-ZwzWtyJWiu846tJV_U_GYHdD_XlFSSi90_zkqCnpH4cHnETF-GoQwpalvQ-V'
BaseUrl ='http://dev.virtualearth.net/REST/V1/Imagery/Metadata/Aerial/%f,%f?zl=%d&key=%s'

#constraint 
EarthRadius = 6378137
MinLatitude = -85.05112878
MaxLatitude = 85.05112878
MinLongitude = -180
MaxLongitude = 180

def clip(n, minValue, maxValue):
    return min(max(n, minValue), maxValue)

def getLevelOfDetail(lat, lon):
    lvd = 21
    is_empty = True
    while is_empty:
        url = BaseUrl %(lat, lon, lvd, API_KEY)
        res = json.loads(urllib.request.urlopen(url).read().decode('utf-8'))
        if res['resourceSets'][0]["resources"][0]["vintageStart"] == None:
            lvd -= 1
        else:
            is_empty = False
    return lvd

def mapSize(levelOfDetail):
    return 256 << levelOfDetail


def groundResolution(latitude, levelOfDetail):
    latitude = clip(latitude, MinLatitude, MaxLatitude)
    return math.cos(latitude * math.pi / 180) * 2 * math.pi * EarthRadius / mapSize(levelOfDetail)


def mapScale(latitude, levelOfDetail, screenDpi):
    return groundResolution(latitude, levelOfDetail) * screenDpi / 0.0254

def latLongToPixelXY(latitude, longitude, levelOfDetail):
    latitude = clip(latitude, MinLatitude, MaxLatitude)
    longitude = clip(longitude, MinLongitude, MaxLongitude)

    x = (longitude + 180) / 360
    sinLatitude = math.sin(latitude * math.pi / 180)
    y = 0.5 - math.log((1 + sinLatitude) / (1 - sinLatitude)) / (4 * math.pi)

    ms = mapSize(levelOfDetail)
    return int(clip(x * ms + 0.5, 0, ms - 1)), int(clip(y * ms + 0.5, 0, ms - 1))

def pixelXYToLatLong(pixelX, pixelY, levelOfDetail):
    ms = mapSize(levelOfDetail);
    x = (clip(pixelX, 0, ms - 1) / ms) - 0.5;
    y = 0.5 - (clip(pixelY, 0, ms - 1) / ms);

    return (90 - 360 * math.atan(math.exp(-y * 2 * math.pi)) / math.pi), 360 * x;


def pixelXYToTileXY(pixelX, pixelY):
    return int(pixelX / 256), int(pixelY / 256)


def tileXYToPixelXY(tileX, tileY):
    return tileX * 256, tileY * 256


def tileXYToQuadKey(tileX, tileY, levelOfDetail):
    string = ''
    while levelOfDetail > 0:
        digit = '0'
        levelOfDetail = levelOfDetail - 1
        mask = 1 << (levelOfDetail)
        if tileX & mask != 0:
            digit = chr(ord(digit) + 1)
        if tileY & mask != 0:
            digit = chr(ord(digit) + 2)
        string += digit
    
    return string

def quadKeyToTileXY(quadKey):
    tileX = 0
    tileY = 0
    levelOfDetail = len(quadKey)
    i = levelOfDetail
    while(i > 0):
        mask = 1 << (i - 1)
        key = quadKey[levelOfDetail - i]
        if key == '1':
            tileX |= mask
        elif key == '2':
            tileY |= mask
        elif key == '3':
            tileX |= mask
            tileY |= mask
        elif key != 0:
            raise ValueError('Invalid QuadKey digit sequence.')
        i -= 1
    
    return tileX, tileY


def getImage(quadKey):
    url = "http://ecn.t3.tiles.virtualearth.net/tiles/a%s.jpeg?g=6358" % quadKey
    return Image.open(BytesIO(urllib.request.urlopen(url).read()))


def imageConcatenation(tileX1, tileY1, tileX2, tileY2, levelOfDetail):
    pixelX1, pixelY1 = tileXYToPixelXY(tileX1, tileY1)
    pixelX2, pixelY2 = tileXYToPixelXY(tileX2, tileY2)
    
    # create a blank image container for retrieved images
    imw = int(abs(pixelX1 - pixelX2))
    imh = int(abs(pixelY1 - pixelY2))
    color = (255,255,255,0)
    result_img = Image.new('RGBA', (imw, imh), color)
    
    # get color of empty retrieved image for comparision
    empty_color = getImage('000000000000000000000').getcolors()

    i = 0
    for ty in range(tileY1, tileY2 + 1):
        j = 0
        for tx in range(tileX1, tileX2 + 1):
            quadKey = tileXYToQuadKey(tx, ty, levelOfDetail)
            img = getImage(quadKey)
            
            # decrease levelOfDetail if image is empty
            while img.getcolors() == empty_color:
                quadKey = quadKey[:-1]
                img = getImage(quadKey)
            
            #add qualified image to result
            result_img.paste(img, (j, i))
            img.close()
            j += 256
        i += 256
    return result_img




