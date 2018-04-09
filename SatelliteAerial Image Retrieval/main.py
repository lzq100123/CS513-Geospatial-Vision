import sys
from tilesystem import (getLevelOfDetail, latLongToPixelXY, pixelXYToTileXY, imageConcatenation)

def main():
    #default parameter
    lat1 = 41.834594
    lon1 = -87.628887
    lat2 = 41.832947
    lon2 = -87.626451
    
    if len(sys.argv) != 5:
        print('run with default setting')
        print('Point1:(%f, %f), Point2:(%f, %f)' %(lat1, lon1, lat2, lon2))
    else:
        lat1, lon1 = float(sys.argv[1]), float(sys.argv[2])
        lat2, lon2 = float(sys.argv[3]), float(sys.argv[4])
    
    nlat1, nlon1 = max(lat1, lat2), min(lon1, lon2)  #northwest point
    nlat2, nlon2 = min(lat1, lat2), max(lon1, lon2)  #southeast point
    
    levelOfDetail = max(getLevelOfDetail(nlat1, nlon1),getLevelOfDetail(nlat2, nlon2))
    
    pixelX1, pixelY1 = latLongToPixelXY(nlat1,nlon1, levelOfDetail)
    pixelX2, pixelY2 = latLongToPixelXY(nlat2,nlon2, levelOfDetail)
    tileX1, tileY1 = pixelXYToTileXY(pixelX1, pixelY1)
    tileX2, tileY2 = pixelXYToTileXY(pixelX2, pixelY2)

    res_img = imageConcatenation(tileX1, tileY1, tileX2, tileY2, levelOfDetail)
    res_img = res_img.convert('RGB')
    res_img.save('./result.jpg')


if __name__ == '__main__':
    main()