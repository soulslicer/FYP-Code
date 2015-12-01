import json
import numpy as np
import cv2

import rospkg
rospack = rospkg.RosPack()
path = rospack.get_path('sonar')

def cosd(angle):
    return np.cos(np.radians(angle))

def sind(angle):
    return np.sin(np.radians(angle))

def sonar_xym_map(xm, ym):
    MapX = int(round(xm*100,0))+520
    MapY = int(round(ym*100,0))
    if MapX < 0 or MapX >=1600 or MapY < 0 or MapY >=1600:
        return 0
    return sonar_xym[MapX][MapY]

def sonar_xym_test(MapX, MapY):
    return sonar_xym[MapX][MapY]

def sonar_pixel_map(x, y):
    return sonar_pixel[x][y]

"""""""""""""""""""""""""""""""""
Data generation
"""""""""""""""""""""""""""""""""

# # Generate sonar pixel map
# sonar_pixel = [[0 for y in xrange(480)] for x in xrange(394)]
# with open("/Users/user/git/bbauv/src/sensors/sonar/scripts/sonar_pixel-394-480.txt", "r") as f:
#   for line in f:
#     line_arr = line.split()
#     sonar_pixel[int(line_arr[0])][int(line_arr[1])] = (float(line_arr[2]),float(line_arr[3]))

# # Generate XY in metres to pixel map
# sonar_xym = [[0 for y in xrange(1600)] for x in xrange(1600)]
# for x in range(0,394):
#     for y in range(0,480):
#         RTHeta = sonar_pixel[x][y]
#         R = RTHeta[0]
#         Theta = RTHeta[1]
#         YM = R*cosd(Theta)
#         XM = R*sind(Theta)
#         MapX = int(round(XM*100,0))+520
#         MapY = int(round(YM*100,0))
#         sonar_xym[MapX][MapY] = [x,y]

# start = 0
# end = 1600
# sonar_xym_filled = [[0 for y in xrange(end)] for x in xrange(end)]
# for MapX in range(start,end):
#     for MapY in range(start,end):
#         # If the point has hole
#         if sonar_xym[MapX][MapY] == 0:
#             # Try in the x direction
#             for i,j in [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1),
#                         (-2,0),(-2,-2),(0,-2),(2,-2),(2,0),(2,2),(0,2),(-2,2),
#                         (-3,0),(-3,-3),(0,-3),(3,-3),(3,0),(3,3),(0,3),(-3,3),
#                         (-4,0),(-4,-4),(0,-4),(4,-4),(4,0),(4,4),(0,4),(-4,4),
#                         (-5,0),(-5,-5),(0,-5),(5,-5),(5,0),(5,5),(0,5),(-5,5),
#                         (-6,0),(-6,-6),(0,-6),(6,-6),(6,0),(6,6),(0,6),(-6,6),
#                         ]:
#                 # If get a hit, assign then break
#                 if MapX+i < start or MapX+i >= end:
#                     continue
#                 if MapY+j < start or MapY+j >= end:
#                     continue
#                 test_map = sonar_xym[MapX+i][MapY+j]
#                 if test_map != 0:
#                     sonar_xym_filled[MapX][MapY] = test_map
#                     break
#         else:
#             sonar_xym_filled[MapX][MapY] = sonar_xym[MapX][MapY]

# test = np.asmatrix(sonar_xym_filled)
# cv2.imshow("a",test*1000)
# while(1):
#     cv2.waitKey(15)

# # Save to file
# data = {
#   'sonar_xym': sonar_xym_filled,
#   'sonar_pixel': sonar_pixel
# }
# with open('sonar_mapping.txt', 'w') as outfile:
#     json.dump(data, outfile)

"""""""""""""""""""""""""""""""""
Data loading
"""""""""""""""""""""""""""""""""

with open(path + '/scripts/sonar_mapping.txt') as data_file:    
    data = json.load(data_file)
sonar_pixel = data['sonar_pixel']
sonar_xym = data['sonar_xym']

# R = 14.841354
# Theta = 16.855444 
# YM = R*cosd(Theta)
# XM = R*sind(Theta)
# print sonar_xym_map(XM,YM)
# print sonar_pixel_map(360,12)
