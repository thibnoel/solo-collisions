import numpy as np 
from scipy import ndimage
import matplotlib.pyplot as plt
from solo12_shoulder_collision_utils import followBoundary, colMapToDistField

# Load the collision map from file
res = 500
col_map_file = './npy_data/collision_map_centered_res{}.npy'.format(res)
col_map = np.load(col_map_file, allow_pickle=True)
col_map = col_map.T

dist_field_file = './npy_data/updated_collision_map_distance_res{}.npy'.format(res)
dist_field = np.load(dist_field_file, allow_pickle=True)
dist_field = dist_field - np.min(dist_field)

# Get x-gradient in "sx"
sx = ndimage.sobel(dist_field,axis=0,mode='wrap')
# Get y-gradient in "sy"
sy = ndimage.sobel(dist_field,axis=1,mode='wrap')
# Get square root of sum of squares
sobel=np.hypot(sx,sy)

plt.imshow(dist_field, cmap=plt.cm.RdYlGn)
plt.figure()
plt.imshow(sx, cmap=plt.cm.RdYlGn)
plt.figure()
plt.imshow(sy, cmap=plt.cm.RdYlGn)

plt.show()