This directory contains the collision data at the shoulder, generated with FCL and the Solo12 latest URDF model at different resolutions.
You can read the data with 
```python 
col_map = np.load(col_map_flename, allow_pickle=True)
```

Each of the files contains the collision distance (spatial : files with name 'map', DoF space : files with name 'distance') between the FL_upper leg link and the base link, as a function of the x and y rotations of the shoulder.
For a given resolution, 
```python 
col_map[i,j]
``` 
corresponds to the shoulder configuration (rot_x = - PI + 2PI*(i/res), rot_y = - PI + 2PI *(j/res))
