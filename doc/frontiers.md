
## Frontier detection

### Legend
| Color | Occupancy | Value in grid | Value in img |
| -------- | -------- | -------- | -------- |
| white | Free | 0 | 255 |
| black | Obstacle | 100 | 0 |
| gray | Unknown | -1 | 128 |

--- 

Source 

![Source image](frontiers/map.png)

### Edges

1. Binarization with unknown space as obstacles
2. Erode to avoid get centroid on impassable cell
3. Canny over binary image

![Binary unk as obs](frontiers/unk_obs.png)
![Binary unk as obs eroded](frontiers/unk_obs_eroded.png)
![edges](frontiers/edges.png)

### Frontiers

1. Binarization with unknown space as free space
2. Adding drone mask to obstacle map
3. Eroding obstacles

![Binary unk as free](frontiers/unk_free.png)
![Obstacles](frontiers/obstacles.png)
![Obstacles eroded](frontiers/obs_eroded.png)

1. Bitwise AND operation between obstacles eroded and edges

![Obstacles eroded](frontiers/obs_eroded.png) + 
![edges](frontiers/edges.png) =
![frontiers](frontiers/frontiers.png)

### Segmentation of frontiers

`cv2.findContours()` followed by `cv2.moments()` don't work well for 1 pixel lines (polygons). Using `cv2.connectedComponents()` instead. Three frontiers detected in the example:

![mask 1](frontiers/mask1.png)
![mask 3](frontiers/mask3.png)
![mask 4](frontiers/mask4.png)
![mask 2](frontiers/mask2.png)

Frontiers are filtered by area value in pixels to discard noise or non-relevant frontiers (g.e., see frontiers 2 and 3).

Centroids are calculed with pixel of each frontiers.

![centroids](frontiers/centroids.png)
![centroids resized](frontiers/centroids_big.png)
