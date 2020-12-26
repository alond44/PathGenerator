# Path Generator

## Introduction
In the following explanation we'll demonstrate how to use our Path Generating system in order to create paths for the drone over the city, adjust the different parameters to meet different flight requirements and get the outputed paths. After that we'll show some result generated using our code and explaing the pros and cons of the two methods we implemented.

## How to Use
All of our code is inside a python class named PathGenerator. In order to use it you should:

### 1. Use the parser in oreder to create the dsm map.
The Parser is found in DsmParser.py and in order to run it you should use the following code:

```python
Inputpath = Path(__file__).parent.absolute()
FileName = 'dsm_binary'
_dsm = create_map(Inputpath, FileName)
```
Make sure your .bin file is under workingfolder/BinFiles/

Note: The folder 'BinFiles' does not have to be inside your working folder. The other option is to keep the dsm binary file in a folder named 'BinFiles' in a different folder ('Files' for example) and pass that folder's absolute path as 'Inputpath' to 'create_map'.

### 2. Create an Instance of the PathGenerator:

```python
def __init__(self, velocity, flight_height, dsm=None, pixel_dist=2.0)
```

At this point you need to choose the flight parameters:
* velocity: The drone's velocity (in meters per second).
* flight_height: The flight's altitude (in meters).
* dsm: The dsm map we have created at step 1. Note we can pass the default value (`None`) and load the map using the `init_map` method after generating an instance.
* pixel_dist: The distance of a pixel's width in real life (in meters).

#### Usage Example

```python
pg = PathGenerator(velocity=50, flight_height=150, dsm=_dsm, pixel_dist=2)
```

### 3. Change Map Resolution (Optional):

#### Zoom In

```python
def map_zoom_in(self, multiplier: int)
```

* multiplier: The enhance multiplier. Must be > 0. Passing a negative value will cause the method to have no effect.

We've created a way to get more specific with the drone's location by dividing each pixel to a few more so each pixel will represent a smalller real world tile with the same height value.
For example: the first of the following images represents a single pixel and the second represents that same pixel after calling `pg.map_zoom_in(2)` resulting it to split into four different pixels that represent half of the width and fourth of the area.


![alt text](https://github.com/alond44/PathGenerator/blob/main/Ilustrations/single_pixel.png "Pixel Before Zoom In")


![alt text](https://github.com/alond44/PathGenerator/blob/main/Ilustrations/divided_pixel.png "Pixel After Zoom in")


This method has some pluses and minuses consequences:

* Plus: The drone's location is more spacific (as every pixel represents a smaller real life area). 
* Plus: The difference between the given constraints and the outputed path's cost (distance or time) is smaller. // TODO: check if affects on the error and show results. 
* Minus: The computation time lengthen. // TODO: check and show results.


#### Zoom Out 

```python
def map_zoom_out(self, multiplier: int)
```

The argument multiplier serves the same purpose as it does in the zoom in method. 
A call 'pg.map_zoom_out(x)' will reverse the effect of 'pg.map_zoom_in(x)' and will make no change to the map. However a zoom out method call does not have to come after a call to zoom in. It can be called indepentently and in that case the method will merge every x pixels to one with a height value of the maximum pixel in the pixel group that was merged.

This method's pluses and minuses are the opposite from those of the zoom in method.

Notes:
* Using zoom out with multiplier M is possible even when the dsm map dimensions are YxY and Y is not divisable by x.
* Using this method might cause information lose due to the max value pixel merge.


##### Illustration:

![alt text](https://github.com/alond44/PathGenerator/blob/main/Ilustrations/zoom_out_example.png "Zoom Out example")


### 4. Creating Paths:

```python
def gen_paths(self, flag, constrain, path_type, start_location=None, path_num=1, to_print=False, epsilon=1.0)
```

parameters:
* flag: Will hold either 'd' for distance constraint or 't' for time constraint.
* constrain: Either the paths' length in meters for distance constrained paths or the travel time for time in seconds constrained paths.
* path_type: 'prob' or 'a_star' as will be explain in more details later- 'prob' creates a simple path using random walk in the map while 'a_star' create path using weighted a* algorithm between some randome sampled points.
* start_location: The path's first point. If None is passed or the given point is not valid a random one will be generated.
* path_num: The number of paths wanted.
* to_print: Pass True to print the resulting paths over the dsm map.
* epsilon: Only when using 'a_star'. epsilon should be >= 1.

Return value - A list of 'path_num' paths.

#### Usage Example

```python
pg.gen_paths(flag='d', constrain=1000, path_type='a_star', start_location=[150, 150], path_nums=1, to_print=True, weight=2)
```


### Extra 



## Our Results and Algorithm Explanation:

### Simple- Random Walk Path
![alt text](https://github.com/alond44/PathGenerator/blob/main/Results/random_walk%20result.png "Random Walk Example Result")
