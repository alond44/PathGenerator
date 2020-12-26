# Path Generator

## Introduction
In the following explanation we'll demonstrate how to use our Path Generating system in order to create paths for the drone over the city, adjust the different parameters to meet different flight requirements and get the outputed paths. After that we'll show some result generated using our code and explain the pros and cons of the two methods we implemented.

## How to Use
All of our code is inside a python class named PathGenerator. In order to use it you should:

### 0. Package Requirments and Imports.

If needed, a requirements.txt file is included and can be installed on any conda environment (for example) using:

`conda install --file requirements.txt`

Then, import the map creation function, the PathGenerator class and the other constant classes:

```python
from DSM_Paths.DsmParser import create_map
from DSM_Paths.path_generator import PathGenerator, PathType, ConstraintType
```
Now you're good to go!

### 1. Use the parser in order to create the dsm map.
The Parser is found in DsmParser.py and in order to run it you should use the following code:

```python
Inputpath = Path(__file__).parent.absolute()
FileName = 'dsm_binary'
_dsm = create_map(Inputpath, FileName)
```
Make sure your .bin file is under workingfolder/BinFiles/

#### Note:

The folder 'BinFiles' does not have to be inside your working folder. The other option is to keep the dsm binary file in a folder named 'BinFiles' in a different folder ('Files' for example) and pass that folder's absolute path as 'Inputpath' to 'create_map'.

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
#### DSM Load Alternative:

If you did not initiate the dsm  map in the constractor (passed `dsm=None`) you'll need to use the `init_map` method in order to use the instance you've created.

```python
def init_map(self, input_path=None, file_name=None, save_tif=False, pixel_dist=2.0)
```
This method essentially uses the `creat_map` function for you so the requirement concerning the BinFile folder still holds.

### 3. Change Map Resolution (Optional):

#### Zoom In

```python
def map_zoom_in(self, multiplier: int)
```

* multiplier: The enhance multiplier. Must be > 0. Passing a negative value will cause the method to have no effect.

We've created a way to get more specific with the drone's location by dividing each pixel to a few more so each pixel will represent a smalller real world tile with the same height value.

This method has some pluses and minuses consequences:

* Plus: The drone's location is more spacific (as every pixel represents a smaller real life area). 
* Plus: The difference between the given constraints and the outputed path's cost (distance or time) is smaller. 
* Minus: The computation time lengthen.

These effects will be demonstrated in the results section.


##### Illustration:

![alt text](https://github.com/alond44/PathGenerator/blob/main/Ilustrations/zoom_in_example.png "Zoom In Example")


#### Zoom Out 

```python
def map_zoom_out(self, multiplier: int)
```

The argument multiplier serves the same purpose as it does in the zoom in method. 
A call 'pg.map_zoom_out(x)' will reverse the effect of 'pg.map_zoom_in(x)' and will make no change to the map. However a zoom out method call does not have to come after a call to zoom in. It can be called indepentently and in that case the method will merge every x pixels to one with a height value of the maximum pixel in the pixel group that was merged.

This method's pluses and minuses are the opposite from those of the zoom in method.

##### Notes:
* Using zoom out with multiplier M is possible even when the dsm map dimensions are YxY and Y is not divisable by x.
* Using this method might cause information lose due to the max value pixel merge.


##### Illustration:

![alt text](https://github.com/alond44/PathGenerator/blob/main/Ilustrations/zoom_out_example.png "Zoom Out Example")


### 4. Creating Paths:

```python
def gen_paths(self, flag: ConstraintType, constraint: float, path_type: PathType, start_location=None, path_num=1, to_print=False, weight=1.0)
```

parameters:
* flag: Will hold either 'd' for distance constraint or 't' for time constraint.
* constraint: Either the paths' length in meters for distance constrained paths or the travel time for time in seconds constrained paths.
* path_type: 'prob' or 'a_star' as will be explain in more details later- 'prob' creates a simple path using random walk in the map while 'a_star' create path using weighted a* algorithm between some randome sampled points.
* start_location: The path's first point. If None is passed or the given point is not valid a random one will be generated.
* path_num: The number of paths wanted.
* to_print: Pass True to print the resulting paths over the dsm map.
* weight: Relevant only when using 'a_star'. weight should be >= 1.

Return value - A list of 'path_num' paths.

#### Usage Example

```python
pg.gen_paths(flag='d', constraint=1000, path_type='a_star', start_location=[150, 150], path_nums=1, to_print=True, weight=2)
```


### Extras

#### Path Printer

```python
def print_path(self, path=None, path_color='r', path_style='--')
```

This method gets a path and some optinal parameters that control the path's present style.
Passing a `None` value in the path argument results in printing the instance's dsm map only.

##### Usage Example
```python
pg.print_path(path=my_path)
```

#### Path Distance Calculator

```python
def calc_path_distance(self, path: list)
```
This method recieve a path and returns the path's distance on the instance's map.

#### Path Travel Time Calculator

```python
def calc_path_travel_time(self, path: list)
```

This method recieve a path and returns the path's travel duration on the instance's map.

## Our Results and Algorithm Explanation:

### Simple Example and Algorithm
#### Local Path- Probability Random Walk Path
The first kind of algorithm we have imlemented in order to create the path was a "random surfer".
At each point the agent decides which place to go from 8-ways opportunities by random, with the limitation of not going back from the same way that we came from (so in practice 7-ways selection).

<img src="https://github.com/alond44/PathGenerator/blob/main/Results/simple_example_Probabilistic.png" width="600">

As could be seen from the result, this algorithm created more "localy wandering" path that hasn't spread much over the city.

#### Extensive Path- Weighted A* Path
The second algorithm we have implemented was a weighted A* algorithm.
The Path was created by sampling random points in the map while calculating a optimal path (or suboptimal path bounded to weight*optimal_path) between those points.

<img src="https://github.com/alond44/PathGenerator/blob/main/Results/simple_example_Weighted_A_Star.png" width="600">

As could be seen from the result, this algorithm created more distributed path that explore much bigger parts of the city.
