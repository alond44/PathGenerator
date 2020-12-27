# Path Generator

## Introduction
In the following explanation we'll demonstrate how to use our Path Generating system in order to create paths for the drone over the city, adjust the different parameters to meet different flight requirements and get the outputed paths. After that we'll show some result generated using our code and explain the pros and cons of the two methods we implemented.

## How to Use
All of our code is inside a python class named PathGenerator. In order to use it you should follow the following steps.

### 0. Package Requirments and Imports.

If needed, a requirements.txt file is included and can be installed on any conda environment (for example) using:

`conda install --file requirements.txt`

Then, import the map creation function, the PathGenerator class and our constants classes:

```python
from DSM_Paths.DsmParser import create_map
from DSM_Paths.path_generator import PathGenerator, PathType, ConstraintType
```
Now that you're all set, let's get to the main usage steps.

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
#### DSM Loading Alternative:

If you did not initiate the dsm  map in the constractor (passed `dsm=None`) you'll need to use the `init_map` method before you can use the other PathGenerator methods.

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

We've created a way to get higher resolution of the drone's location by dividing each pixel to a few pixels such that each pixel will represent a smalller real world tile with the same height value.

This method has some advantages and disadvantages:

* Advantage: The drone's location is more spacific (as every pixel represents a smaller real life area). 
* Advantage: The difference between the given constraints and the outputed path's cost (distance or time) is smaller. 
* Disadvantage: The computation time becomes larger.

These effects will be demonstrated in the results section.


##### Illustration:

![alt text](https://github.com/alond44/PathGenerator/blob/main/Ilustrations/zoom_in_example.png "Zoom In Example")


#### Zoom Out 

```python
def map_zoom_out(self, multiplier: int)
```

The argument multiplier serves the same purpose as it does in the zoom in method. 
#TODO: sentence isn't clear
For this call: `pg.map_zoom_out(x)` this method will merge every group the map's pixel in to squares (with `x` pixel in each side) and then merge them in to a single pixel with a height value of the maximum pixel height value from the merged pixel group.
'pg.map_zoom_out(x)' call essentially reverse the effect of 'pg.map_zoom_in(x)' if one was called earlier (it revert the map back to it's state before the zoom in call). However, a zoom out method call does not have to come after a zoom in call and the map's side doesn't have to be divisable by x (refer to the illustration bellow for an example). 

This method's advantages and disadvantages are the opposite from those of the zoom in method.

##### Notes:
* Using zoom out with multiplier M is possible even when the dsm map dimensions are YxY and Y is not divisable by x.
* Using this method might cause information lose due to the max value pixel merge (lower resolution).


##### Illustration:

![alt text](https://github.com/alond44/PathGenerator/blob/main/Ilustrations/zoom_out_example.png "Zoom Out Example")


### 4. Creating Paths:

```python
def gen_paths(self, flag: ConstraintType, constraint: float, path_type: PathType, start_location=None, path_num=1, to_print=False, weight=1.0)
```

parameters:
* flag: Will hold either ConstraintType.DISTANCE for distance constraint or ConstraintType.TIME for time constraint.
* constraint: Either the paths' length in meters for distance constrained paths or the travel time for time in seconds constrained paths.
* path_type: The kind of path generating algorithm used. Will hold either PathType.AREA_EXPLORE for probability random path and PathType.MAP_ROAM for weighted A* generated path to random spots.
* start_location: The path's first point. If None is passed or the given point is not valid a random one will be generated.
* path_num: The number of paths wanted.
* to_print: Pass True to print the resulting paths over the dsm map.
* weight: Relevant only when using 'a_star'. weight should be >= 1.

Return value - A list of 'path_num' paths.

#### Usage Example

```python
pg.gen_paths(flag='d', constraint=1000, path_type='a_star', start_location=[150, 150], path_nums=1, to_print=True, weight=2)
```


### Extra Methods

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
This method receives a path and returns the path's distance on the instance's map.

#### Path Travel Time Calculator

```python
def calc_path_travel_time(self, path: list)
```

This method receives a path and returns the path's travel duration on the instance's map.

## Our Results and Algorithm Explanation:

### Algorithms and Simple Examples
#### Local Path- Probability Random Walk Path
The first kind of algorithm we have imlemented in order to create the path was a "random surfer".
At each point the agent decides which place to go from 8-ways opportunities by random, with the limitation of not going back from the same way that we came from (so in practice 7-ways selection).

<img src="https://github.com/alond44/PathGenerator/blob/main/Results/simple_example_Probabilistic.png" width="600">

As could be seen from the result, this algorithm created more "localy wandering" path that hasn't spread much over the city.

##### Method Call Example

```python
pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1500, path_type=PathType.AREA_EXPLORE, start_location=None, path_num=1, to_print=True, weight=2)
```

#### Extensive Path- Weighted A* Path
The second algorithm we have implemented was a weighted A* algorithm.
The Path was created by sampling random points in the map while calculating a optimal path (or suboptimal path bounded to weight*optimal_path) between those points.

<img src="https://github.com/alond44/PathGenerator/blob/main/Results/simple_example_Weighted_A_Star.png" width="600">

As could be seen from the result, this algorithm created more distributed path that explore much bigger parts of the city.

##### Method Call Example

```python
pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1500, path_type=PathType.MAP_ROAM, start_location=None, path_num=1, to_print=True, weight=2)
```

### Tests Results

The tests.py holds these function calls:

```python
    simple_example(pg)
    path_generating_error_test(pg, flag=ConstraintType.DISTANCE, desired_cost=2001, path_num=4)
    path_generating_error_test(pg, flag=ConstraintType.TIME, desired_cost=50, path_num=4)
    path_generating_calculation_time_test(pg, flag=ConstraintType.DISTANCE, desired_cost=2001, path_num=4)
    path_generating_calculation_time_test(pg, flag=ConstraintType.TIME, desired_cost=50, path_num=4)
```

We Used them to print the simple example we showed above and to test:
1. The average error of our algorithms (the difference between the given constraint and the resulted paths travel time or distance).
2. The average path calculation duration.
And how are both affected by the zoom in/out methods. 

The results weren't surprising.
The zoom in method caused longer calculation time but resulted in a much smaller as we passed a bigger multiplier.
On the flip side, zoom out had the opposite affect, the calculation time shortened but it resulted a bigger constraint error.
Furthermore, we can notice that the distance error in our Weighted-A* runs are dependent of the width a pixel represent in real life. Meaning an error can't be larger then the width of a pixel but we are limited to make 'pixel_dist' sized steps from one point to it's neighbor in Weighted-A* (we allow movement a pixel up, down, left and right - no diagonals).
The probabilistic paths aren't limited to 'pixel_dist' sized steps (as we allow diagonal moves to neighbors) so this point does not apply to them.

#### Notes
* We tested every combination of constraint type and path type (4 combinations) in both the error test and calculation time test.
* The tests outputs can be found as .png or as .txt files under the 'Results/Tests Outputs' folder. 

### Summary
In this project we have implemented 2 different algorithm for calculating path for drone while avoiding obstacles (calculated by the drone height).
We have shown in this document how we have tested our code and how to use it and the different results we have got from our experiments.
