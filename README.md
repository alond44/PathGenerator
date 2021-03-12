# Path Generator

## Introduction
In the following README file we'll demonstrate how to use our Path Generating system in order to: 
* Create paths for the drone over the city. 
* Adjust the different parameters to meet different flight requirements 
* Get the outputed paths.
After that, we'll show some result generated using our code and explain the pros and cons of the two methods we implemented.

## How to Use
All of our code is inside a python class named PathGenerator. In order to use it you should follow the steps below.

### 0. Package Requirments and Imports.

If needed, a requirements.txt file is included and can be installed on any conda environment (for example) using:

`conda install --file requirements.txt`

After adapting your environment, import the map creation function, the PathGenerator class and our constants classes:

```python
from DSM_Paths.DsmParser import DSMParcer
from DSM_Paths.path_generator import PathGenerator, PathType, ConstraintType
```
Now that you're all set, let's get to the main usage steps.

### 1. Use the parser in order to create the dsm map.
The Parser is found in DsmParser.py and in order to run it you should use the following code:

```python
Inputpath = Path(__file__).parent.absolute()
FileName = 'dsm_binary'
_, _, _, x_org, y_org, z_org, Wx, Wy, dWx, dWy, dsm = DSMParcer(Inputpath, FileName, False)
```
Make sure your .bin file is under workingfolder/BinFiles/

#### Note:

The folder 'BinFiles' does not have to be inside your working folder. The other option is to keep the dsm binary file in a folder named 'BinFiles' in a different folder ('Files' for example) and pass that folder's absolute path as 'Inputpath' to 'create_map'.

### 2. Create an Instance of the PathGenerator:

```python
def __init__(self, velocity, flight_height, dsm=None, origin=(0.0, 0.0, 0.0), map_dimensions=(0, 0), pixel_dimensions=(0.0, 0.0), stride_multiplier=1.0, max_angle=45.0)
```

At this point you need to choose the flight parameters:
* velocity: The drone's velocity (in meters per second).
* flight_height: The flight's altitude (in meters).
* dsm: The dsm map we have created at step 1. Note we can pass the default value (`None`) and load the map using the `init_map` method after generating an instance.
* origin: A distance vector between the world map's origin and the dsm map's origin. len(origin) must be 3.
* map_dimensions: A two value tuple: (<row_number>, <column_number>).
* pixel_dimensions: A two value tuple: (<dWx>, <dWy>) where dWx is the distance (in meters) one stride in the x axis direction ((x, y) -> (x + 1, y)) represents and dWy is the distance (in meters) one stride in the y axis direction represents.
* stride_multiplier: Adjusts the strides' length. default stride length is 0.6 * velocity must be > 0.
* max_angle: The maximal drone turn degree we are allowing.

#### Usage Example

```python
pg = PathGenerator(velocity=7.0, flight_height=-50.0, dsm=dsm_, origin=(x_org, y_org, z_org),
                       map_dimensions=(Wx, Wy), pixel_dimensions=(dWx, dWy), max_angle=30.0)
```
#### DSM Loading Alternative:

If you did not initiate the dsm  map in the constructor (passed `dsm=None`) you'll need to use the `init_map` method before you can use the other PathGenerator methods.

```python
def init_map(self, input_path=None, file_name=None, save_tif=False, pixel_dist=2.0)
```
This method essentially uses the `DSMParcer` function for you so the requirement concerning the BinFile folder still holds.

### 3. Change Map Resolution (Optional):

TODO: decide if these methods are still relevant.

#### Zoom In

```python
def map_zoom_in(self, multiplier: int)
```

* multiplier: The enhance multiplier. Must be > 0. Passing a negative value will cause the method to have no effect.

We've created a way to get higher resolution of the drone's location by dividing each pixel to a few pixels such that each pixel will represent a smalller real world tile with the same height value.

This method has some advantages and disadvantages:

* Advantage: The drone's location is more spacific (as every pixel represents a smaller real life area). 
* Advantage: The difference between the given constraints and the outputed path's cost (distance or time) is smaller. 
* Disadvantage: The computation time becomes longer.

These effects will be demonstrated in the results section.


##### Illustration:

![alt text](https://github.com/alond44/PathGenerator/blob/main/Ilustrations/zoom_in_example.png "Zoom In Example")


#### Zoom Out 

```python
def map_zoom_out(self, multiplier: int)
```

The multiplier argument serves the same purpose as it does in the zoom in method.
For this call: `pg.map_zoom_out(x)` the method will group the map's pixels to squares (with `x` pixels in each side) and then merge them into a single pixel with a height value of the maximum pixel height value out of the merged pixel group.
'pg.map_zoom_out(x)' call essentially reverse the effect of 'pg.map_zoom_in(x)' if one was called earlier (it revert the map back to it's state before the zoom in call). However, the zoom out method is independent and does not have to be called after a zoom in call.  

This method's advantages and disadvantages are the opposite from those of the zoom in method.

##### Notes:
* Using zoom out with multiplier M is possible even when the dsm map dimensions are YxY and Y is not divisable by M (refer to the illustration bellow for an example).
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
pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1000, path_type=PathType.MAP_ROAM, start_location=[150, 150], path_nums=1, to_print=True, weight=2)
```


### Extra Methods

#### Path Printer

```python
def print_path(self, path=None, path_color='r', path_style='--')
```

This method gets a path and some optimal parameters that control the path's print style.
Passing a `None` value as the path argument results in printing the instance's dsm map only.

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
The first kind of algorithm we have imlemented in order to create paths was a "random surfer".
At each point the agent decides which place to go from 8-ways opportunities by random, with the limitation of not going back from the same way that it came from (so in practice 7-ways selection).

<img src="https://github.com/alond44/PathGenerator/blob/main/Results/simple_example_Probabilistic.png" width="600">

As could be seen from the result, this algorithm created more "localy wandering" path that hasn't spread much over the city.

##### Method Call Example

```python
pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1500, path_type=PathType.AREA_EXPLORE, start_location=None, path_num=1, to_print=True)
```

#### Extensive Path- Weighted A* Path
The second algorithm we have implemented was a weighted A* algorithm.
The Path was created by sampling random points in the map while calculating an optimal path (or suboptimal path bounded to weight*optimal_path) between those points.

<img src="https://github.com/alond44/PathGenerator/blob/main/Results/simple_example_Weighted_A_Star.png" width="600">

As could be seen from the result, this algorithm created more distributed path that explore much bigger parts of the city.

##### Method Call Example

```python
pg.gen_paths(flag=ConstraintType.TIME, constraint=50, path_type=PathType.MAP_ROAM, start_location=None, path_num=1, to_print=True, weight=2)
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

We used these function calls to print the simple example we showed above and to test:
1. The average error of our algorithms (the difference between the given constraint and the resulted paths travel time or distance).
2. The average path calculation duration.
And how are both affected by the zoom in/out methods. 

The zoom in method resulted in longer calculation time but caused a much smaller constraint error as we passed a bigger multiplier.
Zoom out had the opposite affect, the calculation time shortened but it resulted a bigger constraint error.
Furthermore, we can notice that the distance error in our Weighted-A* runs are dependent of the width a pixel represent in real life. Meaning an error can't be larger then the width of a pixel because we are limited to make 'pixel_dist' sized steps from one point to it's neighbor in Weighted-A* (we allow movement a pixel up, down, left and right - no diagonals).
As opposed to that, the probabilistic paths aren't limited to 'pixel_dist' sized steps (as we allow diagonal moves to neighbors) so this point does not apply to them.

#### Notes
* We tested every combination of constraint type and path type (4 combinations) in both the error test and calculation time test.
* The tests outputs can be found as .png or as .txt files under the 'Results/Tests Outputs' folder. 

### Summary
In this project we have implemented 2 different algorithm for calculating paths for drones while avoiding obstacles (calculated by the drone height).
We have shown in this document how we have tested our code, how to use it and the different results we have got from our experiments.
