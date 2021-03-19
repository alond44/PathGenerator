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

The folder 'BinFiles' does not have to be inside your working folder. The other option is to keep the dsm binary file in a folder named 'BinFiles' in a different folder ('Files' for example) and pass that folder's absolute path as 'Inputpath' to 'DSMParcer'.

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

### 3. Creating Paths:


```python
def gen_paths(self, flag: ConstraintType, constraint: float, path_type: PathType, start_location=None, path_num=1, to_print=False, weight=1.0, result_folder_path=None)
```

parameters:
* flag: Will hold either ConstraintType.DISTANCE for distance constraint or ConstraintType.TIME for time constraint.
* constraint: Either the paths' length in meters for distance constrained paths or the travel time for time in seconds constrained paths.
* path_type: The kind of path generating algorithm used. Will hold either PathType.AREA_EXPLORE for probability random path and PathType.MAP_ROAM for weighted A* generated path to random spots.
* start_location: The path's first point. If None is passed or the given point is not valid a random one will be generated.
* path_num: The number of paths wanted.
* to_print: Pass True to print the resulting paths over the dsm map.
* weight: Relevant only when using 'a_star'. weight should be >= 1.
* result_folder_path: The path to the folder we want our output csv files to be written to. If it is `None` the path files will be created in a "Path" folder inside the work directory.

Return value - A list of 'path_num' paths.

Note: The generated paths will be printed to .csv files. Each way point will be represented by location in every axes of the world's coordinate system as well as speed in each of those axes.

#### Usage Example

```python
pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1000, path_type=PathType.MAP_ROAM, start_location=[150, 150], path_nums=1, to_print=True, weight=2)
pg.gen_paths(ConstraintType.TIME, 100, PathType.AREA_EXPLORE, path_num=5, to_print=True)
```


### 4. Static Fields and Obstacle Debug Option:

```python
    SAMPLE_RATE = 0.6           # The drone way point minimal distance coefficient is SAMPLE_RATE * velocity.
    MAX_STRIDE_LEN = 6.0        # Allowing the largest stride length to be MAX_STRIDE_LEN meters.
    MIN_STRIDE_LEN = 2.0        # Allowing the smallest stride length to be MIN_STRIDE_LEN meters.
    MAX_ANGLE = 60.0            # Allowing the largest maximum turn angle value to be MAX_ANGLE meters
    MIN_ANGLE = 20.0            # Allowing the smallest maximum turn angle value to be MIN_ANGLE meters
    DEGREE_DIVISOR = 2          # Allowing DEGREE_DIVISOR possible angles of right turn and DEGREE_DIVISOR of left turn.
    RADIUS = 1.2                # We don't allow the drone to be within RADIUS meter of an obstacle.
    RANDOM_TURN_PROB = 0.10     # The probability of a random turn while creating paths of type AREA_EXPLORE.
    RANDOM_TURN_ANGLE = 120.0   # The desired turn angle during a random turn (in degrees).
    EPSILON = 0.001
    # Important: read the doc if you decide to change these constants.
    # These are used to better our obstacle creation process.
    MIN_RECURSION_DEPT = 50  # We don't allow the recursive method for obstacle tracing to have
    # a maximal dept lower then 50.
    RECURSION_TO_MAP_SIZE_RATIO = 75.0 / 770.0  # Used to set a depth maximum for the obstacle finding function.
```
These static fields responsible to many of the class's functionalities.

* SAMPLE_RATE - this field indicates the way point sampling rate of the drone. A sampling rate of 0.5 way points per second means that each of the paths' way points need to be far enough from it's previous so that the drone won't fly past a way point and end up skipping it an option that might cause the drone to crash like shown in the following image.

<img src="https://github.com/alond44/PathGenerator/blob/random_turn/Ilustrations/Collision%20Caused%20By%20Waypoint%20Skip.png" width="400">

* MAX_STRIDE_LEN and MIN_STRIDE_LEN - these fields are responsible to adapt the distance traveled between way points so that the strides won't be too short and cost use in calculation time and not too long and the PathGenerator will have trouble moving through tight hallways (considering the drone needs to keep safety distance from obstacles).
* MAX_ANGLE and MIN_ANGLE - similar to the stride length adjusting fields, these are used to adjust the turn angle recieved from the user while building instances. If the user inputs a turn angle larger than MAX_ANGLE, the turn angle will be MAX_ANGLE (and the similarly for MIN_ANGLE). This is used so the angles won't be too wide and the flight will be physically possible (while keeping a constant flight speed) and so that the angles won't be too narrow and multiple stride options won't be as different from each other.
* DEGREE_DIVISOR - this field is responsible for the number of neighbor states each state of our search will have. Having a degree divisor with value 3 means we can proceed, from each way point, to 3 way points to the left of the drone, 3 to the right and 1 forward.
* RADIUS - the drone will allways be at least RADIUS meters away from every obstacle while following outputed paths.
* RANDOM_TURN_ANGLE - relevant for generating AREA_EXPLORE paths only. This field sets the maximal turn angle preformed while randomizing a turn when generating a path (the turn angle will be smaller if the drone could not preform the full turn angle). You can make the drone to do random circles while setting the value to 360 :).
* EPSILON - a value that affects how we determine equality between two values. we might have numeric errors that can cause two values that supposed to be equal be slightly different therefore we use that variable to determin equality (|a - b| < EPSILON meaning a == b). There shouldn't be any need to change that value.
* RANDOM_TURN_PROB - explained in the comment.
* RECURSION_TO_MAP_SIZE_RATIO - we use recursion depth limitation in the recursive method used for creating the obstacle's polygons. We limit the depth to avoid exceeding the system's maximal depth and to create smaller obstacle polygons so we won't delimit as much legal flight zone (we use convex polygons to represent the obstacles and the might cover legal flight zone when delimiting concave obstacles - like a 'c' shaped one). 
* MIN_RECURSION_DEPT - this helps us adjust the recursion depth recieved after calculating it using the previous field (upper limit is received using the os python library).

 #### DEBUG_OBSTACLE
 The path_generator file contains a boolean variable DEBUG_OBSTACLE that when True the obstacle's polygons will be printed when calling the print_path method.
 If you want to check your obstacle creation, you can set the variable to be True, create a PathGenerator instance and call:
 
 ```python
 pg.print_path()
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

### Tests Results

The tests.py holds these function calls:
(TODO: fill)

#### Notes
* We tested every combination of constraint type and path type (4 combinations) in both the error test and calculation time test.
* The tests outputs can be found as .png or as .txt files under the 'Results/Tests Outputs' folder. 

### Summary
In this project we have implemented 2 different algorithm for calculating paths for drones while avoiding obstacles (calculated by the drone height).
We have shown in this document how we have tested our code, how to use it and the different results we have got from our experiments.
