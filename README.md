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

```python
def map_zoom_in(self, multiplier: int)
```

* multiplier: The enhance multiplier. Must be > 0. Passing a negative value will cause the method to have no effect.

We've created a way to get more specific with the drone's location by dividing each pixel to a few more so each pixel will represent a smalller real world tile with the same height value.
For example: the first of the following images represents a single pixel and the second represents that same pixel after calling `pg.resize(2, True)` resulting it to split into four different pixels that represent half of the width and fourth of the area.

This usage has some positive and negative consequences:
* + The drone's location is more spacific (as every pixel represents a smaller real life area). 
* + The difference between the given constraints and the outputed path's cost (distance or time) is smaller. // TODO: check if affects on the error and show results. 
* - The computation time lengthen. // TODO: check and show results.



Another usage of resize_dsm is achieved by passing 
This method can also reverse its affect by passing a False value with the same multiplier value. by passing a false flag the method merge adjecent pixels to be one with the height of the maximum height of the united pixels. This option can be run without enhancing first, however calling resize with a false enhance flag comes with few consequences:
* It might cause information lose (caused by merging a group of pixels that may hold different height values to one with a single height value).



### 4. Creating Paths:

```python
def gen_paths(self, flag, constrain, path_type, start_location=None, path_num=1, to_print=False, epsilon=1.0)
```

parameters:
* flag: could be either 'd' for distance or 't' time meaning what kind of constrain we want for our path.
* constrain: If we chose flag = 'd' the number inserted here will be the limit for distance the drone will go. If we chose flag = 't' so constraing will be the time limit for the drone path.
* path_type: 'prob' or 'a_star' as will be explain in more details later- 'prob' creates a simple path using random walk in the map while 'a_star' create path using weighted a* algorithm between some randome sampled points.
* start_location: The start location of the drone. If `None` is passed a random one will be generated. 
* path_num: The number of paths we want to generate with these constraints. The default value is 1.
* to_print: A boolean value indicating if we want the paths to be printed on the map.
* epsilon: Only when using 'a_star'. epsilon should be >= 1.

#### Usage Example

```python
pg.gen_paths(flag='d', constrain=1000, path_type='a_star', start_location=[150, 150], path_nums=1, to_print=True, epsilon=2)
```


## Our Results and Algorithm Explanation:

### Simple- Random Walk Path
![alt text](https://github.com/alond44/PathGenerator/blob/main/Results/random_walk%20result.png "Random Walk Example Result")
