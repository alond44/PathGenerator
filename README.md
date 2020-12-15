# Path Generator

## Introduction
In the following explanation we are going to explain how to simply use our Path Generating system in order to create path for the drone in the city, change the different parameters and get the path. After that we will show some result that we generated and explaing the pros and cons of the 2 methods we chose to code.

## How to Use
All of our code is inside a python class named PathGenerator. In order to use it you should:

### 1. Use the parser in oreder to create the dsm map.
The Parser is found in DsmParser.py and in order to run it you should use the following code:

'''python
    Inputpath = Path(__file__).parent.absolute()
    FileName = 'dsm_binary'
    dsm_ = create_map(Inputpath, FileName)
'''
make sure your .bin file is under workingfolder/BinFiles/

### 2. Create an Instance of the PathGenerator:

'''python
    pg = PathGenerator(velocity=, flight_height=, dsm=_dsm, pixel_dist=)
'''

At this point you need to choose the flight parameters:
* velocity: The velocity of the drone
* flight_height: the height of the drone flight
* dsm: the dsm map we have created at step 1
* pixel_dist: the distance between 2 pixels on the map

### 3. Change Map Resolution:
TODO: Alon please add...

### 4. Creating The Path:

'''python
    pg.gen_paths(flag='d', constrain=1000, path_type='a_star', start_location=[150, 150], to_print=True, epsilon=2)
'''

parameters:
* flag: could be either 'd' for distance or 't' time meaning what kind of constrain we want for our path.
* constrain: If we chose flag = 'd' the number inserted here will be the limit for distance the drone will go. If we chose flag = 't' so constraing will be the time limit for the drone path.
* path_type: 'prob' or 'a_star' as will be explain in more details later- 'prob' creates a simple path using random walk in the map while 'a_star' create path using weighted a* algorithm between some randome sampled points.
* start_location: the start location of the drone
* to_print = TODO- needed to fix in code- return the path itself for either options and print it only if the parameter is True...
* epsilon: Only when using 'a_star'. epsilon should be >= 1.

## Our Results and Algorithm Explanation:

### Simple- Random Walk Path
![alt text](https://github.com/alond44/PathGenerator/blob/main/Results/random_walk%20result.png "Random Walk Example Result")
