# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Results
My code running on the simulator can be seen in the video below. It successfully completes an entire lap around the course without any incidents. There are times when it slows down a bit too much in complex scenarios but I might be able to fix this with more tuning of acceleration parameters.

https://youtu.be/HHE7dbg77oo 

## Code Explanation
### Path Generation
The spline library mentioned above was heavily used during the path generation portion of the code. Five points were passed into the spline object in order to create a polynomial that best fit the five points. If the previous path had more than two unused points left in it then the first two points for the spline are taken from the last two points of the previous path. If the previous path had less than 2 unused points in it then the current position of the car and the previous position of the car are used as the first two points for the spline. 

To generate the last three points for the spline I picked three points in front of the car spaced 30 meters apart as shown in the code below.

``` c++
// Generate points along the current lane that you are in by incrementing the s value of the car
vector<double> s_30_point = getXY(car_s+30, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> s_60_point = getXY(car_s+60, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> s_90_point = getXY(car_s+90, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

Because we are using Frenet coordinates we can get a further ahead point in the car's current lane by simply adding a desired distance to the car's current s value while keeping the d value the same. 

After all 5 points for the spline are generated they are used to create the actual spline object as seen below.
``` c++
// Create a spline
tk::spline sp;

// Add the points to the spline
sp.set_points(spline_pts_x, spline_pts_y);
```

Before the spline object is used to generate points for the path, the previous path's unused points are added to the next path so the car can continue on it's current path until it needs new points. After the old path points have been added, the spline object is used to genrate (50 - the size of the leftover points) more points for the path. 

Once all 50 points have been created they are sent to the simulator where the car will sequntially move through them. 


### Lane Changing and Driving Logic
The main factor behind both speed and lane changing was the presence of a slow moving car in front of our vehicle. In order to detect a slower moving car in front of us, the sensor_fusion variable was parsed to check the location of each of the detected vehicles around our car. If the detected car was in our lane, ahead of us, and within a certain distance threshold then the car would slow down and prepare to change lanes. The code for this can be seen below.

``` c++
// Check if the car is in our lane by checking its d value
if((other_d < lane*4 + 4) and (other_d > lane*4))
{
  // Calculate the velocity of the other vehicle
  double other_vel = sqrt(pow(vx, 2) + pow(vy, 2));
  
  other_s += (double)prev_path_size * 0.02 * other_vel; // use previous points to project s value onward
    
  // Check if the car is in front of us
  if(other_s > car_s)
  {
    // Check if the car is too close
    // TODO: Tune this val
    if(other_s - car_s < 27.5)
    {
      // Set flag for being too close to the car in front
      // Also set flag to look for lane change so we can get around the slow car
      car_in_front = true;
      prep_for_lane_change = true;
    }        
  }              
}
```

The forward projection of the other vehicles s value was very important because it let us know where the car would be in the future after we had actually executed our path. Using the current value for the car for future decision making led to a number of close calls and even collisions during initial testing. The safety distance value between the two cars was experimented with to try to find the optimal distance but this can likely be improved further. It might also be good to make this value a function of the car's speed in order to accurately reflect how quickly our car might close this distance. 

After the prep_for_lane_change flag has been set, the sensor_fusion variable is looped through again to determine if any cars are to our left or right that would prevent us from safely changing lanes. The s value of the other car is compared to our car's s value and if it is in the unsafe window around our car a flag is set to prevent a change into that lane. The code for this can be seen below.

``` c++
// Check farther in front so that we don't get stuck behind other cars in the lane we might want to change into
if(other_s < car_s + 20 and other_s > car_s - 15)
{
  if(other_lane == lane-1)
  {
    left_car_too_close = true;
  }
  else if(other_lane == lane+1)
  {
    right_car_too_close = true;
  }
}
```

We check further in front of our car to prevent us from changing lanes and then getting stuck behind another slow moving car. This happened a number of times when the unsafe window was smaller and lead to instances of unsafe driving decisions made by the car. The 35 meter no go zone might seem fairly large but the sizable window helps to keep safety as the number 1 goal of the vehicle. In the results video linked above there are cases where a lane change into a faster lane might have been possible for the car but doing so would have been risky due to the presence of other cars. 

In order to actually change lanes the lane value of our car is simply incremented or decremented if the don't change lanes flags have not been set. This can be seen in the code below.

``` c++
// If conditions are right then change lanes. Prioritize left lane changes
if(left_car_too_close == false and lane != 0)
{
  lane--;
}
else if(right_car_too_close == false and lane != 2)
{
  lane++;
}
```

Left lane changes are checked for first as it is the common rule of the road to pass on the left. 

If the car_in_front flag was set then our vehicle must slow down in order to prevent a collision with the car in front of it. To do this I just decreased the velocity used to generate points during the path generation portion of the code. This lower velocity leads to the 50 generated points being closer together which in turn causes the car to drive slower. The code for this can be seen below.

``` c++
// Decrease our speed if there's a car close in front of us 
if(car_in_front)
{
  target_vel -= .336;
}
// Increase speed if we have open road and are below the limit
else if(target_vel < max_vel)
{
  target_vel += .336;
}
```

Included in this code is the logic to speed back up if we are below our max velocity and have no other cars in front of us. This gradual acceleration combined with the the starting velocity of 0 m/s helps to fix the issue with the car instantly reaching it's max velocity during the beginning of the simulation. 


## Acknowledgements
As mentioned previously the spline library was a crucial part of my project for generating a drivable path for the car. I also referred to the Q&A video included with the project to help with some of the issues I was having with path generation. 
