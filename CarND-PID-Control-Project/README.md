# PID Control Project
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)
[video1]: ./videos/best.mp4 "Best solution"
[video2]: ./videos/largekp.mp4 "Large Kp"
[video3]: ./videos/largeki.mp4 "Large Ki"
[video4]: ./videos/largekd.mp4 "Large Kd"

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`

## Results

##Effect each of the P, I, D components

* Kp scales the direct reaction on the tracking error. The larger it is, the faster a tracking error is reduced.
A large Kp results in over shooting. The  car oscillates around the optimal path. The video shows a large Kp.

![alt text][video2]

* Ki scales the integrated tracking error. The integration gives the accumulated offset that should have been corrected previously.
Ki scales therefore the bias compensation. The video shows a large Ki.

![alt text][video3]

*Kd scales the derivative of the tracking error. The derivate of the tracking error is the rate of change and describes the dynamic behind the tracking error. The Kd parameter allows to damp oscilations of the car. A larger value reduces oscillation and inreases the time needed to compensate the tracking error. The video shows a large Kd.

![alt text][video4]

## How the final hyperparameters were chosen

I started first with a manual adjustment to enable the PID controller to keep the car on the lane.
Using this parameters the simulator was able to drive the car around the course with the highest speed.

The result has been these parameters

| Kp      | Ki       | Kd       |
|:-------:|:--------:|:--------:|
| 0.08    | 0.00018  | 0.777    |

In a second step I used the twiggle parameter optimzation algorithm to improve the parameters.
It started with an error of 0.842142.
All 900 update requests the PID controller calls the twiggle optimizer to change the value of one parameter. 
This covers about round of the parcour and ensures that the summarized tracking error is comparable in each optimization step.
All 4500 update requests the optimization switches to the next parameter. the best error I gained is 0.235613.

The optized parameters are:

| Kp        | Ki         | Kd       |
|:---------:|:----------:|:--------:|
| 0.0724342 | 0.00006939 | 0.74038  |

![alt text][video1]





