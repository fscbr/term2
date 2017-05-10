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
5. Choose Fastest to Simple Graphics quality for the simulator configuration 

## Effect each of the P, I, D components

* Kp scales the direct reaction on the tracking error. The larger it is, the faster a tracking error is reduced.
A large Kp results in over-shooting. The car oscillates around the optimal path. The video shows a large Kp.

![large Kp value][video2]

<a href="http://www.youtube.com/watch?feature=player_embedded&v=OPx6l4ZHsf4
" target="_blank"><img src="http://img.youtube.com/vi/OPx6l4ZHsf4/0.jpg" 
alt="here at youtube" width="480" height="270" border="10" /></a>


* Ki scales the integrated tracking error. The integration gives the accumulated offset that should have been corrected previously.
Ki scales therefore the bias compensation. The video shows a large Ki.

![large Ki value][video3]

<a href="http://www.youtube.com/watch?feature=player_embedded&v=4IPbuwjEkWY
" target="_blank"><img src="http://img.youtube.com/vi/4IPbuwjEkWY/0.jpg" 
alt="here at youtube" width="480" height="270" border="10" /></a>

* Kd scales the derivative of the tracking error. The derivate of the tracking error is the rate of change and describes the dynamic behind the tracking error. The Kd parameter allows to damp oscilations of the car. A larger value reduces oscillation and inreases the time needed to compensate the tracking error. The video shows a large Kd.

![large Kd value][video4]

<a href="http://www.youtube.com/watch?feature=player_embedded&v=ZdmMWmH42Oo
" target="_blank"><img src="http://img.youtube.com/vi/ZdmMWmH42Oo/0.jpg" 
alt="here at youtube" width="480" height="270" border="10" /></a>


* Update rate of the simulator. My PID controller is optimized for the fastest upto simple graphics quality. Higher graphics modi result because of a slow frame rate in a lagged communication with the PID control. This leads to oscillation and might cause the car to get off the track.


## How the final hyperparameters were chosen

I started first with a manual adjustment to enable the PID controller to keep the car on the lane.
Using this parameters, the simulator was able to drive the car around the course with the highest speed.

The result has been these parameters:

| Kp      | Ki       | Kd       |
|:-------:|:--------:|:--------:|
| 0.08    | 0.00018  | 0.777    |

In a second step, I used the twiggle parameter optimization algorithm to improve the parameters.
It started with an error of 0.842142.
All 900 update requests the PID controller calls the twiggle optimizer to change the value of one parameter. 
This covers about round of the parcour and ensures that the summarized tracking error is comparable in each optimization step.
All 4500 update requests the optimization switches to the next parameter. The best error I gained is 0.235613.

The optimized parameters are:

| Kp        | Ki         | Kd       |
|:---------:|:----------:|:--------:|
| 0.0724342 | 0.00006939 | 0.74038  |

![best solution][video1]

<a href="http://www.youtube.com/watch?feature=player_embedded&v=NW2TqJ_NRkQ
" target="_blank"><img src="http://img.youtube.com/vi/NW2TqJ_NRkQ/0.jpg" 
alt="here at youtube" width="480" height="270" border="10" /></a>

## Introducing a second PID for the speed

After receiving a review where the car was not able to stay on the track, I decided to introduce a second PID for the throttle.
Doing this, I hope to reduce the effect of slower computers that have a low update rate even in fast graphics mode.
The PID out put is [-1,1]. The throttle value is then:  throttle_value = pid*0.4+0.6. 

This PID receives the absolute value of the tracking error to get reduce the throttle, when the tracking error is high and to accelerate, when the tracking error is low. The twiggle optimization for steering resulted in:

| Kp        | Ki         | Kd       |
|:---------:|:----------:|:--------:|
| 0.0711094 | 0.000064576 | 0.696917  |

for the throttle:

| Kp        | Ki         | Kd       |
|:---------:|:----------:|:--------:|
| 0.50415   | 0.00001    | 0.01     |

 I tested it for graphics mode from "Fast" to "Beautiful" and the car stayed on the lane.




