# CarND2-P4 PID Control

## Description

**This my 4th project result of Udacity self-driving car nanodegree (CarND) term 2. It's required to implement a PID controller of a vehicle. A vehicle simulator is provided to validate performance of PID controller.**

**The following demonstrates designed PID controlled vehicle moving in simulator :** 

![alt text][image1]

* Udacity self-driving car nanodegree (CarND) :

  https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
  
* Udacity self-driving car nanodegree term 2 simulator :

  https://github.com/udacity/self-driving-car-sim/releases/

[//]: # (Image References)
[image1]: ./images/p.gif
[image2]: ./images/p1.gif
[image3]: ./images/p2.gif
[image4]: ./images/p3.gif


**File structure:**

* The PID controller setup and interaction between simulator and PID controller are implemented in C++ file `./src/main.cpp`.

* The PID controller is implemented in C++ file `./src/PID.cpp`.

## Usage
* `mkdir build` 
* `cd build`
* `cmake ..`
* `make`
* `./pid` to use default setting or for example, `./pid 0.3 0.002 6.0 40.0` to set Kp = 0.3, Ki = 0.002, Kd = 6.0 and Vt = 40.0 mph. (in which Kp is proportional coefficient Kp, Ki is integration coefficient, Kd is deviation coefficient and Vt is target speed.)
* Download simulator `term2_sim.app` (if in OSX) and open it. Click `play!` bottom, select Project 4: PID Controller to start.

## PID controller

PID controller means propotional(P), integration (I) and deviation (D) controller.

The vehicle oscillates more with higher porportional coefficient Kp, oscillates less with higher deviation coefficient Kd and follow close to the lane center more with relatively slight value of integration coefficient Kp. 

For example, P controller is demonstrated in the following video. Kp = 0.2 and Kp = 0.4 show different oscillation behavior. Higher Kp shows higher oscillation.

Kp = 0.2

![alt text][image2]

Kp = 0.4

![alt text][image3]

Another PD controller is dmonstrated with Kp = 0.4 and Kd = 1.0 in the following video. Obviously, deviation component mitigates the oscillation when it's compared with P controller.

Kp = 0.4 and Kd = 1.0

![alt text][image4]


## Parameter setting and hyperparameter design

I started with Kp = 0.2 ~ 0.4. The vehicle oscillates heavily when Kp is large. Then I added Kd = 1.0 to mitigate the oscillation. After tried some parameters' settings, I found it's still difficult to steer enough to pass through the curve lane while keeping the following oscillation small at high speed.

Since human beings drive through curve lane with relatively low speed, the throttle may be better controlled related to steer angle and current speed.

Therefore, I designed the following method to control the throttle :

If current speed exceeds target speed or steer angle is too large ( |steering| > 0.25 ), the throttle should be lessened. The lessened throttle is set as 0.15 by trial and error.

`throttle = 0.15`

If current speed doesn't reach target speed and steer angle is small  ( |steering| <= 0.25 ), the throttle should be increased. The throttle is designed to be proportional to absolutely difference between target speed and current speed. Also, the throttle value is modulated related to steering value (when higher steer angle is, lower throttle is). In the end, the final throttle value is limited by 0.4 to prevent too large throttle.

`throttle = abs(target_speed - current_speed) * (1.01 - abs(Steer));`

`throttle = min(0.4, throttle);`

After several manual tunings, I found several parameters that can successfully make vehicle drive the entire lap around the track. And here's one successful parameter set :

* Kp = 0.3
* Ki = 0.002
* Kd = 6.0
* Vt = 40.0
