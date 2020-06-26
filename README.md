# PID Controller
 PID controller in C++ to correct the steering and throttle of a vehicle around a track. Feedback is provided by the simulator: cross track error (CTE) and the speed (mph). PID parameters are tuned manually by qualitatively inspecting the driving behaviour in the simulator in response to parameter changes. This repo uses [Udacity's CarND-PID-Control-Project repo](https://github.com/udacity/CarND-PID-Control-Project) as a base template and guide.

<p align="center"> 
  <img src="./README_images/pid_controller.gif">
</p>

[//]: # (List of Images used in this README.md)
[image1]: ./README_images/pid_sim.png "Simulator"
[image2]: ./README_images/pid_graph.png "PID Diagram"
[image3]: ./README_images/p_control.gif "P Controller"
[image4]: ./README_images/pd_control.gif "PD Controller"
[image5]: ./README_images/drift.png "Drift"
[image6]: ./README_images/pd_drift.gif "PD + Drift"
[image7]: ./README_images/pid_drift.gif "PID + Drift"

## Directory Structure
```
.PID-Controller
├── CMakeLists.txt        # Compiler Instructions
├── cmakepatch.txt        # Sub Dependency for Mac
├── install-mac.sh        # Dependency for Mac
├── install-ubuntu.sh     # Dependency for Linux
├── .gitignore            # git prevents unnecessary uploads
├── README.md
├── README_images         # Images used by README.md
|   └── ...
├── build.sh              # Compiles to create a build
├── run.sh                # Runs the build
└── src                   # C++ code
    ├── json.hpp          # json helper functions
    ├── main.cpp          
    ├── PID.cpp
    └── PID.h
```
Note: cmakepatch.txt is used by install-mac.sh. 

## Installation
Open your terminal and type:
```sh
git clone https://github.com/laygond/PID-Controller.git
cd PID-Controller
sudo ./install-ubuntu.sh # (or './install-mac.sh')
```
This shell file will install the dependencies for our project
- cmake >= 3.5
- make >= 4.1 (Linux, Mac), 3.81 (Windows)
- gcc/g++ >= 5.4
- uWebSocketIO  # Allows communication bewtween c++ code and simulator

Note: you might need to grant executable permission to the install shell files before running them: `chmod +x install-ubuntu.sh`. Running the install may also need the root permission prefix `sudo`. For Windows set-up and more details go to the 'Dependencies section' from [here](https://github.com/udacity/CarND-PID-Control-Project)

## Udacity's Simulator

![alt text][image1]

The simulator can be downloaded [here (Choose latest Term 2 Version)](https://github.com/udacity/self-driving-car-sim/releases). After the simulator is downloaded, make sure it has executable permision: `chmod +x term2_sim.x86_64` (in the case of linux). Several projects come bundled in this simulator release; you must choose PID among the options.

## Run Project
You can either run the following or the simulator first. The order is irrelevant. In terminal go to your PID-Controller repo and type:
```sh
./build.sh
./run.sh
```
Note: you might need to `chmod +x` the build and run shell files.

## Project Analysis
### Overview
From the simulator we are constantly measuring
- CTE  : cross track error; distance from car to reference
- Speed: current speed of the car in mph
- Angle: current steering angle of the car

and in order for the car to move and steer we should send to the simulator
- Steer   : steering angle value between [-1,1]
- Throttle: throttle value [-1,1]

where 1 or -1 referes to the max or min steering angle or throttle (forward & reverse). For this project the max speed has been raised to 100 mph.

In order to send steering and trottle values that keep the car in the center of the track, two PID controllers have been stablished in `main.cpp`: `speed_controller` and `steering_controller`

![alt text][image2]
<p align="center"> 
Image from Wikipedia: A block diagram of a PID controller in a feedback loop. r(t) is the desired process value or setpoint (SP), and y(t) is the measured process value (PV).
</p> 

<b>The PID controller input</b> is the difference between the desired and actual measurement value. This input is later split into three. 
- P_error : the current error
- I_error : the cumulative error
- D_error : the differential error

In code it would look like
```
  d_error = e - p_error;
  p_error = e;
  i_error += e;
```
where e is the input difference.

In the case of:
- speed controller: the reference or desired speed for our vehicle has been set to 30 mph. Therefore the input difference is the (speed - desired).
- steering controller: the input difference is given directly in the form of CTE. Therefore there is no need to calculate it. In fact, there is no way to calculate it since neither the car's actual pose nor reference to the center of the track are given in map coordinates.

<b>The PID controller output</b> is 
```
  return -Kp*p_error - Ki*i_error - Kd*d_error;
```
where Kp, Ki, and Kd are the parameters to find such that the desired value is achieved.

### P Controller
Let's explore what happens if we were to use only Kp for steering, i.e. Ki and Kd are set to zero.

![alt text][image3]

Once it reaches the reference track the next step will make the car go off track again and be far from the reference. This will continue causing the car to be marginally stable  (bounded oscillations). Therefore the objective is to reach the reference so that the next step also remains in the reference (converges).

### PD Controller
![alt text][image4]

PD solves the problem! It makes the car converge but what if there is a systematic bias. Say you believe your wheels are aligned but they are not, now there is a systematic steering drift. This will make it converge at a distance far from the reference.

### PD Controller + Drift
![alt text][image6]

The car is slightly to the right of the track. To simulate the drift, 0.05 has been added to the value being sent back to the simulator. 

![alt text][image5]

### PID Controller + Drift

![alt text][image7]

The PID fixes the systematic bias. You can see how it slightly corrects itself to the center lane as the car advances.

### Parameter Tuning
Based on the previous parameter explorations, the PID parameters for the steering controller were tuned manually by qualitatively inspecting the driving behaviour in the simulator in response to parameter changes. The PID parameters for the speed controller were taken from the Behavioral Cloning repo. 

|   PID    |   Kp   |   Ki   |   Kd   |
| -------- | ------ | ------ | ------ |
| Steering | 0.14	  | 0.0009 | 1.0    |
| Speed    | 0.1	  | 0.002	 |	0	    |

Parameter optimization is possible by defining a loss function which in the case of the steering can be a mean squared error (MSE) of the cte for a time range (tips: omit the first N time steps until controller adjust). In the case of the speed controller you can do the same: an MSE of the input speed difference, however, incorporating cte into this loss function as well seems to be an important indicator since the car might need to slow down while turning. 

Twiddle and SGD are computational methods to find these parameters and definitely better options for tuning than manual tuning. However, I could not find a way from code to reset multiple times the car's pose back to the start in the simulator. Twiddle explore multiple parameters by tweaking them little by little and therefore needs the simulator to run several times. Since most likely the car will crash during the first initial parameter sets a way to reset the car is needed.

Another method might be applying a neural network approach and train it with MSE as loss function and SGD as a backpropagation. However, we need a ground truth data of what should be the correct steering values (which we do not have).