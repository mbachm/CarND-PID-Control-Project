# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

Overview
---
This is my implementation of project 4 of term 2 of the Udacity - Self-Driving Car NanoDegree . You can find the original repo under [CarND-PID-Control-Project](https://github.com/udacity/CarND-PID-Control-Project).

Reflection
---
### Description of the PID components
1. P - proportional coefficient:

   This component has the most effect on the controller. It computes an outcome proportional to the cross track error (CTE). However, it will always overshot and oscillate around the CTE. The D parameter countersteers this coefficient.

2. I - integral coefficient:

   The I parameter compensates systematic bias (e.g. steering of the wheels of a car are wrong). This is measured and done by the sum/integral of the CTE over time.

3. D - derivative coefficient:

   The D parameter countersteers the P coefficient. This is done by a differential term, so the controller will notice that the vehicle is reducing the CTE. The countersteering allows to approach the target trajectory in a smoother way.


All 3 coefficient have a distinct hyperparameter (tau) which is multiplied with the outcome of their term. The overall PID term is 
> -(P-tau * CTE) -(D-tau * differential of CTE) - (I-tau * Integral/Sum of CTE over time)

### Discovery of final hyperparameters 
At first, I manually tuned the hyperparameters. Afterwards, I used the twiddle algorithm you can find in `PID.cpp` to optimize the hyperparameters. The most import step was to wait until the vehicle had some speed and then start the twiddle algorith with not to high `dp` values to change the parameters. The final results are:
- `P: 0.243176`
- `I: 0.000226266`
- `D: 2.99982`

How to run this project
--- 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build`
2. cd build`
3. `cmake ..`
4. `make`
5. `./pid`
