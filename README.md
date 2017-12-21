##Model Predictive Control

[//]: # (Image References)
[image1]: ./Images/Model.png "MPC Model"
[video1]: ./Images/1.png "MPC Car result"
[video1]: ./Images/2.png "MPC Car result 2"

Drive a vehicle on a simulated track without leaving the main path and running over or hitting curbs using Model Predictive Control (MPC).

Here is a video recording of the car navigating on the simulated track with MPC: [MPC Car result 2](https://youtu.be/U3xH2gtERaI)

[![alt text][video1]](https://youtu.be/U3xH2gtERaI "MPC Car result 2")

### The Model

The goal of MPC is to optimize the control inputs: Steering angle and Acceleration.
I used the model/MPC algorithm that is described in the lectures and quizzes.

For the trajectory, I chose duration and timestep values: *N*, as 15 and *dt* as 0.05 after some fine tuning.

This is the Model I used from the lectures:
![alt text][image1]


State: We use the initial state consisting on [x, y, psi, v, cte, epsi].

x, y: the global x and y position of the car. <br>
psi: Orientation of the car. <br>
v: velocity <br>
cte: cross track error <br>
epsi: orientation error. <br>

Actuators: steer_value and throttle_value

We also set the vechicle model constraints - lower and upper bounds and variables.

### Timestep Length and Elapsed Duration (*N* and *dt*)
I started the values of N and dt with 10 and 0.05. With these values I acheived good track navigation but with zero latency. 

Once the latency is added, I started seeing the car is not making the curves. The predicted trajectory clearly showed it needs larger timestep length. I played with some more values before settling down to 20, which helped the car complete the track successfully.

###Update:
Additional tuning done for the cost function affecting steering and acceleration. This gives smoother transitions during sharp turns.

### Polynomial Fitting and MPC Preprocessing

I used polyfit method to fit a polynomial. Used a 3rd degree polynomial since we have curved lines and not straight ones.

<code>auto coeffs = polyfit(positionX, psitionY, 3);</code>

positionX and positionY are vectors of waypoints transformed to the car coordinates from the maps coordinate system.

The cross track error is calculated by evaluating the polynomial at x, f(x) and subtracting y. The values of x, y are used from the quiz. -1 and 10. I tried several other values and settled on the above.

The above x, y values along with orientation, velocity, cte and orientation error init values are passed to the MPC solver.


### Model Predictive Control with Latency

The MPC code uses a 100 ms latency which simulates the time elapsed between actuator commands and the action of those values by the sensors.
Initially I chose to use 0 latency and tune the hyperparameters N and dt. Once the car is performing satisfactorily, I turned on the latency and observed that the car is wobbly and going off track during sharp curves and turns. By playing with different values of timestep length, the init values of state that is fed into the MPC Solver, I settled on the values described earlier.

###Update:
Based on the feedback from my earlier review, I used, kinematic equations for some states factoring in the 100ms latency before sending them to MPC Solver. 

This latency factoring along with the tuning of cost function with steering and acceleration, the vehicle now runs smoothly at ref_v=40 mph and 60 mph. With 40 mph, the vehicle made multiple laps successfully.

