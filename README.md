# CarND_Model-Predictive-Control


## Model Definition

The model used represents a kinematic vehicle model with the following (extended) state vector:
 
```
[x, y, psi, v, cte, epsi]
```

Where `x, y` represent the position in a 2 dimensional carthesian coordinate system, `psi` the orientation,  `v` the velocity of the vehicle, `cte` the cross track error and `epsi` the orientation error.

The control inputs of the vehicle model used in the project are limited to 

```
[delta, a]
```
where `delta` represents the steering and `a` the throttle valve actuation.

Updates of the state vector are calculated using the following equations:

```
x_t+1 		= x_t + v_t * cos(psi_t) * dt
y_t+1 		= y_t + v_t * sin(psi_t) * dt
psi_t+1 	= psi_t + (v_t / Lf) * delta * dt
v_t+1 		= v_t + a * dt
epsi_t+1 	= epsi_t + (v_t / Lf) * delta * dt
cte_t+1 	= cte_t + v_t sin(epsi) * dt
```

Where `Lf` represents a physical characteristic of the vecile  (the difference between the center of the mass and the front axle).  

## N and dt

The size of N and dt had a direct effect on the computational resources as these define the amout of computational steps to be performed. My approach therefore was to define values for `N` that do not exceed 20 as a big horizon would not necessarily increase the predicion performance. In fact my tests reveal that values exceeding 10 implied a bad polyfit for velocities > 40mph and in case `dt` was set to 0.1 (which represents the delay mentioned below). 
Since dt defines the amout of steps per horizon and therefore,the resolution, it was epected that values 0.1 < `dt` would not be beneficial for the prediction as the resolution would imply a detailed trajectory but for a limited horizon. My final selection therefore was

```
N = 10 
dt = 0.1
```

## Dealing with latency

The effect of the actuation delay resulted in a poor MPC behaviour as the vehicle was following points on a trajectory that was not up to date. 
In order to resolve these issues the tradidional approaches would be to either "speed up" the execution (latency decrease) or "slow down" the vehicle state estimation (by using one old state variables). 
Another interesting approach mentioned within the Q&A and the intro of the project was to use the delay within the vehicle model.
I have tried to implement this approach by simplyfying the the equation above with the assumption that no previous state variables are present and the current values for `x, y and psi` are `0` due to the previously performed transformation of the vehicles position into the vehicle`s coordinate system.
Introducing a latency into the current state vector therefore implied a simple multiplication of the current state vector by the latency.

```
px_delayed		= v * cos(psi) * latency
py_delayed		= v * sin(0) * latency
psi_delayed 		= - v * delta / Lf * latency
v_delayed 		= v + a * latency
cte_delayed		= cte + v * sin(epsi) * latency
epsi_delayed		= epsi + psi_delayed
```

Where `px_delayed` was identical to `v * latency`, `py_delayed` identical to 0 and `latency` was set to `0.1`s. 




  
