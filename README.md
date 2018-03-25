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
psi_t+1 		= psi_t + (v_t / Lf) * delta * dt
v_t+1 		= v_t + a * dt
epsi_t+1 	= epsi_t + (v_t / Lf) * delta * dt
cte_t+1 		= cte_t + v_t sin(epsi) * dt
```

Where `Lf` represents a physical characteristic of the vecile  (the difference between the center of the mass and the front axle).  

## N and dt

The size of N and dt had a direct effect on the computational resources as these define the amout of computational steps to be performed. My approach therefore was to define values for `N` that do not exceed 20 as a big horizon would not necessarily increase the predicion performance. In fact my tests reveal that values exceeding 10 implied a bad polfit in case `dt` was set to 0.1. 
Since dt defines the amout of steps per horizon, and therefore, the resolution, it was epected that values 0.1 < `dt` would not be beneficial for the prediction as the resolution would imply a detailed trajectory but for a limited horizon. My final selection therefore was

```
N = 10 
dt = 0.1
``` 

