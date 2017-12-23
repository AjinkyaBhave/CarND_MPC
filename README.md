# Model Predictive Control (MPC) Project

This project involves using an MPC-based approach to drive a car around a simulated track.

## Model

The controller uses the kinematic bicycle model introduced in the lessons. It is a planar model with the (x,y) position, yaw angle, and velocity as states. In addition, the cross-track error (CTE) and heading error (HE), both in vehicle coordinate frame, are augmented states for the MPC formulation. 

The state equations as constraints for the solver are implemented in `MPC.cpp`(lines 109-115):

```C++
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + (v0/Lf) * delta0 * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 + (v0/Lf) * delta0 * dt);
```
## Timestep Length and Elapsed Duration (N & dt)
The final N was 10 time steps with a sampling interval dt of 0.1 seconds, resulting in the state prediction horizon of 1 second. I also tried larger values of N (15-25 steps) which resulted in less accurate model prediction and resulting poorer controller performance. Increasing N also increased the state dimension, and hence, the computation time for the optimisation algorithm. Larger dt (0.2-0.3) resulted in slower responses to actuation, leading to large swerving behavior of the car, finally resulting in instability, especially around turns.
 
## Including Actuator Latency

I modeled the actuator latency into my initial value for the state vector.  To do this, I took the current state values from the simulator and projected them forward one latency time step of 0.1 seconds. I also inverted the sign of the steering value returned from the simulator to match the model convention. The delay model is implemented in `main.cpp` (lines 104-107):

```C++
v = v + throttle_value*delay;
psi = psi + (v/2.67)*steer_value*delay;
px = px+v*cos(psi)*delay;
py = py+v*sin(psi)*delay;
```

This new state is used to transform the waypoints from the simulator so that the new origin of the vehicle coordinate frame is still at (0,0) after 0.1 seconds from the present time. I experimented with not using a velocity update since I read that the throttle does not match well with the real acceleration of the vehicle in the simulator. However, keeping the update did not worsen the controller performance, so I have retained it in the final submission.
 
## Polynomial Fitting and Preprocessing

I transformed the waypoints from global to vehicle coordinate frame so that the cte and heading error are easier to calculate. This is done in `main.cpp` (lines 109-115):
```C++
for(size_t i = 0; i < ptsx.size(); i++) {
	double shift_x = ptsx[i] - px;
	double shift_y = ptsy[i] - py;
	ptsx[i] = (shift_x * cos(psi)+shift_y*sin(psi));
	ptsy[i] = (-shift_x * sin(psi)+shift_y*cos(psi));
}
```
I chose a third-order polynomial for fitting waypoints using polyfit() (line 122). This polynomial is used to calculate the CTE and HE in vehicle coordinates, at 0.1 seconds from present time. The calculation assumes the state (x,y,psi) as (0,0,0) in vehicle coordinates. This is implemented in `main.cpp` (lines 124-127):
```C++
double cte = polyeval(coeffs, 0);
double epsi = -atan(coeffs[1]);
```

## Parameter Tuning

The cost function with final tuned weights for the controller is implemented in `MPC.cpp` (lines 54-71). It includes the CTE and HE, actuator magnitudes, and actuator rates. I first made all weights equal to 1 and started increasing the CTE gain till the car was on the verge of oscillating. I fixed the CTE weight and started tuning the heading error weight till I got a stable trajectory on the straight portions of the road. I then iteratively tuned the steering rate and magnitude weights till the car was able to navigate the road successfully over multiple laps. 

I fixed the weight for the velocity error at 0.8 to enable the controller to choose the best speed while navigating turns, while maintaining a higher speed on straight road sections. The reference speed was set at 40 MPH. The controller was able to navigate the track successfully till a reference speed of 45 MPH with the tuned weights. I have kept the speed at 40 MPH for the final submission.

## Results
I had submitted the project once before after testing it extensively on my machine with a reference speed of 40 MPH. For some reason, the same code caused the car to veer off the road on the reviewer's machine, probably because he/she has a slower machine. 

To show that the MPC parameters work correctly on my machine, I have recorded a video of the original code working for two full laps of the track. The simulator settings were at Fastest with a resolution of 640x480. Higher resolution settings also work successfully. The link to the video is ![MPC at 40 MPH](./MPC_Result_AYB.webm).

I have also incorporated the feedback from the reviewer regarding making the cost weights more optimised and I can now drive the track at 55 MPH successfully. I have recorded that video with the same simulator settings. The link to the video is ![MPC at 55 MPH](./MPC_Result_55MPH.webm).

In case the code does not work on the reviewer's machine, please look at both the videos as proof that I have tuned and successfully run the controller based on the VBox-Simulator latency observed on my machine with the parameters tuned based on that delay. I am submitting the final project with the reference speed set to 40 MPH, just to be conservative. Except for changes in the cost function weights for [CTE, HE, Delta rate], the remaining code is identical to the original submission.