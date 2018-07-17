# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---


## Describe the effect each of the P, I, D components had in your implementation.

P is used to get bring the car back from large cte to center of the road as fast as possible. But if it is set too large, it will create overshoot.

I is used to eliminate systematic error. Since it is for accumulated error, this coefficient need to be set relatively smaller, or it will cause overshoot too.

D is used to avoid overshoot. It taks the change rate of cte and try to pull the controll from overshooting.  


## Describe how the final hyperparameters were chosen.

I first manually tune the coefficients to make sure the car can run on the track without running away, which makes the initial value is `0.2, 0.002, 3.0`.

Then I start using the implemeted Twiddle algorithm as learned in class in code `PID.cpp`. The initial incremental dK is proportional to the initial coeffients value `0.6*Kp_int, 0.6*Ki_int, 0.3*Kd_int`.

Each evaluation cycle is about 1370 steps, which should be as close as possible to one full lap, in order to get best comparable error.

Then the Triddle algorithm starts the iteration of tuning PID coefficients. after ~790,000 steps, the final coefficients value is `0.644186, 0.021039, 6.5168`.

The log of coefficients updates and best according best error change is in the .xls file.

For speed control, I use a very simple throttle control based on `cte` power 2. With Larger cte, the throttle is released more in order to lowe the speed. I manually tune the factor to be 0.15, which can keep the vehicle speed in 20-32 mph range.


