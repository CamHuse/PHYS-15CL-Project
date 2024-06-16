## Files

- `PIDPendulumControl.ino`: Implementation of PID control for the inverted pendulum.
- `README.md`: Documentation file.
  
## PID Control

Proportional-Integral-Derivative (PID) control is a feedback mechanism widely used in control systems. It calculates an error value as the difference between a desired setpoint and a measured process variable. The controller attempts to minimize the error by adjusting the process using three terms: proportional $(K_p)$, integral $(K_i)$, and derivative $(K_d)$.

### PID Control Implementation 

The control output $u(t)$, used to control the speed of the stepper motor, is given by:

$$ u(t) = K_p e(t) + K_i \int e(t)  dt + K_d \frac{d e(t)}{d t} $$

Where:
- $e(t)$ is the error at time $t$.
- $K_p$ is the proportional gain.
- $K_i$ is the integral gain.
- $K_d$ is the derivative gain.
  
Note: The parameters \( K_p \), \( K_i \), \( K_d \) defined in `PIDPendulumControl.ino` are specific to our cart-pendulum system with a pendulum mass and weight particular to our system, so they may not be directly applicable to other systems without retuning. Adjust these parameters based on your specific system's dynamics to achieve optimal control performance.
