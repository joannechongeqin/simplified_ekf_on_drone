# Simplified Extended Kalman Filter on a Drone
## Features
- Estimator: Extended Kalman Filter (EKF)
  - EKF Prediction on X-axis, Y-axis, Z-axis and Yaw angle based on the simplified INS model
  - EKF Correction using sensors (GPS, Magnetometer, Sonar, Barometer)
- Controller: pure pursuit
- Smoother: straight line interpolation

## How to Run
```shell
git clone https://github.com/joannechongeqin/simplified_ekf_on_drone.git
cd simplified_ekf_on_drone
. bd.sh
. run.sh
```
