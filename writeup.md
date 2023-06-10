# Project: Building an Estimator

---

## Requirements for a Passing Submission ([Rubric](https://review.udacity.com/#!/rubrics/1807/view) Points):
1. Provide a Writeup / README that includes all the rubric points and how you they were addressed.
2. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
3. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.
4. Implement all of the elements of the prediction step for the estimator.
5. Implement the magnetometer update.
6. Implement the GPS update.
7. Meet the performance criteria of each step and de-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.

---

### 1. Provide a Writeup / README that includes all the rubric points and how you they were addressed.

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### 2. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

I successfully determined the standard deviation of the measurement noise for both GPS X data and Accelerometer X data by utilizing the logged files and processing them in LibreOffice Calc. 

### 3. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

This function updates the state estimate of the quadcopter using the IMU measurements. The IMU provides measurements of the acceleration and angular velocity of the quadcopter. The acceleration measurements are used to estimate the roll and pitch angles of the quadcopter, while the angular velocity measurements are used to estimate the yaw angle.

The function was improved based on the section "7.1.2 Nonlinear Complementary Filter" of the ["Estimation for Quadrotors"](https://www.overleaf.com/project/5c34caab7ecefc04087273b9) reference paper.  First, a quaternion q_t that represents the current orientation of the quadcopter is created. The FromEuler123_RPY function is then called to make the quaternion from the current roll, pitch, and yaw angles (rollEst, pitchEst, and ekfState(6)).

Next, a new quaternion dq is generated from the angular velocity measurements (gyro) and the time step (dtIMU). This quaternion represents the change in orientation of the quadcopter over the time step.

The q_bar quaternion is then calculated as the product of dq and q_t. This quaternion represents the new orientation of the quadcopter after the time step. The predictedPitch, predictedRoll, and ekfState(6) (which represents the estimated yaw angle) are extracted from q_bar.

The yaw angle is then normalized to be between -pi and pi. This is necessary because the yaw angle can be represented by multiple values that differ by 2*pi, so we need to make sure we choose the correct one.

Finally, the function calculates the roll and pitch estimates by fusing the predicted roll/pitch angles with the accelerometer measurements using a complementary filter. The roll and pitch estimates are updated using the accelRoll and accelPitch variables, respectively.

 ``` c++
    void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
    {
      // Improve a complementary filter-type attitude filter
      // 
      // Currently a small-angle approximation integration method is implemented
      // The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers
      // 
      // Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))
      // to integrate the body rates into new Euler angles.
      //
      // HINTS:
      //  - there are several ways to go about this, including:
      //    1) create a rotation matrix based on your current Euler angles, integrate that, convert back to Euler angles
      //    OR 
      //    2) use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
      //       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)
    
      ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
      // Based on the suggestions provided in https://knowledge.udacity.com/questions/247613
    
      Quaternion<float> q_t = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
      Quaternion<float> dq = Quaternion<float>::FromEuler123_RPY(dtIMU * gyro.x, dtIMU * gyro.y, dtIMU * gyro.z);
      Quaternion<float> q_bar = dq *q_t;
      float predictedPitch = q_bar.Pitch();
      float predictedRoll = q_bar.Roll();
      ekfState(6) = q_bar.Yaw();
    
      // normalize yaw to -pi .. pi
      if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
      if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
    
      /////////////////////////////// END STUDENT CODE ////////////////////////////
    
      // CALCULATE UPDATE
      accelRoll = atan2f(accel.y, accel.z);
      accelPitch = atan2f(-accel.x, 9.81f);
    
      // FUSE INTEGRATION AND UPDATE
      rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll)+dtIMU / (attitudeTau + dtIMU) * accelRoll;
      pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch)+dtIMU / (attitudeTau + dtIMU) * accelPitch;
    
      lastGyro = gyro;
    }
 ```
### 4. Implement all of the elements of the prediction step for the estimator.

This function is used for estimating the state of a quadrotor using an Extended Kalman Filter (EKF). It takes in the current state of the quadrotor, the time step dt, the accelerometer measurements accel, and the gyroscope measurements gyro. It then uses these inputs to predict the next state of the quadrotor after a time step of dt.

The function first creates a Quaternion object called attitude (provided), which represents the orientation of the quadrotor in space. It then rotates the accelerometer measurements from the body frame to the inertial frame using the attitude quaternion, leveraging the operations provided by the Quaternion class.

Next, the function uses a simple integration method to predict the position and velocity of the quadrotor in the x, y, and z directions. It also predicts the acceleration in the x, y, and z directions by adding the inertial acceleration and the gravitational acceleration. 
The function then returns the predicted state of the quadrotor as a vector.

``` c++
VectorXf QuadEstimatorEKF::PredictState(VectorXf curState, float dt, V3F accel, V3F gyro)
{
  assert(curState.size() == QUAD_EKF_NUM_STATES);
  VectorXf predictedState = curState;
  // Predict the current state forward by time dt using current accelerations and body rates as input
  // INPUTS: 
  //   curState: starting state
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //   
  // OUTPUT:
  //   return the predicted state as a vector

  // HINTS 
  // - dt is the time duration for which you should predict. It will be very short (on the order of 1ms)
  //   so simplistic integration methods are fine here
  // - we've created an Attitude Quaternion for you from the current state. Use 
  //   attitude.Rotate_BtoI(<V3F>) to rotate a vector from body frame to inertial frame
  // - the yaw integral is already done in the IMU update. Be sure not to integrate it again here

  Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F acc_inertial = attitude.Rotate_BtoI(accel);

  predictedState (0) = curState(0) + curState(3) * dt; //x
  predictedState (1) = curState(1) + curState(4) * dt; //y
  predictedState (2) = curState(2) + curState(5) * dt; //z
  predictedState (3) = curState(3) + acc_inertial.x * dt;
  predictedState (4) = curState(4) + acc_inertial.y * dt;
  predictedState (5) = curState(5) + (acc_inertial.z - CONST_GRAVITY) * dt ; // do not add the gravity effects
  predictedState (6) = curState(6) ;//+  omega_inertial.z *dt; //yaw must not be integrated

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return predictedState;
}
```

The next function calculates the partial derivative of the rotation matrix Rbg with respect to the yaw angle. The rotation matrix Rbg represents the transformation from the body frame to the global frame, and it depends on the roll, pitch, and yaw angles of the quadrotor.

The function takes in the roll, pitch, and yaw angles as inputs and returns a 3x3 matrix representing the partial derivative of Rbg with respect to the yaw angle. The function first initializes a 3x3 matrix called RbgPrime and sets all its elements to zero.

Next, the elements of RbgPrime are computed according to the reference paper. Finally, the function returns the matrix RbgPrime. 


``` c++
MatrixXf QuadEstimatorEKF::GetRbgPrime(float roll, float pitch, float yaw)
{
  // first, figure out the Rbg_prime
  MatrixXf RbgPrime(3, 3);
  RbgPrime.setZero();

  // Return the partial derivative of the Rbg rotation matrix with respect to yaw. We call this RbgPrime.
  // INPUTS: 
  //   roll, pitch, yaw: Euler angles at which to calculate RbgPrime
  //   
  // OUTPUT:
  //   return the 3x3 matrix representing the partial derivative at the given point

  // HINTS
  // - this is just a matter of putting the right sin() and cos() functions in the right place.
  //   make sure you write clear code and triple-check your math
  // - You can also do some numerical partial derivatives in a unit test scheme to check 
  //   that your calculations are reasonable

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  ///
  RbgPrime(0,0) = -cos(pitch) * sin(yaw);
  RbgPrime(0,1) = -sin(roll) * sin(pitch) * sin(yaw) - cos(roll) * cos(yaw);
  RbgPrime(0,2) = -cos(roll) * sin(pitch) * sin(yaw) + sin(roll) * cos(yaw);
  RbgPrime(1,0) = cos(pitch) * cos(yaw);
  RbgPrime(1,1) = sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw);
  RbgPrime(1,2) = cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);
  RbgPrime(2,0) = 0;
  RbgPrime(2,1) = 0;
  RbgPrime(2,2) = 0;


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return RbgPrime;
}
```

The primary predict function takes the time step dt, the accelerometer measurements accel, and the gyroscope measurements gyro as inputs.

The function first calls the PredictState function to predict the state of the quadrotor forward in time. It then updates the covariance matrix of the EKF using the predicted state and the current covariance matrix.

The covariance matrix is updated using the EKF equation, which involves calculating the Jacobian matrix of the transition model and the transition model covariance matrix. The Jacobian matrix is represented by the matrix gPrime, and it depends on the current state of the quadrotor. The transition model covariance matrix is represented by the matrix Q, and it is loaded from a parameter file.

The function first calculates the partial derivative of the rotation matrix Rbg with respect to the yaw angle using the GetRbgPrime function. It then initializes a Jacobian matrix called gPrime and sets it to the identity matrix.

Finally, the function calculates the necessary helper matrices and builds up the transition Jacobian. It then updates the covariance matrix using the EKF equation.

``` c++
void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
  // predict the state forward
  VectorXf newState = PredictState(ekfState, dt, accel, gyro);

  // Predict the current covariance forward by dt using the current accelerations and body rates as input.
  // INPUTS: 
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //   state (member variable): current state (state at the beginning of this prediction)
  //   
  // OUTPUT:
  //   update the member variable cov to the predicted covariance

  // HINTS
  // - update the covariance matrix cov according to the EKF equation.
  // 
  // - you may find the current estimated attitude in variables rollEst, pitchEst, state(6).
  //
  // - use the class MatrixXf for matrices. To create a 3x5 matrix A, use MatrixXf A(3,5).
  //
  // - the transition model covariance, Q, is loaded up from a parameter file in member variable Q
  // 
  // - This is unfortunately a messy step. Try to split this up into clear, manageable steps:
  //   1) Calculate the necessary helper matrices, building up the transition jacobian
  //   2) Once all the matrices are there, write the equation to update cov.
  //
  // - if you want to transpose a matrix in-place, use A.transposeInPlace(), not A = A.transpose()
  // 

  // we'll want the partial derivative of the Rbg matrix
  MatrixXf RbgPrime = GetRbgPrime(rollEst, pitchEst, ekfState(6));

  // we've created an empty Jacobian for you, currently simply set to identity
  MatrixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
  gPrime.setIdentity();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  MatrixXf u(3, 1);
  u(0,0) = accel.x * dt;
  u(1,0) = accel.y * dt;
  u(2,0) = accel.z * dt;
  MatrixXf helper = RbgPrime * u;

  gPrime(0,3) = dt;
  gPrime(1,4) = dt;
  gPrime(2,5) = dt;
  gPrime(6,3) = helper(0,0);
  gPrime(6,4) = helper(1,0);
  gPrime(6,5) = helper(2,0);

  MatrixXf prevCov = ekfCov;
  ekfCov = gPrime  * prevCov *  gPrime.transpose() + Q;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  ekfState = newState;
}
```

### 5. Implement the magnetometer update.

This function updates the state estimate of the quadcopter's orientation using a magnetometer measurement. The function takes in a single argument, magYaw, which is the measured yaw angle from the magnetometer.

The function first creates a vector z with the measured yaw angle, and a vector zFromX with the estimated yaw angle from the current state estimate. It then calculates the difference between the measured and estimated yaw angles, and normalizes the difference to ensure that the update is performed in the shortest direction around the circle.

Next, the function creates a matrix hPrime with the partial derivative of the measurement model with respect to the state vector. In this case, the measurement model is simply the yaw angle, so the partial derivative is a one-dimensional vector with a one in the 6th position (corresponding to the yaw angle in the state vector) and zeros elsewhere.

Finally, the function calls the Update() function with the measurement vector z, the measurement model partial derivative matrix hPrime, the measurement covariance matrix R_Mag, and the estimated measurement vector zFromX. The Update() function performs the Kalman filter update step to update the state estimate based on the magnetometer measurement.

``` c++
void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{
  VectorXf z(1), zFromX(1);
  z(0) = magYaw;

  MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);
  hPrime.setZero();

  // MAGNETOMETER UPDATE
  // Hints: 
  //  - Your current estimated yaw can be found in the state vector: ekfState(6)
  //  - Make sure to normalize the difference between your measured and estimated yaw
  //    (you don't want to update your yaw the long way around the circle)
  //  - The magnetomer measurement covariance is available in member variable R_Mag
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  zFromX(0) = ekfState(6);

  float yawError = magYaw - ekfState(6);

  if (yawError > F_PI) zFromX(0) += 2.f * F_PI;
  else if (yawError < -F_PI) zFromX(0) -= 2.f * F_PI;

  hPrime(0,6) = 1.0f;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  Update(z, hPrime, R_Mag, zFromX);
}
```
### 6. Implement the GPS update.

This function updates the state estimate of the quadcopter's position and velocity using GPS measurements. The function takes in two arguments, pos and vel, which are the measured position and velocity vectors from the GPS.

The function first creates a vector z with the measured position and velocity, and a vector zFromX with the estimated position and velocity from the current state estimate. It then creates a matrix hPrime with the partial derivative of the measurement model with respect to the state vector. In this case, the measurement model is simply the position and velocity, so the partial derivative is a six-dimensional matrix with ones on the diagonal and zeros elsewhere.

Finally, the function calls the Update() function with the measurement vector z, the measurement model partial derivative matrix hPrime, the measurement covariance matrix R_GPS, and the estimated measurement vector zFromX. The Update() function performs the Kalman filter update step to update the state estimate based on the GPS measurement.

```c++
void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
  VectorXf z(6), zFromX(6);
  z(0) = pos.x;
  z(1) = pos.y;
  z(2) = pos.z;
  z(3) = vel.x;
  z(4) = vel.y;
  z(5) = vel.z;

  MatrixXf hPrime(6, QUAD_EKF_NUM_STATES);
  hPrime.setZero();

  // GPS UPDATE
  // Hints: 
  //  - The GPS measurement covariance is available in member variable R_GPS
  //  - this is a very simple update
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  zFromX(0) = ekfState(0);
  zFromX(1) = ekfState(1);
  zFromX(2) = ekfState(2);
  zFromX(3) = ekfState(3);
  zFromX(4) = ekfState(4);
  zFromX(5) = ekfState(5);

  hPrime(0,0) = 1;
  hPrime(1,1) = 1;
  hPrime(2,2) = 1;
  hPrime(3,3) = 1;
  hPrime(4,4) = 1;
  hPrime(5,5) = 1;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  Update(z, hPrime, R_GPS, zFromX);
}
```

### 7. Meet the performance criteria of each step and de-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.

It works! The same set of gains work for all the scenarios (included the scenarios from the previous project).



  


