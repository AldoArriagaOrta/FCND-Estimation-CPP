#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //
  //  The following system of equations is solved for F1,F2,F3 and  F4
  //  F1 + F2 + F3 + F4 = F_total
  //  F1+ F4 - F2 - F3 = tau_x / L
  //  F1 + F2 - F3 - F4 = tau_y / L
  //  F1 - F2 + F3 -F4 = tau_z / kappa
  //  Important note: the signs in the tau_z equation are inverted in the lesson:
  //  https://knowledge.udacity.com/questions/251050
  //Common coefficients and division by zero checks
  float l = L / (2.f * sqrtf(2.f));
  float invDivisor1 = (l != 0.0f) ? (1.0f / (4.0f * l)) : 0.0f;
  float invDivisor2 = (kappa != 0.0f) ? (1.0f / (4.0f * kappa)) : 0.0f;

  float term1 = collThrustCmd / 4.0f;
  float term2 = momentCmd.x * invDivisor1;
  float term3 = momentCmd.y * invDivisor1;
  float term4 = momentCmd.z * invDivisor2;

  cmd.desiredThrustsN[0] = term1 + term2 + term3 + term4; //front left
  cmd.desiredThrustsN[1] = term1 - term2 + term3 - term4; //front right
  cmd.desiredThrustsN[2] = term1 + term2 - term3 - term4; //rear right
  cmd.desiredThrustsN[3] = term1 - term2 - term3 + term4; //rear left


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

//  const V3F Ixyz = V3F(Ixx, Iyy, -Izz);
//  momentCmd = Ixyz * kpPQR * (pqrCmd - pqr);

  V3F error;
  error = pqrCmd - pqr;
  momentCmd[0] = Ixx * kpPQR.x * error[0];
  momentCmd[1] = Iyy * kpPQR.y * error[1];
  momentCmd[2] = -Izz * kpPQR.z * error[2];

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  pqrCmd = V3F(0.f, 0.f, 0.f);
  float c = -collThrustCmd /mass;

  if (collThrustCmd > 0.0)
  {
      float r13Cmd = CONSTRAIN(accelCmd.x / c, -maxTiltAngle , maxTiltAngle);
      float r23Cmd = CONSTRAIN(accelCmd.y / c, -maxTiltAngle , maxTiltAngle);

      pqrCmd.x = (1/R(2, 2)) * (-R(1, 0) * kpBank * (R(0, 2) - r13Cmd) + R(0, 0) * kpBank * (R(1, 2) - r23Cmd));
      pqrCmd.y = (1/R(2, 2)) * (-R(1, 1) * kpBank * (R(0, 2) - r13Cmd) + R(0, 1) * kpBank * (R(1, 2) - r23Cmd));
   }
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  pqrCmd.z = 0.f;

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  static float iZPosError = 0.0f; // Declare integral error as a static variable to maintain its value between function calls
  float zPosError = posZCmd - posZ;
  const float antiWindup = 0.1f;
  const float g = 9.81f;

  iZPosError+= zPosError * dt;

  iZPosError = CONSTRAIN(iZPosError, -antiWindup, antiWindup); // Clamp the integral error as windup protection

  velZCmd  = CONSTRAIN(velZCmd , -maxDescentRate, maxAscentRate);

  thrust = -mass * (kpPosZ * zPosError + KiPosZ * iZPosError + kpVelZ * (velZCmd - velZ) + accelZCmd - g )/R(2,2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  const V3F kpPos = V3F(kpPosXY, kpPosXY,0.f);
  const V3F kpVel = V3F(kpVelXY, kpVelXY, 0.f);

  if (velCmd.mag() > maxSpeedXY) {
      velCmd = velCmd.norm() * maxSpeedXY;
  }
  else {
      //do nothing
  }

  //PD with FF acc
  accelCmd += kpPos * (posCmd - pos) + kpVel * (velCmd - vel);

  if (accelCmd.mag() > maxAccelXY) {
      accelCmd= accelCmd.norm() * maxAccelXY;
  }
  else {
      //do nothing
  }

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  //float cappedYawCmd = fmodf(yawCmd, 2 * F_PI);
  float cappedYaw = fmodf(yaw, 2 * F_PI);
  float cappedYawCmd = 0;
//  float cappedYawCmd =fmodf(yawCmd, 2 * F_PI);;

  if ( yawCmd > 0 )
  {
    cappedYawCmd = fmodf(yawCmd, 2 * F_PI);
  }
  else
  {
    cappedYawCmd = fmodf(-yawCmd, 2 * F_PI);
  }
  //float error = cappedYawCmd - cappedYaw;
  float error = yawCmd - yaw;

  if (error > F_PI)
  {
      error -= 2 * F_PI;
  }
  else if (error< -F_PI)
  {
      error += 2 * F_PI;
  }
  yawRateCmd=  kpYaw * error;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
