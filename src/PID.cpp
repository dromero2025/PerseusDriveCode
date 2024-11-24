#include "vex.h"

namespace bancroft {
  // PID::PID(float kp, float ki, float kd) : kp{kp}, ki{ki}, kd{kd} {
  //   errorRange = 1;
  //   maxSettleTime = 60;
  //   maxTimeout = 500;
  // }

  PID::PID(float kp, float ki, float kd, unsigned int errorRange, unsigned short settleTime, unsigned short timeout) : 
  kp{kp}, ki{ki}, kd{kd}, errorRange{errorRange}, maxSettleTime{settleTime}, maxTimeout{timeout} {}

  double PID::pidControl(double target, double currentLocation) {
    if (currentTimeout < maxTimeout) {
      error = target - currentLocation;
      integral += error;
      derivative = error - lastError;
      lastError = error;
      if (fabs(error * kp + integral * ki + derivative * kd) < errorRange) {
        currentSettleTime += (currentSettleTime < maxSettleTime);
      } else {
        currentSettleTime = 0;
      }
      currentTimeout++;
    } else {
      return 0;
    }
    return error * kp + integral * ki + derivative * kd;
  }

  void PID::pidReset() {
    error = 0;
    lastError = 0;
    integral = 0;
    derivative = 0;
    currentSettleTime = 0;
    currentTimeout = 0;
  }

  bool PID::isSettled() {
    return (currentSettleTime >= maxSettleTime) || (currentSettleTime == 0 && error == 0);
  }

  bool PID::isTimedOut() {
    return currentTimeout >= maxTimeout;
  }
}