#pragma once

namespace bancroft {
  class PID {
    private:
      const float kp {}, ki {}, kd {};
      unsigned const int errorRange {};
      unsigned const short maxSettleTime {};
      unsigned const short maxTimeout {};
      float error = 0;
      float lastError = 0;
      float integral = 0;
      float derivative = 0;
      unsigned short currentSettleTime = 0;
      unsigned short currentTimeout = 0;
    public:
      PID(float kp, float ki, float kd, unsigned int errorRange = 1, unsigned short settleTime = 60, unsigned short timeout = 500);
      double pidControl(double target, double currentLocation);
      void pidReset();
      bool isSettled();
      bool isTimedOut();
  };
}