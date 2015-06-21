#include "Makeblock.h"

class MeEncoderMotorMod : public MeEncoderMotor {
    public:
      MeEncoderMotorMod(uint8_t addr,uint8_t slot) : MeEncoderMotor(addr, slot) {
        max_rpm = 325;
        tau = 20;
        calibration = 46.0 / tau;
      }
      void RunSpeedMod(float rpm) {
        float rpm_as_46 = constrain(rpm, -max_rpm, max_rpm) * calibration;
        RunSpeed(rpm_as_46);
      }
    private:
      float tau;
      float max_rpm;
      float calibration;
};