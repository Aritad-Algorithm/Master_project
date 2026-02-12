#ifndef ARITAD_ALGORITHM_YAW_H
#define ARITAD_ALGORITHM_YAW_H

#include <Arduino.h>
#include <math.h>

class AritadAlgorithm_Yaw {
public:
  enum ControllerType {
    YAW_CTRL,
    
  };

  AritadAlgorithm_Yaw(ControllerType type);
  float compute(float error, float dError, float u_pre);

private:
  ControllerType ctrlType;

  // ===== Input MF =====
  static float TriangularMF(float x, float l, float m, float r);
  static float TrapezoidalMF(float x, float lS, float lT, float rT, float rE);
  static float GaussianMF(float x, float c, float w);
  static float BellMF(float x, float w, float s, float c);
  static float SigmoidMF(float x, float s, float c);
  static float ZShapeMF(float x, float s, float e);
  static float SShapeMF(float x, float s, float e);

  // ===== Output MF =====
  static float OutputMF_Y_RWS(float x);
  static float OutputMF_Y_RWM(float x);
  static float OutputMF_Y_SA(float x);
  static float OutputMF_Y_LWM(float x);
  static float OutputMF_Y_LWS(float x);

 

  enum OutputMF { RWS, RWM, SA, LWM, LWS};
  enum OutputType { OUT_MF, OUT_CONST };

  struct RuleCell {
    uint8_t outIdx;
    OutputType type;
  };

  float evalOutputMF(uint8_t idx, float x);
};

#endif
