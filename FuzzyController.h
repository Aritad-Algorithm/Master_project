#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

#include <Arduino.h>
#include <math.h>

class FuzzyController {
public:
  enum ControllerType {
    HEIGHT_CTRL,
    PITCH_CTRL,
    ROLL_CTRL
  };

  FuzzyController(ControllerType type);
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
  static float OutputMF_H_OPFD(float x);
  static float OutputMF_H_OPLD(float x);
  static float OutputMF_H_ZDEG(float x);
  static float OutputMF_H_OPLU(float x);
  static float OutputMF_H_OPFU(float x);

  static float OutputMF_P_OPFD(float x);
  static float OutputMF_P_OPLD(float x);
  static float OutputMF_P_ZDEG(float x);
  static float OutputMF_P_OPLU(float x);
  static float OutputMF_P_OPFU(float x);

  static float OutputMF_R_OPFD(float x);
  static float OutputMF_R_OPLD(float x);
  static float OutputMF_R_ZDEG(float x);
  static float OutputMF_R_OPLU(float x);
  static float OutputMF_R_OPFU(float x);

  enum OutputMF { OPFD, OPLD, ZDEG, OPLU, OPFU };
  enum OutputType { OUT_MF, OUT_CONST };

  struct RuleCell {
    uint8_t outIdx;
    OutputType type;
  };

  float evalOutputMF(uint8_t idx, float x);
};

#endif
