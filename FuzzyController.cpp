#include "FuzzyController.h"

// ================= Constructor =================
FuzzyController::FuzzyController(ControllerType type) {
  ctrlType = type;
}

// ================= MF =================
float FuzzyController::TriangularMF(float x,float l,float m,float r){
  return max(min((x-l)/(m-l),(r-x)/(r-m)),0.0f);
}

float FuzzyController::TrapezoidalMF(float x,float lS,float lT,float rT,float rE){
  return max(min(min((x-lS)/(lT-lS),1.0f),(rE-x)/(rE-rT)),0.0f);
}

float FuzzyController::GaussianMF(float x,float c,float w){
  return exp(-0.5f*pow((x-c)/w,2));
}

float FuzzyController::BellMF(float x,float w,float s,float c){
  return 1.0f/(1.0f+pow(fabs((x-c)/w),2*s));
}

float FuzzyController::SigmoidMF(float x,float s,float c){
  return 1.0f/(1.0f+exp(-s*(x-c)));
}

float FuzzyController::ZShapeMF(float x,float s,float e){
  if(x<=s) return 1;
  if(x>=e) return 0;
  float t=(x-s)/(e-s);
  return 1-2*t*t;
}

float FuzzyController::SShapeMF(float x,float s,float e){
  if(x<=s) return 0;
  if(x>=e) return 1;
  float t=(x-s)/(e-s);
  return 2*t*t;
}

// ================= Output MF =================
float FuzzyController::OutputMF_H_OPFD(float x){ return ZShapeMF(x,-400,-150); }
float FuzzyController::OutputMF_H_OPLD(float x){ return TriangularMF(x,-250,-120,-30); }
float FuzzyController::OutputMF_H_ZDEG(float x){ return TriangularMF(x,-40,0,40); }
float FuzzyController::OutputMF_H_OPLU(float x){ return TriangularMF(x,30,120,250); }
float FuzzyController::OutputMF_H_OPFU(float x){ return SShapeMF(x,150,400); }

float FuzzyController::OutputMF_P_OPFD(float x){ return ZShapeMF(x,-80,-30); }
float FuzzyController::OutputMF_P_OPLD(float x){ return TriangularMF(x,-50,-25,-5); }
float FuzzyController::OutputMF_P_ZDEG(float x){ return TriangularMF(x,-5,0,5); }
float FuzzyController::OutputMF_P_OPLU(float x){ return TriangularMF(x,5,25,50); }
float FuzzyController::OutputMF_P_OPFU(float x){ return SShapeMF(x,30,80); }

float FuzzyController::evalOutputMF(uint8_t idx,float x){
  if(ctrlType==HEIGHT_CTRL){
    switch(idx){
      case OPFD:return OutputMF_H_OPFD(x);
      case OPLD:return OutputMF_H_OPLD(x);
      case ZDEG:return OutputMF_H_ZDEG(x);
      case OPLU:return OutputMF_H_OPLU(x);
      case OPFU:return OutputMF_H_OPFU(x);
    }
  } else {
    switch(idx){
      case OPFD:return OutputMF_P_OPFD(x);
      case OPLD:return OutputMF_P_OPLD(x);
      case ZDEG:return OutputMF_P_ZDEG(x);
      case OPLU:return OutputMF_P_OPLU(x);
      case OPFU:return OutputMF_P_OPFU(x);
    }
  }
  return 0;
}

// ================= Compute =================
float FuzzyController::compute(float error,float dError,float u_pre){

  float eMF[5], deMF[3];

  // ===== Input MF =====
  if(ctrlType==HEIGHT_CTRL){
    eMF[0]=ZShapeMF(error,-2,-1);
    eMF[1]=TriangularMF(error,-1.5,-0.2,0);
    eMF[2]=TrapezoidalMF(error,-0.3,-0.1,0.1,0.3);
    eMF[3]=TriangularMF(error,0,0.2,1.5);
    eMF[4]=SShapeMF(error,1,2);

    deMF[0]=ZShapeMF(dError,-40,-20);
    deMF[1]=GaussianMF(dError,0,15);
    deMF[2]=SShapeMF(dError,20,40);
  } else {
    eMF[0]=ZShapeMF(error,-0.5,-0.25);
    eMF[1]=BellMF(error,0.3,2,-0.2);
    eMF[2]=GaussianMF(error,0,0.03);
    eMF[3]=BellMF(error,0.3,2,0.2);
    eMF[4]=SShapeMF(error,0.25,0.5);

    deMF[0]=ZShapeMF(dError,-2,-1);
    deMF[1]=GaussianMF(dError,0,0.6);
    deMF[2]=SigmoidMF(dError,2,1);
  }

  // ===== RULE BASE =====
  static const RuleCell heightRule[5][3]={
    {{OPFD,OUT_MF},{OPFD,OUT_MF},{OPFD,OUT_MF}},
    {{OPLD,OUT_MF},{OPLD,OUT_MF},{OPFD,OUT_MF}},
    {{OPLD,OUT_MF},{ZDEG,OUT_MF},{OPLD,OUT_MF}},
    {{OPLU,OUT_MF},{OPLU,OUT_MF},{0,OUT_CONST}},
    {{OPFU,OUT_MF},{OPFU,OUT_MF},{OPLU,OUT_MF}}
  };

  static const RuleCell pitchRule[5][3]={
    {{OPFD,OUT_MF},{OPLD,OUT_MF},{ZDEG,OUT_MF}},
    {{OPLD,OUT_MF},{ZDEG,OUT_MF},{ZDEG,OUT_MF}},
    {{ZDEG,OUT_MF},{ZDEG,OUT_MF},{ZDEG,OUT_MF}},
    {{OPLU,OUT_MF},{ZDEG,OUT_MF},{OPLD,OUT_MF}},
    {{OPFU,OUT_MF},{OPLU,OUT_MF},{OPLD,OUT_MF}}
  };

  const RuleCell (*rule)[3] =
    (ctrlType==HEIGHT_CTRL)?heightRule:pitchRule;

  float num=0, den=0;

  if(ctrlType==HEIGHT_CTRL){
    for(int i=0;i<5;i++)
      for(int j=0;j<3;j++)
        if(rule[i][j].type==OUT_CONST){
          float w=min(eMF[i],deMF[j]);
          num+=u_pre*w;
          den+=w;
        }
  }

  float xmax=(ctrlType==HEIGHT_CTRL)?400:80;
  float step=(ctrlType==HEIGHT_CTRL)?40:5;

  for(float x=-xmax;x<=xmax;x+=step){
    float mu=0;
    for(int i=0;i<5;i++)
      for(int j=0;j<3;j++)
        if(rule[i][j].type==OUT_MF){
          float w=min(eMF[i],deMF[j]);
          mu=max(mu,min(w,evalOutputMF(rule[i][j].outIdx,x)));
        }
    if(mu>0){ num+=x*mu; den+=mu; }
    yield();
  }

  return (den==0)?u_pre:(num/den);
}
