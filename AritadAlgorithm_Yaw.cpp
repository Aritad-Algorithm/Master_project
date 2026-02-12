#include "AritadAlgorithm_Yaw.h"

// ================= Constructor =================
AritadAlgorithm_Yaw::AritadAlgorithm_Yaw(ControllerType type) {
  ctrlType = type;
}

// ================= MF =================
float AritadAlgorithm_Yaw::TriangularMF(float x,float l,float m,float r){
  return max(min((x-l)/(m-l),(r-x)/(r-m)),0.0f);
}

float AritadAlgorithm_Yaw::TrapezoidalMF(float x,float lS,float lT,float rT,float rE){
  return max(min(min((x-lS)/(lT-lS),1.0f),(rE-x)/(rE-rT)),0.0f);
}

float AritadAlgorithm_Yaw::GaussianMF(float x,float c,float w){
  return exp(-0.5f*pow((x-c)/w,2));
}

float AritadAlgorithm_Yaw::BellMF(float x,float w,float s,float c){
  return 1.0f/(1.0f+pow(fabs((x-c)/w),2*s));
}

float AritadAlgorithm_Yaw::SigmoidMF(float x,float s,float c){
  return 1.0f/(1.0f+exp(-s*(x-c)));
}

float AritadAlgorithm_Yaw::ZShapeMF(float x,float s,float e){
  if(x<=s) return 1;
  if(x>=e) return 0;
  float t=(x-s)/(e-s);
  return 1-2*t*t;
}

float AritadAlgorithm_Yaw::SShapeMF(float x,float s,float e){
  if(x<=s) return 0;
  if(x>=e) return 1;
  float t=(x-s)/(e-s);
  return 2*t*t;
}

// ================= Output MF =================
float AritadAlgorithm_Yaw::OutputMF_Y_RWS(float x){ return ZShapeMF(x,-3.8,-2); }
float AritadAlgorithm_Yaw::OutputMF_Y_RWM(float x){ return TriangularMF(x,-2.01,-1.5,-0.9); }
float AritadAlgorithm_Yaw::OutputMF_Y_SA(float x){ return TriangularMF(x,-1,0,1); }
float AritadAlgorithm_Yaw::OutputMF_Y_LWM(float x){ return TriangularMF(x,0.9,1.5,2.01); }
float AritadAlgorithm_Yaw::OutputMF_Y_LWS(float x){ return SShapeMF(x,2,3.8); }


// Roll
//float FuzzyController::OutputMF_R_OPFD(float x){ return ZShapeMF(x,-300,-160); }
//float FuzzyController::OutputMF_R_OPLD(float x){ return TriangularMF(x,-170,-50,-35); }
//float FuzzyController::OutputMF_R_ZDEG(float x){ return TriangularMF(x,-20,0,20); }
//float FuzzyController::OutputMF_R_OPLU(float x){ return TriangularMF(x,35,50,170); }
//float FuzzyController::OutputMF_R_OPFU(float x){ return SShapeMF(x,160,300); }


// ================= Output MF Selector =================
// ================= Output MF Selector =================
float AritadAlgorithm_Yaw::evalOutputMF(uint8_t idx, float x){
  if(ctrlType == YAW_CTRL){
    switch(idx){
      case RWS: return OutputMF_Y_RWS(x);
      case RWM: return OutputMF_Y_RWM(x);
      case SA:  return OutputMF_Y_SA(x);
      case LWM: return OutputMF_Y_LWM(x);
      case LWS: return OutputMF_Y_LWS(x);
      default:  return 0;
    }
  }

  return 0;
}

// ================= Compute =================
float AritadAlgorithm_Yaw::compute(float error,float dError,float u_pre){

  float eMF[5], deMF[3];

  // ===== Input MF =====
  if(ctrlType==YAW_CTRL){
    eMF[0]=ZShapeMF(error,-60,-45);
    eMF[1]=GaussianMF(dError,-30,20);
    //eMF[2]=GaussianMF(dError,0,20);
    eMF[2]=GaussianMF(dError,0,10);
    eMF[3]=GaussianMF(dError,30,20);
    eMF[4]=SShapeMF(error,45,60);

    deMF[0]=ZShapeMF(dError,-150,-80);
    deMF[1]=GaussianMF(dError,0,45);
    deMF[2]=SShapeMF(dError,80,150);
  } 

  // ===== RULE BASE =====
  static const RuleCell yawRule[5][3]={
    {{RWS,OUT_MF},{RWS,OUT_MF},{RWM,OUT_MF}},
    {{RWS,OUT_MF},{RWM,OUT_MF},{RWM,OUT_MF}},
    {{RWM,OUT_MF},{SA,OUT_MF},{LWM,OUT_MF}},
    {{LWM,OUT_MF},{LWM,OUT_MF},{LWS,OUT_MF}},
    {{LWM,OUT_MF},{LWS,OUT_MF},{LWS,OUT_MF}}
  };


  const RuleCell (*rule)[3] = yawRule;

  float num=0, den=0;

  if(ctrlType==YAW_CTRL){
    for(int i=0;i<5;i++)
      for(int j=0;j<3;j++)
        if(rule[i][j].type==OUT_CONST){
          float w=min(eMF[i],deMF[j]);
          num+=u_pre*w;
          den+=w;
        }
  }

  float xmax=(ctrlType==YAW_CTRL)?3.8:1;
  float step=(ctrlType==YAW_CTRL)?0.1:1;

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
