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
float FuzzyController::OutputMF_H_OPFD(float x){ return ZShapeMF(x,-400,-140); }
float FuzzyController::OutputMF_H_OPLD(float x){ return TriangularMF(x,-250,-100,-20); }
float FuzzyController::OutputMF_H_ZDEG(float x){ return TriangularMF(x,-40,0,40); }
float FuzzyController::OutputMF_H_OPLU(float x){ return TriangularMF(x,20,100,250); }
float FuzzyController::OutputMF_H_OPFU(float x){ return SShapeMF(x,140,400); }

float FuzzyController::OutputMF_P_OPFD(float x){ return ZShapeMF(x,-400,-140); }
float FuzzyController::OutputMF_P_OPLD(float x){ return GaussianMF(x, -150, 100); }
float FuzzyController::OutputMF_P_ZDEG(float x){ return GaussianMF(x,   0, 100);  }
float FuzzyController::OutputMF_P_OPLU(float x){ return GaussianMF(x, 150, 100);  }
float FuzzyController::OutputMF_P_OPFU(float x){ return SShapeMF(x,140,400);  }

// Roll
//float FuzzyController::OutputMF_R_OPFD(float x){ return ZShapeMF(x,-300,-160); }
//float FuzzyController::OutputMF_R_OPLD(float x){ return TriangularMF(x,-170,-50,-35); }
//float FuzzyController::OutputMF_R_ZDEG(float x){ return TriangularMF(x,-20,0,20); }
//float FuzzyController::OutputMF_R_OPLU(float x){ return TriangularMF(x,35,50,170); }
//float FuzzyController::OutputMF_R_OPFU(float x){ return SShapeMF(x,160,300); }

float FuzzyController::OutputMF_R_OPFD(float x){ return ZShapeMF(x,-300,-130); }
float FuzzyController::OutputMF_R_OPLD(float x){ return TriangularMF(x,-140,-50,-19); }
float FuzzyController::OutputMF_R_ZDEG(float x){ return GaussianMF(x,   0, 20);}
float FuzzyController::OutputMF_R_OPLU(float x){ return TriangularMF(x,19,50,140); }
float FuzzyController::OutputMF_R_OPFU(float x){ return SShapeMF(x,130,300); }

// ================= Output MF Selector =================
float FuzzyController::evalOutputMF(uint8_t idx,float x){
  if(ctrlType==HEIGHT_CTRL){
    switch(idx){
      case OPFD:return OutputMF_H_OPFD(x);
      case OPLD:return OutputMF_H_OPLD(x);
      case ZDEG:return OutputMF_H_ZDEG(x);
      case OPLU:return OutputMF_H_OPLU(x);
      case OPFU:return OutputMF_H_OPFU(x);
    }
  }
  else if(ctrlType==PITCH_CTRL){
    switch(idx){
      case OPFD:return OutputMF_P_OPFD(x);
      case OPLD:return OutputMF_P_OPLD(x);
      case ZDEG:return OutputMF_P_ZDEG(x);
      case OPLU:return OutputMF_P_OPLU(x);
      case OPFU:return OutputMF_P_OPFU(x);
    }
  }
  else{
    switch(idx){
      case OPFD:return OutputMF_R_OPFD(x);
      case OPLD:return OutputMF_R_OPLD(x);
      case ZDEG:return OutputMF_R_ZDEG(x);
      case OPLU:return OutputMF_R_OPLU(x);
      case OPFU:return OutputMF_R_OPFU(x);
    }
  }
  return 0;
}

// ================= Compute =================
float FuzzyController::compute(float error,float dError,float u_pre){

  float eMF[5], deMF[3];

  // ===== Input MF =====
  if(ctrlType==HEIGHT_CTRL){
//    eMF[0]=ZShapeMF(error,-2,-1);
//    eMF[1]=TriangularMF(error,-1.5,-0.2,0);
//    eMF[2]=TriangularMF(error,-0.25,0,0.25);
//    eMF[3]=TriangularMF(error,0,0.2,1.5);
//    eMF[4]=SShapeMF(error,1,2);
    eMF[0]=ZShapeMF(error,-2,-1);
    eMF[1]=GaussianMF(error, -1, 0.6);
    eMF[2]=GaussianMF(error, 0, 0.5);
    eMF[3]=GaussianMF(error, -1, 0.6);
    eMF[4]=SShapeMF(error,1,2);

    deMF[0]=ZShapeMF(dError,-40,-20);
    //deMF[1]=TriangularMF(dError,-25,0,25);
    deMF[1]=GaussianMF(dError, 0, 50);
    deMF[2]=SShapeMF(dError,20,40);
  }
  else if(ctrlType==PITCH_CTRL){
    eMF[0]=ZShapeMF(error,-60,-45);
    //eMF[1]=TriangularMF(error,-50,-10,-5);
    eMF[1]=GaussianMF(error, -15, 20);
    eMF[2]=GaussianMF(error, 0, 10);
    //eMF[3]=TriangularMF(error,5,10,50);
    eMF[3]=GaussianMF(error, 15, 20);
    eMF[4]=SShapeMF(error,45,60);

    deMF[0]=ZShapeMF(dError,-100,-19);
    ///deMF[1]=TriangularMF(dError,-20,0,20);////
    deMF[1]=GaussianMF(dError, 0, 40);
    deMF[2]=SShapeMF(dError,19,100);
  }
  else{ // ===== ROLL =====
    eMF[0]=ZShapeMF(error,-100,-30);
    //eMF[1]=TriangularMF(error,-35,-20,-8);
    eMF[1]=GaussianMF(error, -20, 15);
    //eMF[2]=TriangularMF(error,-10,0,10);
    eMF[2]=GaussianMF(error, 0, 10);
    //eMF[3]=TriangularMF(error,8,20,35);
    eMF[3]=GaussianMF(error, 20, 15);
    eMF[4]=SShapeMF(error,35,100);

    deMF[0]=deMF[1]=deMF[2]=1.0;
  }

  // ===== RULE BASE =====
//  static const RuleCell heightRule[5][3]={
//    {{OPFD,OUT_MF},{OPFD,OUT_MF},{OPLD,OUT_MF}},
//    {{OPFD,OUT_MF},{OPLD,OUT_MF},{0,OUT_CONST}},
//    {{OPLD,OUT_MF},{ZDEG,OUT_MF},{OPLU,OUT_MF}},
//    {{0,OUT_CONST},{OPLU,OUT_MF},{OPFU,OUT_MF}},
//    {{OPLU,OUT_MF},{OPFU,OUT_MF},{OPFU,OUT_MF}}
//  };
  static const RuleCell heightRule[5][3]={
    {{OPFD,OUT_MF},{OPFD,OUT_MF},{OPLD,OUT_MF}},
    {{OPFD,OUT_MF},{OPLD,OUT_MF},{OPLD,OUT_MF}},
    {{OPLD,OUT_MF},{ZDEG,OUT_MF},{OPLU,OUT_MF}},
    {{OPLU,OUT_MF},{OPLU,OUT_MF},{OPFU,OUT_MF}},
    {{OPLU,OUT_MF},{OPFU,OUT_MF},{OPFU,OUT_MF}}
  };
  static const RuleCell pitchRule[5][3]={
    {{OPFD,OUT_MF},{OPFD,OUT_MF},{OPLD,OUT_MF}},
    {{OPFD,OUT_MF},{OPLD,OUT_MF},{OPLD,OUT_MF}},
    {{OPLD,OUT_MF},{ZDEG,OUT_MF},{OPLU,OUT_MF}},
    {{OPLU,OUT_MF},{OPLU,OUT_MF},{OPFU,OUT_MF}},
    {{OPLU,OUT_MF},{OPFU,OUT_MF},{OPFU,OUT_MF}}
  };

  static const RuleCell rollRule[5][3]={
    {{OPFD,OUT_MF},{OPFD,OUT_MF},{OPFD,OUT_MF}},
    {{OPLD,OUT_MF},{OPLD,OUT_MF},{OPLD,OUT_MF}},
    {{ZDEG,OUT_MF},{ZDEG,OUT_MF},{ZDEG,OUT_MF}},
    {{OPLU,OUT_MF},{OPLU,OUT_MF},{OPLU,OUT_MF}},
    {{OPFU,OUT_MF},{OPFU,OUT_MF},{OPFU,OUT_MF}}
  };

  const RuleCell (*rule)[3];
  if(ctrlType==HEIGHT_CTRL)      rule=heightRule;
  else if(ctrlType==PITCH_CTRL) rule=pitchRule;
  else                          rule=rollRule;

  float num=0, den=0;

  if(ctrlType==HEIGHT_CTRL || ctrlType==PITCH_CTRL){
    for(int i=0;i<5;i++)
      for(int j=0;j<3;j++)
        if(rule[i][j].type==OUT_CONST){
          float w=min(eMF[i],deMF[j]);
          num+=u_pre*w;
          den+=w;
        }
  }

  float xmax, step;
  if(ctrlType==HEIGHT_CTRL){ xmax=400; step=40; }
  else if(ctrlType==PITCH_CTRL){ xmax=400; step=5; }
  else{ xmax=200; step=5; }

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
  if(den == 0){
    return u_pre;   // <<< 핵심
  }
  return (den==0)?0:num/den;
}
