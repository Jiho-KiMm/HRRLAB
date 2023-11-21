/*
 * NumericalTool.h
 *
 *  Created on: 2021. 7. 1.
 *      Author: KimKyungHwan
 */

#ifndef NUMERICALTOOL_H_
#define NUMERICALTOOL_H_

class NumericalTool{
private:
public:
        NumericalTool();
        virtual ~NumericalTool();
        class SimpleMovingAverage{
        private:
            int LEN;
            double x[100];
            double y;
            double y_old;
            double avg;
            void Shift(double value_in);
        public:
            SimpleMovingAverage(int len=3);
            virtual ~SimpleMovingAverage();
            double Filter(double value_in,int len=3);

        };
        class Calculus{
        private:
            double dt,inv_dt;
            double x,x_old,x_old2;
            char init_plag;
            double xdot,xddot;
            double Sx;

        public:
            Calculus(double _dt=0.001);
            double Diff(const double& _x);
            double Inte(const double& _x);
            void SetDiffnInte(const double& _x);
            double GetDiff(){return xdot;}
            double GetInte(){return Sx;}
            void Reset(){init_plag=0;Sx=0;}
        };
        class LowPassFilter{
        private:
            double output;
            double pre_output;
            char init_plag;
            double pi2;
        public:
            LowPassFilter():init_plag(0){pi2=6.28318530718;}
            double Filter(double input, double f_c,double t_s=0.001);
        };


};
NumericalTool::NumericalTool(){

}
NumericalTool::~NumericalTool() {
        // TODO Auto-generated destructor stub
}
//**********SimpleMovingAverage**********//
NumericalTool::SimpleMovingAverage::SimpleMovingAverage(int len) {
        // TODO Auto-generated constructor stub
  if(len>100) len=100;
  LEN=len;
  for(int i=0;i<LEN;i++) x[i]=0.;
  y=0.;
  y_old=0.;
}
NumericalTool::SimpleMovingAverage::~SimpleMovingAverage() {
        // TODO Auto-generated destructor stub
}
void NumericalTool::SimpleMovingAverage::Shift(double value_in){
        for(int i=0;i<LEN-1;i++) x[i]=x[i+1];
        x[LEN-1]=value_in;
}
double NumericalTool::SimpleMovingAverage::Filter(double value_in,int len){
		LEN=len;
        y=y_old+(value_in-x[0])/LEN;
        y_old=y;
        Shift(value_in);
        return y;
}

//**********Calculus**********//
NumericalTool::Calculus::Calculus(double _dt){
    dt=_dt;
    inv_dt=1./dt;
    init_plag=0;
    Sx=0;
}
double NumericalTool::Calculus::Diff(const double &_x){
//    if(init_plag==0) {x_old=_x;init_plag=1;}
//    x=_x;
//    xdot=(x-x_old)*inv_dt;
//    x_old=x;
//    return xdot;

    if(init_plag==0) {x_old2=x_old=_x;init_plag=1;}
    x=_x;
    xdot=(3*x-4*x_old+x_old2)*0.5*inv_dt;
    x_old2=x_old;
    x_old=x;
    return xdot;
}
double NumericalTool::Calculus::Inte(const double &_x){
    if(init_plag==0) {x_old2=x_old=_x;init_plag=1;}
    x=_x;
    Sx+=x;
    return Sx;
}
void NumericalTool::Calculus::SetDiffnInte(const double &_x){
    if(init_plag==0) {x_old=_x;init_plag=1;}
    x=_x;
    xdot=(x-x_old)*inv_dt;
    x_old=x;

    Sx+=x*dt;
}

//**********LowPassFilter**********//
double NumericalTool::LowPassFilter::Filter(double input, double f_c,double t_s)
{
    if(init_plag==0){pre_output=input;init_plag=1;}
    double w = pi2 * f_c;
    //double alpha = w * t_s / (1 + t_s * w);
    double alpha = w * t_s;
    output= alpha * (double)input + (1 - alpha) * pre_output;
    pre_output=output;
    return output;
}
#endif
