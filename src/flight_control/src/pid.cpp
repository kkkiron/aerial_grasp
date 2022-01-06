#include "../include/flight_control/pid.hpp"

FlightControl::pid::pid(double Kp, double Ki, double Kd, double Max, double Min){

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    this->MaxLimit = Max;
    this->MinLimit = Min;
   
    this->Error = 0;
    this->PastError = 0; 
    this->IntegralError = 0; 
    this->DifferentialError = 0;

    this->Output = 0; 

    StartTime = high_resolution_clock::now();

}

double FlightControl::pid::PidOutput(double SetPoint, double Input){
    EndTime = high_resolution_clock::now();
    
    this->TimeInterval = std::chrono::duration_cast<milliseconds>(EndTime - StartTime);
     
    uint32_t dt = this->TimeInterval.count();

    //this->Error = Input - SetPoint;
 
    this->Error = SetPoint - Input;
    
    this->IntegralError += Ki * Error * dt;

    this->DifferentialError = (Error - PastError) / dt;

    this->Output = Kp * (this->Error) + Ki * (this->IntegralError) + Kd * (this->DifferentialError);
    
    this->StartTime = high_resolution_clock::now(); 

    this->PastError = this->Error;

    return (clipping());

}

double FlightControl::pid::clipping(){
    if(this->Output > MaxLimit)
        this->Output = MaxLimit;
    if(this->Output < MinLimit)
        this->Output = MinLimit;
    return this->Output; 
}
