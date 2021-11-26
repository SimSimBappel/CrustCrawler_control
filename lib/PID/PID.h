class PID {
 private:
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */
    float prevDerivative;
public:
	/* Controller output */
	float out;
    void Controller_Init(PID &pid,float Kp, float Kd, float Ki)
{
	// Use controller variables
    pid.Kp = Kp;
    pid.Kp = Kd;
    pid.Kp = Ki;

    /* Clear controller variables */
	pid.integrator = 0.0;
	pid.prevError  = 0.0;

	pid.differentiator  = 0.0;
	pid.prevMeasurement = 0.0;

};
    float PIDController_Update(PID &pid, float reference, float measurement){
    
    float error = reference - measurement; // Error signal
    float prop = pid.Kp * error; //Proportional contribution

    float integrator = pid.integrator + 0.5 * pid.Ki * pid.T * (error + pid.prevError); //Integral contribution

	// Static integrator Limits 
    if (pid.integrator > pid.limMaxInt) {

        pid.integrator = pid.limMaxInt;

    } else if (pid.integrator < pid.limMinInt) {

        pid.integrator = pid.limMinInt;
    }

		
    float differentiator = -(2.0 * pid.Kd * (measurement - pid.prevMeasurement)	// Derivative Contribution
                        + (2.0 * pid.tau - pid.T) * prevDerivative)
                        / (2.0 * pid.tau + pid.T);


	
	// Compute output and apply limits
	
    float output = prop + integrator + differentiator;

    if (output > pid.limMax) {
        output = pid.limMax;
    } 
    else if (output < pid.limMin) {
        output = pid.limMin;
    }

	// Store error and measurement for later use 
    pid.prevError       = error;
    pid.prevMeasurement = measurement;
    pid.prevDerivative = differentiator;

	// Returns Output
    return output;
}; };
