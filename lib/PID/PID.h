class PID {
 private:
	// Controller gains 
	float Kp;
	float Ki;
	float Kd;

	// Output limits 
	float limMin;
	float limMax;

	// Previous measurements

	float Diff;
	float prevPose;		// Required for differentiator 
    float prevDiff;
    float Inertia; 
public:
	// Controller output 
	float out;
    void Controller_Init(PID &pid,float omegac, float zeta)
{
    float omegac = 5.0, zeta = 1.5, kp = omegac * omegac, kd = 2 * zeta * omegac;
	// Use controller variables
    pid.Kp = Kp;
    pid.Kd = Kd;

	pid.diff  = 0;
	pid.prevPose = 0;

    float PID.Inertia = 0.0001627; // Inertia 

};
    float PIDController_Update(PID &pid, float goal, float pose, float velocity){
    
    float error = goal - pose; // Error signal
    float prop = pid.Kp * error; //Proportional contribution

	float diff = -pid.kv * velocity;
	// Compute output and apply limits
	
    float output = prop + diff;
    
    if (output > pid.limMax) {
        output = pid.limMax;
    } 
    else if (output < pid.limMin) {
        output = pid.limMin;
    }
    
	// Store error and measurement for later use 
    pid.prevError       = error;
    pid.prevPose = pose;
    pid.prevDiff = diff;

	// Returns Output
    return output*pid.Inertia;
}; };
