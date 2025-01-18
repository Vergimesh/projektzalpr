#include "pid.h"

void pid_init(pid_str *pid_data, float kp_init, float ki_init, float kd_init, float anti_windup_limit_init)
{
	pid_data->previous_error = 0;
	pid_data->total_error = 0;

	pid_data->Kp = kp_init;
	pid_data->Ki = ki_init;
	pid_data->Kd = kd_init;

	pid_data->anti_windup_limit = anti_windup_limit_init;
}

void pid_reset(pid_str *pid_data)
{
	pid_data->total_error = 0;
	pid_data->previous_error = 0;
}

float pid_calculate(pid_str *pid_data, float setpoint, float process_variable)
{
	 float error = setpoint - process_variable;


	    pid_data->total_error += error;
	    float p_term = pid_data->Kp * error;
	    float i_term = pid_data->Ki * pid_data->total_error;
	    float d_term = pid_data->Kd * (error - pid_data->previous_error);

	    if (i_term > pid_data->anti_windup_limit){ i_term = pid_data->anti_windup_limit;}
	    if (i_term < -pid_data->anti_windup_limit) {i_term = -pid_data->anti_windup_limit;}

	    pid_data->previous_error = error;

	    float output = (float)(p_term + i_term + d_term);
	    return output;
}
