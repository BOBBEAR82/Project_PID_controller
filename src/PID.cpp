#include "PID.h"
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() 
{
	PID_log.close();
}

void PID::Init(double Kp_int, double Ki_int, double Kd_int) 
{
	PID::K = {Kp_int, Ki_int, Kd_int};
	
	p_error = 0;
	i_error = 0;
	d_error = 0;
	
	cte_old = 0;	
	
	PID::dK = {0.6*Kp_int, 0.6*Ki_int, 0.3*Kd_int};
  
	tuning_enabled = 0;
	
	total_steps = 0;
	
	cycle_steps = 0;
	
	prep_steps = 100;
	
	eval_steps = 1370; //this need to be tuned to make sure it is close to integral multiple of full lap for better error compare
	
	err = 1;
	
	best_err = {1.0E10, 1.0E10, 1.0E10};
	
	K_index = 2;
	
	from_top = 1;
	
	result = 0;
	
	result_old = 0;
	
	max_result_change =0.1;
	
	PID_log.open( "PID_log.txt", ios::out );
}

void PID::UpdateError(double cte) 
{
	p_error = cte;
	i_error += cte;
	d_error = cte - cte_old;
	
	cte_old = cte;
	
	if (tuning_enabled)
	{
		
		//integrate the error in eval steps
		if (cycle_steps % (prep_steps + eval_steps) > prep_steps)
		{
			err += cte *cte;
		}
		
		//either finishing a cycle or integrated error larger than best error will trigger cycle reset
		if (total_steps !=0 && cycle_steps !=0 && (cycle_steps % (prep_steps + eval_steps) == 0 || err > best_err[2]))
		{
			if (from_top) //first part of twiddle algorithm
			{
				from_top = 0;
				
				if (err < best_err[2])
				{
					best_err[2] = err;
					dK[K_index] *= 1.1;
					from_top = 1;
					K_index = (K_index + 1) % 3;
					K[K_index] += dK[K_index];
				}
				else
				{
					K[K_index] -= 2 * dK[K_index];
				}
			}
			else //second part of twiddle algorithm
			{
				if (err < best_err[2])
				{
					best_err[2] = err;
					dK[K_index] *= 1.1;
					from_top = 1;
					K_index = (K_index + 1) % 3;
					K[K_index] += dK[K_index];
				}
				else
				{
					K[K_index] += dK[K_index];
					dK[K_index] *= 0.9;
					from_top = 1;
					K_index = (K_index + 1) % 3;
					K[K_index] += dK[K_index];
				}
			}
			
			std::sort(best_err.begin(), best_err.end());
			
			if (best_err[1]>1.0E8 && best_err[2]>1.0E8)
			{
				best_err[1] = best_err[0];
				best_err[2] = best_err[0];
			}
			
			//avoid negative coefficient value
			K[K_index] = std::max(K[K_index], 0.);
			
			//reset data at the end of one tuning cycle
			err = 0;
			i_error = 0;
			cycle_steps = 0;
			
			//write key data to file
			PID_log<<K[0]<<" "<<
					K[1]<<" "<<
					K[2]<<" "<<
					dK[0]<<" "<<
					dK[1]<<" "<<
					dK[2]<<" "<<
					best_err[0]<<" "<<
					best_err[1]<<" "<<
					best_err[2]<<" "<<endl;
		}
	}
	
	total_steps ++;
	cycle_steps ++;
	
	cout<<"   cycle_steps: "<<cycle_steps<<" total_steps: "<<total_steps<<
		"     Kp: "<<K[0]<<" Ki: "<<K[1]<<" Kd: "<<K[2]<<" dKp: "<<dK[0]<<" dKi: "<<dK[1]<<" dKd: "<<dK[2]<<
		"     err: "<<err<<" best_err: "<<best_err[0]<<" "<<best_err[1]<<" "<<best_err[2]<<endl;

}

double PID::TotalError() 
{
	result = -K[0] * p_error -K[1] * i_error -K[2] * d_error; 
	//result = std::min(std::max(result, result_old - max_result_change), result_old + max_result_change);
	//result_old = result;
	return std::min(std::max(result, -0.6), 0.6);
}

