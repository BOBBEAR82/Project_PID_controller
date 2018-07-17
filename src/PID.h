#ifndef PID_H
#define PID_H
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>

class PID {
public: 
	//Errors
	double p_error;
	double i_error;
	double d_error;

	//PID Coefficients 
	std::vector<double> K; // Kp, KI, Kp
	
	//PID Coefficients increment
	std::vector<double> dK;
  
	//previous cte	
	double cte_old;
  
	//PID Coefficients tuning is needed
	bool tuning_enabled;
	
	//flag showing where the calculation is in twiddle algorithm
	bool from_top;
	
	//total steps count in the simulation
	long total_steps;
	
	//steps count in every tuning cycle
	long cycle_steps;
	
	//steps used for preperatin in every tuning cycle before error integration
	long prep_steps;
	
	//steps used in every tuning cycle for error integration
	long eval_steps;
	
	//error integration in every eval cycle
	double err;
	
	//top 3 smallest error of eval cycle, in order to avoid one single good case happen 
	//which might not be able to represent the best value
	std::vector<double> best_err;
	
	//index of PID Coefficients
	int K_index;
	
	//PID algorithm output
	double result;
	
	//PID algorithm output of last step (not used)
	double result_old;
	
	//max steering change rate from last step (not used)
	double max_result_change;
	
	//log the key data
	std::ofstream PID_log;
	
	/*
	* Constructor
	*/
	PID();

	/*
	* Destructor.
	*/
	virtual ~PID();

	/*
	* Initialize PID.
	*/
	void Init(double Kp, double Ki, double Kd);

	/*
	* Update the PID error variables given cross track error.
	*/
	void UpdateError(double cte);

	/*
	* Calculate the total PID error.
	*/
	double TotalError();
};

#endif /* PID_H */
