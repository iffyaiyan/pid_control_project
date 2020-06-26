#include "PID.h"
#include <limits>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_,PID_Type pid_type_)
{
	Kp=Kp_;
	Ki=Ki_;
	Kd=Kd_;
	prev_cte=0.0;
	cte_i=0.0;

	pid_type=pid_type_;

	for (int i=0;i<3;i++)
	{
		p[i]=0.0;
		dp[i]=1.0;
	}

	best_err=std::numeric_limits<double>::max();
	//second_step=false;
}

void PID::Tunning(double error,int index,bool second_step)
{
	std::cout<<"Index: "<<index<<endl;
	p[index] += dp[index];
	if (!second_step)
	{
		if (error < best_err )
		{
			best_err = error;
			dp[index] *= 0.5;
		}
		else
		{
			p[index] -=  dp[index];

		}
	}
	if (second_step)
	{
		if (error < best_err)
		{
			best_err = error;
			dp[index] *= 0.5;
		}
		else
		{
			p[index] += dp[index];
			dp[index] *= 0.5;
		}
	}

	switch (index)
	{
		case 0: Kp=p[0]; break;
		case 1: Ki=p[1]; break;
		case 2: Kd=p[2]; break;
	}
	std::cout<<"Kp: "<<Kp<<" Ki: "<<Ki<<" Kd: "<<Kd<<endl;
}
void PID::UpdateError(double cte_)
{
	cte=cte_;
	std::cout<<"cte: "<<cte<<std::endl;
	p_error = -Kp*cte;
	i_error = -Ki*cte_i;
	d_error = -Kd*(cte-prev_cte);

	prev_cte = cte;
	cte_i += cte;
}

double PID::TotalError()
{
	double te=p_error+i_error+d_error;

	if (pid_type==PID_Type::Steer)
	{
		if (te<-1)
		{
			te=-1;
		}
		else if(te>1)
		{
			te=1;
		}
	}
	else if (pid_type==PID_Type::Speed)
	{
		//std::cout<<"Speed: "<<te<<std::endl;
		if (te<-30)
		{
			te=-20;
		}
		else if(te>0)
		{
			te=-0.1;
		}
	}
	else
	{	te = fabs(te);

		std::cout<<"ACC: "<<te<<std::endl;

		te*=10.5;
		if (te<-0.001)
		{
			te=0.1;
		}
		else if(te>0.3)
		{
			te=0.3;
		}

		if (cte>0.3)
		{
			te=0.05;
		}
		if (cte>0.2)
		{
			te=0.1;
		}
	}
	return te;
}

