#include "m_pid.h"

m_pid::m_pid(int m_num,int max_st){
	motor_num = m_num;
	max_state = max_st;
}
void m_pid::push_todo(int index,todo fun){
	if(index >= max_state-2  || index == 0 || index == 1)
		return;
	if(todo_size[index - 2] >=10)
		return;
	todo_fun[index-2][todo_size[index-2]++] = fun;
}

double m_pid::get_speed(int index,get_sp fun){
	sp_fun[index] = fun;
}

void m_pid::motor_move(int index,motor fun){
	motor_fun[index] = fun;
}

void m_pid::pid_set(int index,double P,double I,double D){
	PID[index][0] = P;
	PID[index][1] = I;
	PID[index][2] = D;
}
void m_pid::operate(int index){
	now_spd[index] = sp_fun[index]();
	double err = target[index] - abs(now_spd[index]);
	double P = PID[index][0] * err;
	if(PID[index][1] != 0)
		I[index] += PID[index][1] * (err + err_pre[index]);
	else
		I[index] =0;
	double D = PID[index][2] * (err - err_pre[index]);
	err_pre[index] = err;
	double sum = P+I[index]+D;
	if(sum<0){
		sum = 0;
	}
	if(sum >255)
		sum = 255;
	to_pwm[index] = sum;
}
void m_pid::ctrl_start(){
		if(timer_flag){
			if(state == 0){
				for(int i=0;i<motor_num;i++)
					operate(i);
			}
			else if(state == 1){
				for(int i=0;i<motor_num;i++)
					motor_fun[i](to_pwm[i],to_dir[i]);
			}
			else{
				for(int i=0;i<todo_size[state-2];i++)
						todo_fun[state-2][i]();
			}
			timer_flag = false;
		}
}

void m_pid::set_target(int index,double src){
	if(src>0)
		to_dir[index] = 1;
	else
		to_dir[index] = 0;
	target[index] = abs(src);
}
