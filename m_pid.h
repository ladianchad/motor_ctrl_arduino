#ifndef H_M_PID
#define H_M_PID
#include <TimerFive.h>
class m_pid
{
	typedef void (*todo)();
	typedef void (*motor)(int pwm,bool dir);
	typedef double (*get_sp)();
private:
	motor motor_fun[10] {};
	get_sp sp_fun[10] {};
	todo todo_fun[10][10];
	int todo_size[10] {};
	double PID[10][3]{};
	double target[10]{};
	double now_spd[10]{};
	double err_pre[10]{};
	double I[10] {};
	int to_pwm[10] {};
	bool to_dir[10] {};
	int motor_num;
	void operate(int index);
public:
	void timer_set();
	bool timer_flag;
	m_pid(int m_num=2,int max_st=10);
	int state;
	int max_state;
	void push_todo(int index,todo fun);
	double get_speed(int index,get_sp fun);
	void motor_move(int index,motor fun);
	void ctrl_start();
	void set_target(int index,double src);
	void pid_set(int index,double P,double I,double D);
};

#endif
