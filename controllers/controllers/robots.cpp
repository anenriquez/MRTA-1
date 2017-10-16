
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/led.h>
#include <webots/servo.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>


#define NUM_ROBOT			8	//4,5,6,7,8
#define MAX_TERMINATION_TIME	3001


enum {AUCTION_ENABLE, AUCTION_DISABLE, AUCTION_WAIT_BIDDER, AUCTION_GET_BIDDER, BID_WAIT_ACCEPT, BID_WAIT_FOLLOWER, BID_DISABLE};

enum {STATE_INIT, STATE_READY, STATE_GET_SLAVE, STATE_GET_MASTER, STATE_GET_GARBAGE, STATE_ZONE_STAY, STATE_EMPTY_BIN, STATE_SORT, STATE_BREAK_DOWN, STATE_NEW_TASK};	
enum {BEH_GO_POSITION, BEH_STORE_OBJ, BEH_BACK_POSITION, BEH_PICK_UP, BEH_LOAD_BLOCK, BEH_PUSH_PANEL, BEH_STOP};
enum {TASK1_ZONE1_1, TASK1_ZONE1_2, TASK1_ZONE1_3, TASK1_ZONE2_1, TASK1_ZONE2_2, TASK1_ZONE2_3, 
      TASK1_ZONE3_1, TASK1_ZONE3_2, TASK1_ZONE3_3, TASK1_2, TASK2_1, TASK2_2, TASK3_1, TASK3_2, TASK4_1, TASK4_2};
#define NUM_TASK		15

enum {CAP_LOC, CAP_RECOG, CAP_MOB, CAP_PUSH, CAP_GRIP, CAP_STOR};
#define NUM_CAP			6



#define MAX_TASK_BLOCK  7


#define AUCTION_WAIT_TIME 50
#define BID_WAIT_TIME 30

//sensor angle
#define ANG_GAP 0.4
#define SIDE_ANG 1.57

#define MAX_SENSOR_NUMBER 8
#define RANGE 1024.0
#define WHEEL_ROT 7.0

/*
#define FRONT_VEL_MAX 100.0
#define PHI_VEL_MAX 300.0
#define FRONT_ACCEL_MAX 10.0
#define PHI_ACCEL_MAX 30.0
*/
#define FRONT_VEL_MAX 100.0
#define PHI_VEL_MAX 300.0
#define FRONT_ACCEL_MAX 5.0
#define PHI_ACCEL_MAX 80.0


#define AXIS_LENGTH 0.32
#define NUMERATOR 4.72 // exp(1)+1+1


#define FORWARD  1 
#define BACKWARD 2 
#define ROTATION 3  


#define TEAM_FORM_AUCTION 0
#define INDI_AUCTION	  1


#define ASSIGN_MESSAGE		-1
#define AUCTION_MESSAGE		-2
#define REJECT_MESSAGE		-3
#define ABANDON_MESSAGE		-4
#define COMPLETE_MESSAGE	-5
#define DONE_MESSAGE		-6
#define BID_MESSAGE			-7
#define COME_MESSAGE		-8
#define ARRIVE_MESSAGE		-9
#define LOAD_MESSAGE		-10
#define WAIT_MESSAGE		-11
#define PUSH_TOGETHER_MESSAGE -13
#define EMPTY_MESSAGE		-14
#define EMPTY_DONE_MESSAGE	-15
#define TRAY_BACK_MESSAGE   -16
#define SHIFT_BLOCK_MESSAGE -17
#define BLOCK_IS_MESSAGE	-18
#define WHAT_BLOCK_MESSAGE	-19
#define PULL_MESSAGE		-20
#define PULL_DONE_MESSAGE	-21
#define REMOVE_BLOCK_MESSAGE -22
#define SORT_DONE_MESSAGE	-23
#define LOAD_DONE_MESSAGE   -24
#define MOVE_TRAY_MESSAGE	-26
#define LEFT_BLOCK_MESSAGE  -27
#define BLOCK_NUM_MESSAGE	-28
#define RECOG_MESSAGE  -29
#define INIT_MESSAGE -30


#define MY_PI				3.1415
#define MY_HALF_PI			1.5708


#define LED_RED    0xff0000
#define LED_GREEN  0x00ff00
#define LED_BLUE   0x0000ff
#define LED_YELLOW 0xffff00


static WbDeviceTag sensors[MAX_SENSOR_NUMBER];
static WbDeviceTag front_gps;
static WbDeviceTag center_gps; 
static WbDeviceTag emitter;
static WbDeviceTag receiver;
static WbDeviceTag led;
static WbDeviceTag tong_servo;
static WbDeviceTag tong_servo2;
static WbDeviceTag push_servo;
static WbDeviceTag obs_sensors[10];

static int time_step = 0;
static double time_sec = 0;


static void initialize()
{    
	wb_robot_init();  
	//init_position();
	time_step = (int)wb_robot_get_basic_time_step();  
	time_sec = (double) time_step / 1000;//time
	
	for (int i = 0; i < MAX_SENSOR_NUMBER; i++) {
		char sensors_name[5];
		sprintf(sensors_name, "ds%d", i);
		sensors[i] = wb_robot_get_device(sensors_name);
		wb_distance_sensor_enable(sensors[i], time_step);
	}
		
	for (i = 0; i < 10; i++) {
		char sensors_name[5];
		sprintf(sensors_name, "ob%d", i);
		obs_sensors[i] = wb_robot_get_device(sensors_name);
		wb_distance_sensor_enable(obs_sensors[i], time_step);
	}

	emitter = wb_robot_get_device("emitter");
	receiver = wb_robot_get_device("receiver");
	front_gps = wb_robot_get_device("front_gps");
	center_gps = wb_robot_get_device("center_gps"); 
	led = wb_robot_get_device("led");
	tong_servo = wb_robot_get_device("tong_servo");	
	tong_servo2 = wb_robot_get_device("tong_servo2");	
	push_servo = wb_robot_get_device("push_servo");

	
	wb_gps_enable(front_gps, time_step);
	wb_gps_enable(center_gps, time_step);
	wb_receiver_enable(receiver, time_step);
	wb_servo_enable_position(tong_servo, time_step);	
	wb_servo_enable_position(tong_servo2, time_step);	
	wb_servo_enable_position(push_servo, time_step);
	
	
}

double find_max(double a, double b) { if (a >= b) return a; else return b; }
double find_min(double a, double b) { if (a < b)  return a; else return b; }

void get_position(double* rob_pos)
{
	const double* front_pos;
	const double* center_pos;
	front_pos = wb_gps_get_values(front_gps);
	center_pos = wb_gps_get_values(center_gps);
	rob_pos[0] = center_pos[0];//x
	rob_pos[1] = center_pos[2];//z
	rob_pos[2] = atan2(front_pos[0]-center_pos[0], front_pos[2]-center_pos[2]); 
}

double get_good_ang(double ang)
{
	if (ang >= 3.1415) 
		ang -= 6.28;  
	else if (ang <= -3.1415) 
		ang += 6.28;  
	return ang;
}

void get_direction_ang(double r_pos2, double* err_pos)
{
	double arc = atan2(err_pos[0], err_pos[1]);  
	err_pos[2] = -r_pos2 + arc; 
	err_pos[2] = get_good_ang(err_pos[2]);
}

void get_local_speed(double* wheel_vel, double* rob_vel)
{    
	rob_vel[0] = (wheel_vel[0] + wheel_vel[1])/(2.0*WHEEL_ROT);  
	rob_vel[1] = 0;
	rob_vel[2] = (-wheel_vel[0] + wheel_vel[1])/(2.0*WHEEL_ROT*AXIS_LENGTH);
}

int get_tg_direction(double tg_ang)
{
	int tg_direction = 8;	
	double tmp_abs_ang = tg_ang;
	
	//printf("Real=%.2f Change=%.2f\n", tmp_abs_ang, tmp_abs_ang+1.57);
	tmp_abs_ang += 1.57;
	
	if (tmp_abs_ang >= 0 && tmp_abs_ang < 1.0*ANG_GAP)			//0.0~0.4
		tg_direction = 7;
	else if (tmp_abs_ang >= 1.0*ANG_GAP && tmp_abs_ang < 2.0*ANG_GAP)			//0.4~0.8
		tg_direction = 6;
	else if (tmp_abs_ang >= 2.0*ANG_GAP && tmp_abs_ang < 3.0*ANG_GAP)			//0.8~1.2
		tg_direction = 5;
	else if (tmp_abs_ang >= 3.0*ANG_GAP && tmp_abs_ang < 1.57)					//1.2~1.57
		tg_direction = 4;
	else if (tmp_abs_ang >= 1.57 && tmp_abs_ang < 5.0*ANG_GAP)					//1.57~2.0
		tg_direction = 3;
	else if (tmp_abs_ang >= 5.0*ANG_GAP && tmp_abs_ang < 6.0*ANG_GAP)			//2.0~2.4
		tg_direction = 2;
	else if (tmp_abs_ang >= 6.0*ANG_GAP && tmp_abs_ang < 7.0*ANG_GAP)			//2.4~2.8
		tg_direction = 1;
	else if (tmp_abs_ang >= 7.0*ANG_GAP && tmp_abs_ang < 8.0*ANG_GAP)			//2.8~3.2
		tg_direction = 0;
	else
		tg_direction = -1;
	
	return tg_direction;
}


int find_val(int n, int* arr, int target)
{
	int cnt = 0;
	for (int i=0; i<n; i++) {
		if (arr[i] == target)
			cnt++;
	}
	return cnt;
}
int find_val(int n, double* arr, double target)
{
	int cnt = 0;
	for (int i=0; i<n; i++) {
		if (arr[i] == target)
			cnt++;
	}
	return cnt;
}
int find_val(int n, int (*arr)[2], double target)
{
	int cnt = 0;
	for (int i=0; i<n; i++) {
		if (arr[i][0] == target)
			cnt++;
	}
	return cnt;
}

int get_block_avoid()
{
	int back_angle = 0;
	int sensor_val[10];	
	int left_turn = 0;
	int right_turn = 0;
	int check_flag = 0;
	for (int i=0; i<10; i++) {
		sensor_val[i] = (int)wb_distance_sensor_get_value(obs_sensors[i]);		
		//		printf("%d /", sensor_val[i]);
		if (sensor_val[i] > 1) {
			if (i < 5)
				right_turn++;
			else
				left_turn++;
			check_flag = 1;
		}
	}					
	if (check_flag == 1) {
		if (right_turn > left_turn) {
			back_angle = 1;
			//printf("Right turn\n");
		}
		else {
			back_angle = 2;
			//printf("Left turn\n");
		}
	}
	return back_angle;
}

int get_obs_avoid(int tg_direction)
{	
	int sensors_value[MAX_SENSOR_NUMBER];	
	int repulsive_angle=0;    
	int sensor_thres[8]={500, 300, 100, 50, 50, 100, 300, 500};
	int too_close = 0;	
	int left_turn = 0;
	int right_turn = 0;
	int close_cnt = 0;
	for (int i = 0; i < MAX_SENSOR_NUMBER; i++) {
		sensors_value[i] = (int)wb_distance_sensor_get_value(sensors[i]);			
		if (sensors_value[i] > sensor_thres[i]) {
			if (i < 4)
				right_turn++;
			else
				left_turn++;
		}	
		if (i > 0 && i < MAX_SENSOR_NUMBER-1) {
			if (sensors_value[i] > 800)
				close_cnt++;
		}
	} 		
	if (close_cnt > 2)
		repulsive_angle = -1;
	else {
		if (left_turn != 0 || right_turn != 0) {		
			if (abs(right_turn - left_turn) == 1 && right_turn + left_turn > 2)
				repulsive_angle = -1;
			else if (right_turn > left_turn)
				repulsive_angle = 1;
			else if (right_turn < left_turn)
				repulsive_angle = 2;
		}
		if (left_turn == right_turn && left_turn != 0)
			repulsive_angle = -1;
	}

	return repulsive_angle;
}


void get_avoid_wheel(int repulsive_angle, int tg_direction, double* wheel_vel) {
	if (tg_direction == -1) {
		if (repulsive_angle == 1) {
			wheel_vel[0] = 30;			
			wheel_vel[1] = -30;							
		}
		else if (repulsive_angle == 2) {
			wheel_vel[0] = -30;			
			wheel_vel[1] = 30;							
		}
		else {
			wheel_vel[0] = -39;
			wheel_vel[1] = -41;
		}
		//printf("tg_direction=%d\n", tg_direction);
	}
	else if (repulsive_angle == 1) {	
		//wheel_vel[0] = 80;
		//wheel_vel[1] = 40;							
		wheel_vel[0] = 30;
		wheel_vel[1] = 5;							
	}
	else if (repulsive_angle == 2) { 
		//wheel_vel[0] = 40;
		//wheel_vel[1] = 80;					
		wheel_vel[0] = 5;
		wheel_vel[1] = 30;							
	}	
	else if (repulsive_angle == -1) {
		wheel_vel[0] = -39;
		wheel_vel[1] = -41;			
	}
}

const double tray1_center[2] = {0, 7};


void get_wheel_speed(double* r_pos, double* tg_pos, double* curr_vel, double* wheel_vel, double margin, int move_type, int avoid_flag, int block_avoid_flag, int tray1_avoid_flag)
{
	

	double err_pos[3];  //x,z,phi
	double err_loc_pos[3]; 
	double err_vel[3] = {0, 0, 0}; 
	double max_accel[3] = {FRONT_ACCEL_MAX, 0, PHI_ACCEL_MAX};//front (-z), side, rotation
	double max_vel[3] = {FRONT_VEL_MAX, 0, PHI_VEL_MAX};//front, side, rotation  
	double curr_vel_target[3];
	double curr_accel[3];  
	double direction[3];
	double err_loc_abs_pos[3];
	double curr_abs_vel[3];
	double loc_tmp;  
	int i;	
	err_pos[0] = tg_pos[0] - r_pos[0];//x
	err_pos[1] = tg_pos[1] - r_pos[1];//z
	
	get_direction_ang(r_pos[2], err_pos);//phi
	
	int tg_direction = get_tg_direction(err_pos[2]);	
	int tray1_repulsive_angle = 0;	
	//double tray1_dist_thres_sq = 4;
	double tray1_dist_thres_sq = 3;
	double dist_from_tray1_sq = pow(r_pos[0] - tray1_center[0], 2) + pow(r_pos[1] - tray1_center[1], 2);
	//printf("dist=%.2f\n", sqrt(dist_from_tray1_sq));
	if (tray1_avoid_flag == 1 && dist_from_tray1_sq < tray1_dist_thres_sq) {				
		double ang_from_tray1 = get_good_ang(-r_pos[2] + atan2(tray1_center[0]-r_pos[0], tray1_center[1]-r_pos[1]));
		
		if (tg_direction == -1)
			tray1_repulsive_angle = -1;
		else if (ang_from_tray1 >= 0 && ang_from_tray1 < MY_PI*0.67)
			tray1_repulsive_angle = 1;
		else if (ang_from_tray1 < 0 && ang_from_tray1 > -MY_PI*0.67)
			tray1_repulsive_angle = 2;
		
	}
	int repulsive_angle = 0;
	int block_repulsive_angle = 0;
	if (avoid_flag == 1) {
		repulsive_angle = get_obs_avoid(tg_direction);
	}
	if (block_avoid_flag == 1) {
		block_repulsive_angle = get_block_avoid();
	}
	if (tray1_repulsive_angle != 0 || repulsive_angle != 0) {
		if (tray1_repulsive_angle != 0) {
			get_avoid_wheel(tray1_repulsive_angle , tg_direction, wheel_vel);		
			if (repulsive_angle != 0) {
				wheel_vel[0] = -39;
				wheel_vel[1] = -41;		
			}
		}
		else {
			get_avoid_wheel(repulsive_angle , tg_direction, wheel_vel);
			if (rand()%2 == 0) {
				wheel_vel[0] += (rand()%20);
				wheel_vel[1] -= (rand()%10);
			}
			else {
				wheel_vel[0] -= (rand()%10);
				wheel_vel[1] += (rand()%20);
			}
		}	
	}		
	else if (block_repulsive_angle != 0) {
		get_avoid_wheel(block_repulsive_angle , tg_direction, wheel_vel);		
	}
	else {
		if (fabs(err_pos[0]) < 0.05)
			err_pos[0] = 0;  
		if (fabs(err_pos[1]) < 0.05)
			err_pos[1] = 0;  
		if (fabs(err_pos[2]) < 0.05)
			err_pos[2] = 0;
			
		double rep_effect = 1.0;
		
		
		err_loc_pos[0] =  (cos(-r_pos[2])*err_pos[0] + sin(-r_pos[2])*err_pos[1]);
		err_loc_pos[1] = (-sin(-r_pos[2])*err_pos[0] + cos(-r_pos[2])*err_pos[1]);  
		err_loc_pos[2] = err_pos[2];
			
		
		if (move_type == FORWARD) {
			err_loc_pos[1] = sqrt(err_loc_pos[0]*err_loc_pos[0] + err_loc_pos[1]*err_loc_pos[1]) - margin;
			err_loc_pos[0] = 0;		
		}
		else if (move_type == BACKWARD) {
			err_loc_pos[1] = sqrt(err_loc_pos[0]*err_loc_pos[0] + err_loc_pos[1]*err_loc_pos[1]) - margin;
			err_loc_pos[0] = 0;
			err_loc_pos[2] =  get_good_ang(err_loc_pos[2] + 3.1415);		
		}	
		else if (move_type == ROTATION) {
			err_loc_pos[1] = 0;		
			err_loc_pos[2] = get_good_ang(-r_pos[2] + tg_pos[2]);	
			max_accel[2] *= 0.1;
			max_vel[2] *= 0.1;
		}

		for (i=0; i<3; i++) {    		
			if (err_loc_pos[i] >= 0)
				direction[i] = 1.0;
			else
				direction[i] = -1.0;
			err_loc_abs_pos[i] = direction[i]*err_loc_pos[i];    		
		}
		
		if (move_type == BACKWARD)
			direction[1] *= -1.0;

		
		curr_abs_vel[0] = direction[1]*curr_vel[0];
		curr_abs_vel[1] = direction[0]*curr_vel[1];
		curr_abs_vel[2] = direction[2]*curr_vel[2];
		
		for (i=0; i<3; i++) {    
			if (i < 2)
				loc_tmp = err_loc_abs_pos[1-i];
			else
				loc_tmp = err_loc_abs_pos[i];
			curr_vel_target[i] = find_min(max_vel[i], sqrt(find_max(0, 2*max_accel[i]*loc_tmp)));        
			if (curr_abs_vel[i] < curr_vel_target[i])
				curr_accel[i] = find_min(max_accel[i], (curr_vel_target[i] - curr_abs_vel[i])/0.1);		
			else
				curr_accel[i] = 0;
			
			curr_abs_vel[i] += curr_accel[i]*0.1;
		}    
		
		err_vel[0] = direction[1]*curr_abs_vel[0];  
		err_vel[2] = direction[2]*curr_abs_vel[2];     
				
		wheel_vel[0] = WHEEL_ROT*(err_vel[0] - AXIS_LENGTH*err_vel[2]);
		wheel_vel[1] = WHEEL_ROT*(err_vel[0] + AXIS_LENGTH*err_vel[2]);
	}		
}

double real_dist(double* p1, double* p2, int sqare_root) {
	if (sqare_root == 1)
		return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
	else
		return pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2);
}

int go_target(double* rob_pos, double* real_tg, double* rob_vel, double* wheel_vel, double angle_margin, double dist_margin, int move_type, int avoid_flag, int block_avoid_flag, int tray1_avoid_flag)
{			
	int reach = 0;	
	if (move_type != ROTATION) {	
		double tmp_dist = real_dist(rob_pos, real_tg, 1);
		double tmp_ang = get_good_ang(-rob_pos[2] + atan2(real_tg[0]-rob_pos[0], real_tg[1]-rob_pos[1]));
		if (move_type == BACKWARD)			
			tmp_ang = get_good_ang(tmp_ang + MY_PI);		
		if (tmp_dist < 0.01)
			angle_margin = 10;
		if (tmp_dist > dist_margin + 0.05 || fabs(tmp_ang) > angle_margin) {
			get_wheel_speed(rob_pos, real_tg, rob_vel, wheel_vel, dist_margin, move_type, avoid_flag, block_avoid_flag, tray1_avoid_flag);
			get_local_speed(wheel_vel, rob_vel);  		
			wb_differential_wheels_set_speed(wheel_vel[0], wheel_vel[1]); 							
		}
		else
			reach = 1;		
	}
	else {
		double tmp_ang = get_good_ang(rob_pos[2] - real_tg[2]);		
		if (fabs(tmp_ang) > angle_margin) {
			rob_vel[0] = 0;
			rob_vel[1] = 0;
			get_wheel_speed(rob_pos, real_tg, rob_vel, wheel_vel, angle_margin, move_type, avoid_flag, block_avoid_flag, tray1_avoid_flag);
			get_local_speed(wheel_vel, rob_vel);  		
			wb_differential_wheels_set_speed(wheel_vel[0], wheel_vel[1]); 							
		}
		else
			reach = 1;
	}
	//printf("Left=%.2f Right=%.2f\n", wheel_vel[0], wheel_vel[1]);
	return reach;
}

int shift_tasks(int task_cnt, double (*receive_tasks)[7])
{
	int tmp_cnt = task_cnt;
	int fin_cnt = 0;
	for (int i=0; i<task_cnt; i++) {
		if (receive_tasks[fin_cnt][0] == -100) {
			tmp_cnt--;
			for (int j=fin_cnt; j<tmp_cnt; j++) {
				for (int k=0; k<7; k++) {
					receive_tasks[j][k]  = receive_tasks[j+1][k];
				}
			}
		}
		else 
			fin_cnt++;
	}
	return tmp_cnt;
}

int find_shift(int cnt, int target1, int target2, double (*people)[7])
{
	int tmp_sft = 0;
	for (int i=0; i<cnt; i++) {
		if (target2 != -1) {
			if (people[i][0] == target1 && people[i][1] == target2)
				tmp_sft = 1;
		}
		else {
			if (people[i][0] == target1)
				people[i][0] = -100;
		}
		if (i<cnt-1) {
			for (int j=0; j<7; j++)
				people[i][j] = people[i+tmp_sft][j];			
		}
	}
	
	if (target2 != -1) {
		if (tmp_sft == 1)
			cnt--;
		else 
			cnt = shift_tasks(cnt, people);	
	}		
	return cnt;
}

int get_cap_availiability(double (*rob_cap)[NUM_CAP], double (*task_req)[NUM_CAP])
{
	int task_cap_condition = 0;
	int cap_available = 0;
	for (int i=0; i<NUM_CAP; i++) {	
		cap_available = 0;
		if (rob_cap[0][i] > 0)
			cap_available = 1;
		task_cap_condition += (int)task_req[0][i] * (int)(1 - cap_available);
	}
	if (task_cap_condition == 0) {
//		printf("[[Capability is available for the task]]\n");		
		cap_available = 1;
	}
	else {
		//printf("[[Not enough capability]]\n");		
		cap_available = 0;
	}	
	return cap_available;
}

double get_consumption_rate(double (*rob_cap)[NUM_CAP], double (*task_req)[NUM_CAP], double* consumption_rate)
{		
	consumption_rate[0] = 0;
	consumption_rate[1] = 0;
	double quality = 0;
	for (int i=0; i<NUM_CAP; i++) {	
		quality += task_req[0][i] * task_req[1][i] * rob_cap[0][i];
		consumption_rate[0] += task_req[0][i] * rob_cap[1][i] * rob_cap[2][i];
		consumption_rate[1] += task_req[0][i] * rob_cap[1][i] * (1.0 - rob_cap[2][i]);		
	}	
	return quality;
}

int get_bid_values(double (*rob_cap)[NUM_CAP], double (*task_req)[NUM_CAP], double esti_dist, double energy, double* bid_value )
{
	int task_available = 0;
	int check = get_cap_availiability(rob_cap, task_req);
	if (check == 1) {
		bid_value[0] = 0;
		bid_value[1] = 0;			
		double velocity = 0.56;
		double est_time = esti_dist/velocity;		
		int task_cap_condition = 0;	
		int cap_available = 0;
		double consumption_rate[2];
		bid_value[0] = get_consumption_rate(rob_cap, task_req, consumption_rate);				
		bid_value[1] = consumption_rate[0] * esti_dist + consumption_rate[1] * est_time;
		if (energy > bid_value[1]) {
			printf("[[Energy is available for the task (Curr=%.2f Req=%.2f]]\n", energy, bid_value[1]);
			task_available = 1;
		}
		else {
			printf("[[Not enough energy for the task (Curr=%.2f Req=%.2f]]\n", energy, bid_value[1]);
			task_available = 0;
		}	
	}
	return task_available;
}




void get_utility(double (*receive_bidders)[9], double* bid_utility, double bid_weight, double s_weight, int bid_cnt)
{
	//receive_bidders: (0)task / (1)auctioneer_id / (2)bidder_id / (3)quality / (4)cost
	double tot_quality = 0;
	double tot_cost = 0;
	double tot_eng = 0;
	for (int i=0; i<bid_cnt; i++) {
		bid_utility[i] = -1;
		tot_quality += receive_bidders[i][3];//quality
		tot_cost += receive_bidders[i][4];//cost
		tot_eng += receive_bidders[i][8];//eng
	}
	if (tot_quality == 0) {
		tot_quality = 1;
		//printf("$$$$$$ Problem!! Tot quality is zero\n");
	}
	if (tot_cost == 0) {
		tot_cost = 1;
		//printf("$$$$$$ Problem!! Tot cost is zero\n");
	}
	if (tot_eng == 0) {
		tot_eng = 1;
		//printf("$$$$$$ Problem!! Tot cost is zero\n");
	}
	printf("[Total Q=%.2f / C=%.2f / E=%.2f\n", tot_quality, tot_cost, tot_eng);	
	for (i=0; i<bid_cnt; i++) {
		double norm_Q = receive_bidders[i][3]/tot_quality;
		double norm_C = receive_bidders[i][4]/tot_cost;		
		double norm_E = receive_bidders[i][8]/tot_eng;		
		printf("[ID=%d / nQ=%.2f / nC=%.2f / nE=%.2f", (int)receive_bidders[i][2], norm_Q, norm_C, norm_E);
		bid_utility[i] = 0.5 * ((1 - s_weight) * (bid_weight * (norm_Q) - (1-bid_weight) * (norm_C)) + s_weight * norm_E + 1.0);
		printf(" / U=%.2f]\n", bid_utility[i]);
	}
}



double zone_stay[24][2] = {									
	{8.0, 11.8}, {8.7, 10.4}, {9.4, 9}, {9.7, 7.6}, {9.7, 6.2}, {9.4, 4.8}, {8.7, 3.4}, {8.0, 2.0},
	{0.7, -3.3}, {-0.7, -3.3}, {2.1, -2.9}, {-2.1, -2.9}, {3.5, -2.2}, {-3.5, -2.2}, {4.9, -1.5}, {-4.9, -1.5},
	{-8.0, 11.8}, {-8.7, 10.4}, {-9.4, 9}, {-9.7, 7.6}, {-9.7, 6.2}, {-9.4, 4.8}, {-8.7, 3.4}, {-8.0, 2.0}};
//double station_stay[2][2] = {{3.1, 22.2}, {-3.1, 22.2}};

double tray1_zone[3][3]	 = {{0.95, 7 , -MY_HALF_PI}, {0.0, 6, 0}, {-0.95, 7, MY_HALF_PI}};
double zone_center[3][2] = {{4.5, 7},{0, 2.5},{-4.5, 7}};
double zone1_2[3] = {2.8, 4, 0};   //1<->2
double zone2_3[3] = {-2.8, 4, 0};  //2<->3

double sorting_pos[3] = {0, 18.3, MY_PI};
double sorting_wait_pos[4][3] = {{0.7, 17, 0}, {-0.7, 17, 0}, {1.6, 17, 0}, {-1.6, 17, 0}};
double pick_block_pos[3] = {0, 17.9, 0};
double sorting_out_pos[2] = {0, 18.5};


double stone_tray_pos[3] = {-0.6, 18.8, -MY_HALF_PI};
double garbage_tray_pos[3] = {0.6, 18.8, MY_HALF_PI};
double recycle_tray_pos[3] = {0, 19.5, 0};
double small_tray_pos[3][3] = {{0.6, 18.8, MY_HALF_PI}, {0, 19.5, 0}, {-0.6, 18.8, -MY_HALF_PI}};


double station_node_left[3][2] = {{1.6, 11}, {2, 17.8}, {1.6, 11.5}};
double station_node_right[3][2] = {{-1.6, 11}, {-2, 17.8}, {-1.6, 11.5}};

double tray1_left[5][3] = {{1.5, 5, 0}, {0.7, 5.5, 0}, {0.7, 12.4, 0}, {0.7, 5.75, 0}, {7, 0, 0}};
double tray1_right[5][3] = {{-1.5, 5, 0}, {-0.7, 5.5, 0}, {-0.7, 12.4, 0}, {-0.7, 5.75, 0}, {-7, 0, 0}};


int get_zone(double* target, int curr_zone, int target_zone)
{		
	//printf("Zone %d to ", curr_zone);
	if (curr_zone == 0) {
		if (target_zone == 1) {
			target[0] = zone1_2[0];
			target[1] = zone1_2[1];		
		}				
		else if (target_zone == 2) {
			target[0] = zone1_2[0];
			target[1] = zone1_2[1];		
		}				
		curr_zone = 1;
	}
	else if (curr_zone == 1) {
		if (target_zone == 0) {
			target[0] = zone1_2[0];
			target[1] = zone1_2[1];
		}
		else if (target_zone == 2) {
			target[0] = zone2_3[0];
			target[1] = zone2_3[1];
		}
		curr_zone = target_zone;
	}	
	else if (curr_zone == 2) {		
		if (target_zone == 0) {
			target[0] = zone2_3[0];
			target[1] = zone2_3[1];
		}
		else if (target_zone == 1) {
			target[0] = zone2_3[0];
			target[1] = zone2_3[1];		
		}
		curr_zone = 1;
	}	
	else if (curr_zone == 4) {
		target[0] = station_node_left[2][0];
		target[1] = station_node_left[2][1];		
		curr_zone = 0;					
	}
	else if (curr_zone == 5) {
		target[0] = station_node_right[2][0];
		target[1] = station_node_right[2][1];		
		curr_zone = 2;					
	}
		
	//printf("Zone %d\n", curr_zone);
	return curr_zone;
}

double get_dist(double* rob_pos, int curr_task, int curr_zone, int leader_zone) {
	double esti_dist = 0;
	int target_zone = 0;
	double target[2] = {rob_pos[0], rob_pos[1]};
	double mid_target[2] = {0, 0};
	double final_target[2] = {0, 0};	
	
	if (curr_task < TASK1_2) {
		if (curr_task >= TASK1_ZONE1_1 && curr_task <= TASK1_ZONE1_3)
			target_zone = 0;
		else if (curr_task >= TASK1_ZONE2_1 && curr_task <= TASK1_ZONE2_3)
			target_zone = 1;
		else if (curr_task >= TASK1_ZONE3_1 && curr_task <= TASK1_ZONE3_3)
			target_zone = 2;				
		final_target[0] = zone_center[target_zone][0];
		final_target[1] = zone_center[target_zone][1];
		
		while (curr_zone != target_zone) {
			//			printf("zone %d to", curr_zone);
			curr_zone = get_zone(mid_target, curr_zone, target_zone);			
			esti_dist += real_dist(target, mid_target, 1);
			//			printf(" %d (tot=%.2f)=>", curr_zone, esti_dist);
			target[0] = mid_target[0];
			target[1] = mid_target[1];
		}
		//		printf("\n");
		esti_dist += real_dist(target, final_target, 1);		
	}		
	else if (curr_task == TASK1_2) {
		target_zone = leader_zone;
		
		//		printf("Into loop1, curr_zone = %d\n", curr_zone);
		while (curr_zone != target_zone) {
			//printf("zone %d to", curr_zone);
			curr_zone = get_zone(mid_target, curr_zone, target_zone);			
			esti_dist += real_dist(target, mid_target, 1);
			//printf(" %d (tot=%.2f)=>", curr_zone, esti_dist);
			target[0] = mid_target[0];
			target[1] = mid_target[1];
		}
		//		printf("\n");		
	}
	else if (curr_task == TASK3_1) {	
		if (curr_zone == 0) {			
			esti_dist += real_dist(rob_pos, station_node_left[0], 1);			
			esti_dist += (station_node_left[0][0] - station_node_left[1][0]);			
		}
		else if (curr_zone == 1) {
			esti_dist += real_dist(rob_pos, zone1_2, 1);
			esti_dist += real_dist(zone1_2, station_node_left[0], 1);			
			esti_dist += (station_node_left[0][0] - station_node_left[1][0]);			
		}
		else if (curr_zone == 2) {
			esti_dist += real_dist(rob_pos, station_node_right[0], 1);			
			esti_dist += (station_node_left[0][0] - station_node_left[1][0]);			
		}
		else {
			double dist1 = real_dist(rob_pos, station_node_left[1], 1);
			double dist2 = real_dist(rob_pos, station_node_right[1], 1);			
			if (dist1 < dist2)
				esti_dist += dist1;
			else
				esti_dist += dist2;			
		}
		
		esti_dist += real_dist(sorting_pos, station_node_left[1], 1);		
		esti_dist += (0.8*12);
	}	
//	printf("[[Estimated dist for task %d= %.2f\n", curr_task, esti_dist);
	return esti_dist;
}

int find_ID(int cnt, int target, int* people)
{
	int exist_ID = 0;
	for (int i=0; i<cnt; i++) {
		if (people[i] == target) {
			exist_ID = 1;
			break;
		}		
	}	
	return exist_ID;
}

void get_zone_position(int rob_int_name, double* target)
{
	double gap = 1.2;	
	double gap2 = 0.9;
	switch(rob_int_name) {
	case 1:
		target[0] += gap;
		break;	
	case 2:
		target[0] += gap2;
		target[1] += gap2;
		break;
	case 3:
		target[1] += gap;
		break;	
	case 4:
		target[0] -= gap2;
		target[1] += gap2;
		break;
	case 5:
		target[0] -= gap;
		break;
	case 6:
		target[0] -= gap2;
		target[1] -= gap2;
		break;
	case 7:
		target[1] -= gap;			
		break;
	case 8:
		target[0] += gap2;
		target[1] -= gap2;
		break;	
	}
}

int irand(int den) 
{ 
	int r_indx = rand()%5;
    int r_val[5]; 
	r_val[0] = rand()%den; 
	r_val[1] = rand()%den; 
	r_val[2] = rand()%den; 	
	r_val[3] = rand()%den; 
	r_val[4] = rand()%den; 	
    return r_val[r_indx]; 
} 
double drand() 
{ 
	int r_indx = rand()%5;
    double r_val[5]; 
	r_val[0] = (double)rand() / ((double) RAND_MAX + 1); 
	r_val[1] = (double)rand() / ((double) RAND_MAX + 1); 
	r_val[2] = (double)rand() / ((double) RAND_MAX + 1); 	
	r_val[3] = (double)rand() / ((double) RAND_MAX + 1); 
	r_val[4] = (double)rand() / ((double) RAND_MAX + 1); 	
    return r_val[r_indx]; 
} 

int get_match(int task, int auctioneer, int cnt, double quality, double cost, double bid_weight, double esti_dist, double (*receive_tasks)[7])
{
	int feedback = -4;
	int task2 = task;
	int mem_task = 0;

	if (task <= TASK1_2) 
		task2 = 1;
	for (int i=0; i<cnt; i++) {
		if ((int)receive_tasks[i][0] == task && (int)receive_tasks[i][1] == auctioneer) {
			return task;
		}
		mem_task = (int)receive_tasks[i][0];
		if (mem_task <= TASK1_2)
			mem_task = 1;
		if (task2 > mem_task)
			return -1;
		else if (task2 == mem_task) {
			if (quality == -1)
				return -2;
			else {
				if (receive_tasks[i][3] == INDI_AUCTION) {
					double tot_quality = quality + receive_tasks[i][4];
					double tot_cost = cost + receive_tasks[i][5];
					double norm_Q1 = quality/tot_quality;
					double norm_Q2 = receive_tasks[i][4]/tot_quality;					
					double norm_C1 = cost/tot_cost;
					double norm_C2 = receive_tasks[i][5]/tot_cost;															
					double u1 = 0.5 * (bid_weight * (norm_Q1) - (1-bid_weight) * (norm_C1) + 1.0);
					double u2 = 0.5 * (bid_weight * (norm_Q2) - (1-bid_weight) * (norm_C2) + 1.0);
					
					printf("[nQ1,2=(%.2f, %.2f) / nC1,2=(%.2f, %.2f) / U1,2=(%.2f, %.2f)\n", norm_Q1, norm_Q2, norm_C1, norm_C2, u1, u2);
					if (u1 > u2) { 
						printf("Task %d (%.2f) > Task %d (%.2f) :exchange in memory!\n", task, u1, (int)receive_tasks[i][0], u2);												
						receive_tasks[i][0] = (double)task;
						receive_tasks[i][1] = (double)auctioneer;
						receive_tasks[i][2] = 0;
						receive_tasks[i][4] = quality;
						receive_tasks[i][5] = cost;
						receive_tasks[i][6] = esti_dist;				
						return -3;											
					}				
				}
			}			
		}
	}	
	return feedback;
}


int find_match(int task, int auctioneer, int cnt, double (*receive_tasks)[7])
{
	for (int i=0; i<cnt; i++) {
		if ((int)receive_tasks[i][0] == task && (int)receive_tasks[i][1] == auctioneer)
			return 1;
	}
	return 0;
}


int get_task_indx(int curr_task)
{
	int t_indx = -1;
	if (curr_task >= TASK1_ZONE1_1 && curr_task < TASK1_2)
		t_indx = 0;
	else if (curr_task == TASK1_2)
		t_indx = 1;		
	else if (curr_task == TASK2_1 || curr_task == TASK2_2)
		t_indx = 2;		
	else if (curr_task == TASK3_1)
		t_indx = 3;		
	else if (curr_task == TASK3_2)
		t_indx = 4;	
	return t_indx;
}

void update_cost(int curr_task, double dist_measure, double time_measure, double* task_dist, double* task_time, int tot_block, int k)
{
	int t_indx = get_task_indx(curr_task);
	double pre_task_dist = task_dist[t_indx];
	double pre_task_time = task_time[t_indx];
	
	if (t_indx == 0 || t_indx == 1 || t_indx == 3) {
		if (tot_block > 1) {
			printf("Since come_cnt is %d, I divide!\n", tot_block);
			dist_measure /= (double)tot_block;
			time_measure /= (double)tot_block;
		}
		else if (tot_block <= 0) {
			printf("\n[Divide by zero!!!]\n");
			printf("\n[Divide by zero!!!]\n");
			printf("\n[Divide by zero!!!]\n");
		}
	}
	
	task_dist[t_indx] = pre_task_dist + (dist_measure - pre_task_dist) / ((double)(k+1));
	task_time[t_indx] = pre_task_time + (time_measure - pre_task_time) / ((double)(k+1));
	
	printf("From dist=%.2f to %.2f  /  time=%.2f to %.2f\n", pre_task_dist, task_dist[t_indx], pre_task_time, task_time[t_indx]);		
}

void get_d2t2(double (*receive_bidders)[9], double* d2t2, int bid_cnt)
{
	d2t2[0] = 0;
	d2t2[1] = 0;
	for (int i=0; i<bid_cnt; i++) {
		d2t2[0] += receive_bidders[i][5];
		d2t2[1] += receive_bidders[i][6];
	}
	d2t2[0] /= (double)bid_cnt;
	d2t2[1] /= (double)bid_cnt;	
	printf("\n[Get d2t2]=(%.2f, %.2f) bid_cnt=%d\n", d2t2[0], d2t2[1], bid_cnt);
}

double get_energy_consumption(int curr_task, double (*rob_cap)[NUM_CAP], double (*task_req0)[NUM_CAP], double (*task_req1)[NUM_CAP], double (*task_req2)[NUM_CAP], double (*task_req3)[NUM_CAP], double rob_vel, double time_sec) 
{
	double energy = 0;
	double consumption_rate[2] = {0, 0};			
	double my_noise = (drand() - 0.5) * 0.01;
	if (curr_task != -1) {
		if (curr_task < TASK1_2 || curr_task == TASK3_1)
			get_consumption_rate(rob_cap, task_req1, consumption_rate);	
		else if (curr_task == TASK1_2)
			get_consumption_rate(rob_cap, task_req2, consumption_rate);	
		else if (curr_task == TASK2_1 || curr_task == TASK2_2 || curr_task == TASK3_2)
			get_consumption_rate(rob_cap, task_req3, consumption_rate);	
		energy = (consumption_rate[0] * (fabs(rob_vel) * time_sec) + consumption_rate[1] * time_sec) * (1 + my_noise);
	}
	return energy;
}

double get_ave_bid_weight(double curr_bid_weight, double (*bidders)[9], int cnt)
{
	double new_bid_weight = curr_bid_weight;

	for (int i=0; i<cnt; i++)
		new_bid_weight += bidders[i][7];
	
	return new_bid_weight / (double)(cnt+1);
}
double update_bid_weight(double curr_bid_weight, double measure_bid_weight)
{
	double alpha = 0.01;
	double new_bid_weight = curr_bid_weight + alpha * (measure_bid_weight - curr_bid_weight);
	return new_bid_weight;
}
int get_real_category(int block_indx)
{
	if (block_indx < 10 || (block_indx >= 30 && block_indx < 40))				
		return 0;
	else if (block_indx < 20 || (block_indx >= 40 && block_indx < 50))				
		return 1;
	else
		return 2;
}
int get_block_category(int block_category, double rob_recog_qual)
{	
	double r_val = drand(); 
	double tmp_prob1 = rob_recog_qual - 0.1;
	double tmp_prob2 = tmp_prob1 + 0.5*(1 - tmp_prob1);
	printf("random val=%.2f / 0~%.2f / ~%.2f / ~1.00\n", r_val, tmp_prob1, tmp_prob2);
	if (r_val > tmp_prob1 && r_val <= tmp_prob2) {
		printf("In first prob area\n");
		if (block_category == 0)
			block_category = 1;
		else if (block_category == 1)
			block_category =2;
		else if (block_category == 2)
			block_category = 0;
	}
	else if (r_val > tmp_prob2) {
		printf("In second prob area\n");
		if (block_category == 0)
			block_category = 2;
		else if (block_category == 1)
			block_category =0;
		else if (block_category == 2)
			block_category = 1;
	}	
	return block_category;
}
void get_rob_cap(int rob_int_name, int rob_num, double (*rob_cap)[NUM_CAP])
{
	if (rob_num == 4 || rob_num == 6) {
		rob_cap[0][CAP_LOC] =	0.5;
		rob_cap[1][CAP_LOC] =	0.15;
		rob_cap[2][CAP_LOC] =	0;
		
		rob_cap[0][CAP_RECOG] = 0.5;
		rob_cap[1][CAP_RECOG] = 0.1;
		rob_cap[2][CAP_RECOG] = 0;
		
		rob_cap[0][CAP_MOB] =	0.1;
		rob_cap[1][CAP_MOB] =	1.0;
		rob_cap[2][CAP_MOB] =	1;
		
		rob_cap[0][CAP_PUSH] =	0;
		rob_cap[1][CAP_PUSH] =	0;
		rob_cap[2][CAP_PUSH] =	0;
		
		rob_cap[0][CAP_GRIP] =	0.5;
		rob_cap[1][CAP_GRIP] =	0.5;
		rob_cap[2][CAP_GRIP] =	1;
		
		rob_cap[0][CAP_STOR] =	0.1;
		rob_cap[1][CAP_STOR] =	1.5;
		rob_cap[2][CAP_STOR] =	1;
	}
	else if (rob_num == 5) {//2, 3
		switch (rob_int_name) {		
			case 2:
				rob_cap[0][CAP_LOC] =	0.6;
				rob_cap[1][CAP_LOC] =	0.2;
				rob_cap[2][CAP_LOC] =	0;
				
				rob_cap[0][CAP_RECOG] = 0.6;
				rob_cap[1][CAP_RECOG] = 0.2;
				rob_cap[2][CAP_RECOG] = 0;
				
				rob_cap[0][CAP_MOB] =	0.3;
				rob_cap[1][CAP_MOB] =	2.0;
				rob_cap[2][CAP_MOB] =	1;
				
				rob_cap[0][CAP_PUSH] =	0;
				rob_cap[1][CAP_PUSH] =	0;
				rob_cap[2][CAP_PUSH] =	0;
				
				rob_cap[0][CAP_GRIP] =	0.6;
				rob_cap[1][CAP_GRIP] =	1.0;
				rob_cap[2][CAP_GRIP] =	1;
				
				rob_cap[0][CAP_STOR] =	0.2;
				rob_cap[1][CAP_STOR] =	2.0;
				rob_cap[2][CAP_STOR] =	1;
				break;
			case 3:
				rob_cap[0][CAP_LOC] =	0.5;
				rob_cap[1][CAP_LOC] =	0.15;
				rob_cap[2][CAP_LOC] =	0;
				
				rob_cap[0][CAP_RECOG] = 0.5;
				rob_cap[1][CAP_RECOG] = 0.1;
				rob_cap[2][CAP_RECOG] = 0;
				
				rob_cap[0][CAP_MOB] =	0.1;
				rob_cap[1][CAP_MOB] =	1.0;
				rob_cap[2][CAP_MOB] =	1;
				
				rob_cap[0][CAP_PUSH] =	0;
				rob_cap[1][CAP_PUSH] =	0;
				rob_cap[2][CAP_PUSH] =	0;
				
				rob_cap[0][CAP_GRIP] =	0.5;
				rob_cap[1][CAP_GRIP] =	0.5;
				rob_cap[2][CAP_GRIP] =	1;
				
				rob_cap[0][CAP_STOR] =	0.1;
				rob_cap[1][CAP_STOR] =	1.5;
				rob_cap[2][CAP_STOR] =	1;
				break;		
		}		
	}
	else if (rob_num >= 7) {//3, 4
		switch (rob_int_name) {		
			case 3:
				rob_cap[0][CAP_LOC] =	0.6;
				rob_cap[1][CAP_LOC] =	0.2;
				rob_cap[2][CAP_LOC] =	0;
				
				rob_cap[0][CAP_RECOG] = 0.6;
				rob_cap[1][CAP_RECOG] = 0.2;
				rob_cap[2][CAP_RECOG] = 0;
				
				rob_cap[0][CAP_MOB] =	0.3;
				rob_cap[1][CAP_MOB] =	2.0;
				rob_cap[2][CAP_MOB] =	1;
				
				rob_cap[0][CAP_PUSH] =	0;
				rob_cap[1][CAP_PUSH] =	0;
				rob_cap[2][CAP_PUSH] =	0;
				
				rob_cap[0][CAP_GRIP] =	0.6;
				rob_cap[1][CAP_GRIP] =	1.0;
				rob_cap[2][CAP_GRIP] =	1;
				
				rob_cap[0][CAP_STOR] =	0.2;
				rob_cap[1][CAP_STOR] =	2.0;
				rob_cap[2][CAP_STOR] =	1;
				break;
			case 4:
				rob_cap[0][CAP_LOC] =	0.5;
				rob_cap[1][CAP_LOC] =	0.15;
				rob_cap[2][CAP_LOC] =	0;
				
				rob_cap[0][CAP_RECOG] = 0.5;
				rob_cap[1][CAP_RECOG] = 0.1;
				rob_cap[2][CAP_RECOG] = 0;
				
				rob_cap[0][CAP_MOB] =	0.1;
				rob_cap[1][CAP_MOB] =	1.0;
				rob_cap[2][CAP_MOB] =	1;
				
				rob_cap[0][CAP_PUSH] =	0;
				rob_cap[1][CAP_PUSH] =	0;
				rob_cap[2][CAP_PUSH] =	0;
				
				rob_cap[0][CAP_GRIP] =	0.5;
				rob_cap[1][CAP_GRIP] =	0.5;
				rob_cap[2][CAP_GRIP] =	1;
				
				rob_cap[0][CAP_STOR] =	0.1;
				rob_cap[1][CAP_STOR] =	1.5;
				rob_cap[2][CAP_STOR] =	1;
				break;		
		}		
	}	
}
int main()
{  
	initialize();		
	srand( (unsigned)time(NULL) );	

	int full_record = 0;
		
	FILE* w_file;
	FILE* v_file;
	FILE* energy_file;	
	FILE* fin_eng_file;	

	int store_time[11];
	
	double w_data[11];
	double v_data[11];
	double energy_data[11];
	int t_cnt = 0;

	const char* rob_name;
	rob_name = wb_robot_get_name();
	int rob_int_name = atoi(rob_name);
	
	double rob_pos[3] = {0, 0, 0}; 
	double rob_vel[3] = {0, 0, 0}; 
				
	double wheel_vel[2] = {0, 0};    
	int init_cnt = 0;
	int auction_state = AUCTION_DISABLE;
	int bid_state = BID_DISABLE;
	int rob_state = STATE_INIT;	
	int beh_state = BEH_STOP;		
	int move_type = FORWARD;
				
	double target[3];//x,z,phi	
	double next_target[3];//x,z,phi	


	int rob_num = 0;
	int test_mode = -1;//0:qb, 1:cb, 2:eb, 3:qcb, 4:qcb & subsidy, 5:price_weight test, 6:subsidy_weight test
	int real_termination_time = 0;

	double rob_cap[3][NUM_CAP];
	
	

	double caripper_grip_qual = 0;

	double task_req0[2][NUM_CAP];
	task_req0[0][CAP_LOC] =		1;
	task_req0[1][CAP_LOC] =		0.5;
	task_req0[0][CAP_RECOG] =	0;
	task_req0[1][CAP_RECOG] =	0;
	task_req0[0][CAP_MOB] =		1;
	task_req0[1][CAP_MOB] =		0.5;
	task_req0[0][CAP_PUSH] =	0;
	task_req0[1][CAP_PUSH] =	0;
	task_req0[0][CAP_GRIP] =	0;
	task_req0[1][CAP_GRIP] =	0;
	task_req0[0][CAP_STOR] =	0;
	task_req0[1][CAP_STOR] =	0.0;

	double task_req1[2][NUM_CAP];
	task_req1[0][CAP_LOC] =		1;
	task_req1[1][CAP_LOC] =		0.5;
	task_req1[0][CAP_RECOG] =	1;
	task_req1[1][CAP_RECOG] =	0.8;
	task_req1[0][CAP_MOB] =		1;
	task_req1[1][CAP_MOB] =		0.5;
	task_req1[0][CAP_PUSH] =	0;
	task_req1[1][CAP_PUSH] =	0;
	task_req1[0][CAP_GRIP] =	1;
	task_req1[1][CAP_GRIP] =	1.0;
	task_req1[0][CAP_STOR] =	0;
	task_req1[1][CAP_STOR] =	0.0;
	
	double task_req2[2][NUM_CAP];
	task_req2[0][CAP_LOC] =		1;
	task_req2[1][CAP_LOC] =		0.7;
	task_req2[0][CAP_RECOG] =	0;
	task_req2[1][CAP_RECOG] =	0.0;
	task_req2[0][CAP_MOB] =		1;
	task_req2[1][CAP_MOB] =		0.5;
	task_req2[0][CAP_PUSH] =	0;
	task_req2[1][CAP_PUSH] =	0;
	task_req2[0][CAP_GRIP] =	1;
	task_req2[1][CAP_GRIP] =	0.5;
	task_req2[0][CAP_STOR] =	1;
	task_req2[1][CAP_STOR] =	1.0;
	
	double task_req3[2][NUM_CAP];
	task_req3[0][CAP_LOC] =		1;
	task_req3[1][CAP_LOC] =		1.0;
	task_req3[0][CAP_RECOG] =	0;
	task_req3[1][CAP_RECOG] =	0.0;
	task_req3[0][CAP_MOB] =		1;
	task_req3[1][CAP_MOB] =		0.8;
	task_req3[0][CAP_PUSH] =	1;
	task_req3[1][CAP_PUSH] =	0.8;
	task_req3[0][CAP_GRIP] =	0;
	task_req3[1][CAP_GRIP] =	0.0;
	task_req3[0][CAP_STOR] =	0;
	task_req3[1][CAP_STOR] =	0.0;
	

	
	
	
	double block_list[MAX_TASK_BLOCK][5];
	int block_cnt = 0;
	double dist_margin = 0.1;
	double angle_margin = 0.1;
	int load_obj_step = 0;
	int my_follower = -1;	
	int my_leader = -1;
	double time = 0;    /* a match lasts for 10 minutes */
	int comm_time = 0;
	int tg_direction = 0;			
	double repulsive_angle = 0;	
	int curr_zone = -3;
	int target_zone = -1;
	int zone_arrival = 0;
	double hold_dist = MY_HALF_PI;
	double up_dist = 0;
	int tot_block = 0;
	int left_block = 0;
	int final_block = 0;
	
	int task_cnt = 0;
	double receive_tasks[NUM_TASK][7];//auction_task, auctioneer_id, bid_flag, team or indi, quality, cost, esti_dist 
	int curr_task = -1;
	int prev_task = -1;
	int curr_pre_task = -1;
	int auction_task = -1;
	double auction_data[10];
	int auction_data_size;
	int wait_cnt = 0;
	int bid_cnt = 0;
	double receive_bidders[NUM_ROBOT][9];//receive_bidders: (0)task / (1)auctioneer_id / (2)bidder_id / (3)quality / (4)cost / (5)d2 / (6)t2 / (7)bid_weight / (8)eng
	double tmp_tong_ang = 0;
	int push_step = 0;	
	double push_dist = 0;
	double push_max_dist;
	double back_dist;
	int avoid_flag = 0;
	int block_avoid_flag = 0;
	int tray1_avoid_flag = 0;
	int pick_up_step = 0;
	int empty_flag = 0;
	int tray_full_flag = 0;
	int wait_flag = 0;
	
	int leader_zone = -1;
	int sorting_step = 0;
	int block_category = 0;
	int stop_cnt = 0;
	int coll_cnt = 0;
	int init_coll_cnt = 0;
	int send_cnt = 0;	
	int max_tray_block = 3;
	int tray_block_cnt[3] = {0, 0, 0};
	int tray_block_req[3] = {0, 0, 0};
	double send_message[10][15];
	int full_tray = -1;
	int min_dist_block_indx = -1;
	int arrive_flag = 0;
	int bidder_ID[NUM_ROBOT];
	int bid_wait_cnt = 0;
	int last_tray = -1;
	int grip_fail_flag = 1;
	double hold_step = 0.03;
	double hold_max = 2.1;
	
	int max_block_contain = 2;

	int final_winner = 0;
	double final_winner_utility = 0;
	int team_auc_flag = 0;
	int bid_task = 0;
	double assign_receive_data[10];
	int come_flag = 0;

	double task_dist[5] = {0, 0, 0, 0, 0};//1_1, 1_2, 2_1 & 2_2, 3_1, 3_2
	double task_time[5] = {0, 0, 0, 0, 0};//1_1, 1_2, 2_1 & 2_2, 3_1, 3_2
	int update_cnt[5] = {0, 0, 0, 0, 0};
	double energy_consumption = 0;
	int prev_sec = 0;

	double dist_measure = 0;
	double time_measure = 0;
	
	double dist_strt = 0;
	double time_strt = 0;	
	int strt_flag = -1;
	int div_block = 0;
	double ave_d2t2[2] = {0, 0};
	int come_cnt = 0;
	int task1_1_checker = 0;
	int task1_2_checker = 0;
	
	
	


	
	int no_prob_flag = 0;
	
	
	int switch_grip_fail = 0;

	
	double max_energy = 70000;
	double min_energy = 40000;
	double curr_energy = max_energy;

	double E = 0;// (curr_energy - min_energy)/(max_energy - min_energy)

	
	double bid_weight = 0;	
	double s_weight = 0;
	double s_weight_max = 0.9;
	double std_max = 450;

	double W_min = 0.1;
	double W_max = 0.9;
	double sqare_num = 0.5;
	double bias_bid_weight = 0.5; //(0 <= bias_bid_weight < 1)
	double s_factor = 0.5878;// = log(Wmax/((1-bias_bid_weight)*Wmin + bias_bid_weight*Wmax))
	double W_measure = 0;
	double scalar = (W_max/(1-bias_bid_weight));
	
	int time_cnt = 0;
	int target_block_indx = 0;
	
	double G[MAX_TERMINATION_TIME];	
	

	double cap_loc_dec = 0;
	while (1) {		
		tg_direction = 0;
		repulsive_angle = 0;
		double rnd = drand();		
		get_position(rob_pos);								
		if (rob_state == STATE_INIT) {						
			init_cnt++;
			if (init_cnt == 1)
				printf("Initialize robot %d!!\n", rob_int_name);
			else if (init_cnt > 10) {				
				printf("OKAY\n");	
				rob_state = STATE_READY;
				beh_state = BEH_STOP;	
				

				wb_servo_set_position(tong_servo, -MY_HALF_PI);								
				wb_servo_set_position(tong_servo2, MY_HALF_PI);			
			}					
		}		
		else {				
			int reach = 0;
			if (bid_wait_cnt > BID_WAIT_TIME) {
				bid_wait_cnt = 0;
				if (task_cnt > 0) {				
					for (int i = 0; i<task_cnt; i++) {
						int curr_bid_task = (int)receive_tasks[i][0];
						int curr_auctioneer = (int)receive_tasks[i][1]; 
						int curr_bid_done_flag = (int)receive_tasks[i][2];
						
						if (curr_bid_task == TASK3_1 && curr_bid_done_flag == 0) {
							int task_available = 0;
							if (curr_task == -1) 
								task_available = 1;
							else if (curr_task < curr_bid_task) {					
								if (beh_state != BEH_LOAD_BLOCK) {
									task_available = 1;							
								}						
								if (rob_state == STATE_GET_GARBAGE) {						
									double tmp_dist = real_dist(rob_pos, target, 1);													
									if (tmp_dist > 1)
										task_available = 1;
									else
										task_available = 0;
								}
							}					
							if (task_available == 1) {											
								
								int t_indx = get_task_indx(curr_bid_task);
								send_message[send_cnt][0] = BID_MESSAGE;					
								send_message[send_cnt][1] = (double)curr_bid_task;
								send_message[send_cnt][2] = (double)curr_auctioneer;
								send_message[send_cnt][3] = (double)rob_int_name;					
								send_message[send_cnt][4] = receive_tasks[i][4];
								send_message[send_cnt][5] = receive_tasks[i][5];
								send_message[send_cnt][6] = task_dist[t_indx];
								send_message[send_cnt][7] = task_time[t_indx];
								send_message[send_cnt][8] = bid_weight;
								send_message[send_cnt][9] = curr_energy;
								send_message[send_cnt][14] = 10;	
								send_cnt++;
								receive_tasks[i][2] = 1;//bidding flag																																	
								if (bid_state != BID_WAIT_FOLLOWER)
									bid_state = BID_WAIT_ACCEPT;
								printf("[[Rob %d bids Task %d of auctioneer %d: q=%.2f/c=%.2f/d2=%.2f/t2=%.2f/\n", 
									rob_int_name, curr_bid_task, curr_auctioneer, receive_tasks[i][4], receive_tasks[i][5], task_dist[t_indx], task_time[t_indx]);	
							}
							else
								receive_tasks[i][0] = -100;
						}											
					}
					task_cnt = shift_tasks(task_cnt, receive_tasks);
				}
			}
			else
				bid_wait_cnt++;
			
			double target_dist = 0;
			double target_angle = 0;
			switch (beh_state) {				
				case BEH_GO_POSITION:		
					avoid_flag = 0;
					//hold_dist = 0;
					block_avoid_flag = 1;
					tray1_avoid_flag = 1;
															
					target_dist = sqrt(pow(rob_pos[0] - target[0], 2) + pow(rob_pos[1] - target[1], 2));						
					target_angle = get_good_ang(-rob_pos[2] + atan2(target[0]-rob_pos[0], target[1]-rob_pos[1]));
					//printf("D=%.2f A=%.2f\n", target_dist, target_angle);
					//if (target_dist < 2 || ((rob_state == STATE_ZONE_STAY || beh_state == BEH_STORE_OBJ) && move_type == ROTATION)) {
					if ((target_dist < 2 && zone_arrival == 1) || ((rob_state == STATE_ZONE_STAY || beh_state == BEH_STORE_OBJ) && move_type == ROTATION)) {
						block_avoid_flag = 0;
						//avoid_flag = 0;					
					}
					else {
						block_avoid_flag = 1;
						avoid_flag = 1;
						if (rob_state == STATE_ZONE_STAY && move_type == FORWARD)
							block_avoid_flag = 0;
					}				
					
					if (rob_state == STATE_GET_GARBAGE) {
						avoid_flag = 1;
						dist_margin = 0.1;
						angle_margin = 0.1;
						if (grip_fail_flag == 1)
							dist_margin = 0.25;
						if (target_dist > 1) 
							hold_dist = -MY_HALF_PI;
						else if (target_dist > dist_margin*2)
							hold_dist = 0;
						wb_servo_set_position(tong_servo, hold_dist);								
						wb_servo_set_position(tong_servo2, -hold_dist);
					}
					
					
					if (rob_state == STATE_EMPTY_BIN)
						tray1_avoid_flag = 0;
					else if (rob_state == STATE_SORT) {
						avoid_flag = 1;
						if (sorting_step == 8 && fabs(rob_pos[2]) > 0.85 && fabs(rob_pos[2]) < 2.19) 
							block_avoid_flag = 1;
						else if (rob_pos[1] > 17)
							block_avoid_flag = 0;
					}
										
					if (rob_state == STATE_GET_GARBAGE && target_dist >= 0.1 && target_dist <= 0.4 && fabs(target_angle) > 0.4) {
						wheel_vel[0] = -20;
						wheel_vel[1] = -20;
						wb_differential_wheels_set_speed(wheel_vel[0], wheel_vel[1]);
					}
					else
						reach = go_target(rob_pos, target, rob_vel, wheel_vel, angle_margin, dist_margin, move_type, avoid_flag, block_avoid_flag, tray1_avoid_flag);

					if (reach == 1) {
						rob_vel[0] = 0;
						rob_vel[1] = 0;
						rob_vel[2] = 0;
						//wb_differential_wheels_set_speed(-10, -10);	
						if (rob_state == STATE_GET_GARBAGE) {
							if (zone_arrival == 1) {	
								rob_cap[0][CAP_GRIP] = 0;											
								if (hold_dist > -hold_max) {									
									hold_dist -= hold_step;
									dist_margin = 0.5;
									angle_margin = 0.5;
									wb_differential_wheels_set_speed(0, 0);	
									if (grip_fail_flag == 1) {										
										wb_servo_set_position(tong_servo, hold_dist);								
										wb_servo_set_position(tong_servo2, -hold_dist*0.3);
									}
									else {																			
										wb_servo_set_position(tong_servo, hold_dist);								
										wb_servo_set_position(tong_servo2, -hold_dist-0.02);
									}									
								}
								else {
									int tmp_block_category = get_real_category(target_block_indx);
									int tmp_prob_category = tmp_block_category;
									if (no_prob_flag == 0)
										tmp_prob_category = get_block_category(tmp_block_category, rob_cap[0][CAP_RECOG]);									

									send_message[send_cnt][0] = RECOG_MESSAGE;
									send_message[send_cnt][1] = 0;							
									if (tmp_block_category != tmp_prob_category)										
										send_message[send_cnt][1] = 1;							
									send_message[send_cnt][14] = 2;
									send_cnt++;
									wb_servo_set_position(tong_servo, 0);								
									wb_servo_set_position(tong_servo2, 0);
									hold_dist = 0;
									if (grip_fail_flag == 1 && my_follower != rob_int_name) {
										printf("Rob %d Block pick fail!\n", rob_int_name);
										beh_state = BEH_LOAD_BLOCK;
										target[0] = rob_pos[0];
										target[1] = rob_pos[1];
										target[2] = rob_pos[2];
										zone_arrival = 0;
										
									}
									else {
										dist_margin = 0.1;
										angle_margin = 0.1;									
										hold_dist = 0;		
										wb_differential_wheels_set_speed(0, 0);										
											
										block_cnt++;

										if (my_follower != -1 && my_follower != rob_int_name) {
											//double send_message[send_cnt][6];		
											send_message[send_cnt][0] = COME_MESSAGE;
											send_message[send_cnt][1] = (double)rob_int_name;	
											send_message[send_cnt][2] = (double)my_follower;	
											send_message[send_cnt][3] = rob_pos[0];
											send_message[send_cnt][4] = rob_pos[1];						
											send_message[send_cnt][5] = (double)curr_zone;		
											send_message[send_cnt][14] = 6;
											send_cnt++;
											rob_state = STATE_GET_SLAVE;
											beh_state = BEH_STOP;
											hold_dist = - MY_HALF_PI;
											wb_servo_set_position(tong_servo, hold_dist);								
											wb_servo_set_position(tong_servo2, -hold_dist);
//											printf("[[(1)Doing Task1_1! block cnt=%d\n", block_cnt);
											
										}
										else if (my_follower == -1){
											if (auction_task == -1) {											
												if (curr_task <= TASK1_ZONE1_3)
													target_zone = 0;
												else if (curr_task <= TASK1_ZONE2_3)
													target_zone = 1;
												else if (curr_task <= TASK1_ZONE3_3)
													target_zone = 2;

												auction_task = TASK1_2;
												auction_data[0] = (double)target_zone;												
												auction_data_size = 1;
												auction_state = AUCTION_ENABLE;
												team_auc_flag = INDI_AUCTION;
												rob_state = STATE_GET_SLAVE;
												beh_state = BEH_STOP;
												hold_dist = - MY_HALF_PI;
												wb_servo_set_position(tong_servo, hold_dist);								
												wb_servo_set_position(tong_servo2, -hold_dist);
												printf("Auction Task1_2 for get a follower! (indi auc)\n");
											}
											else {
												printf("[Wait auctioning Task1_2 until Task2_1 is allocated!]\n");
												wb_differential_wheels_set_speed(0, 0);
											}
										}
										else {
											if (grip_fail_flag == 1)
												block_cnt--;
											if (min_dist_block_indx != -1) {
											//	printf("(1)Send load message for block id[%d]\n", (int)block_list[min_dist_block_indx][3]);											
												send_message[send_cnt][0] = LOAD_MESSAGE;
												send_message[send_cnt][1] = (double)my_follower;
												if (grip_fail_flag == 1)
													send_message[send_cnt][1] = -1;
												send_message[send_cnt][2] = (double)empty_flag;
												send_message[send_cnt][3] = (double)curr_pre_task;
												send_message[send_cnt][4] = block_list[min_dist_block_indx][3];
												send_message[send_cnt][5] = (double)grip_fail_flag;
												send_message[send_cnt][14] = 6;
												send_cnt++;
												left_block--;												
											}
											else {
												printf("Let's Not decrease left block=%d\n", left_block);										
											}										
											if (block_cnt < max_block_contain && left_block > 0) {
												printf("(1)Block cnt=%d / Left block=%d / tot block=%d\n", block_cnt, left_block, tot_block);
												double min_dist = 10000;
												for (int bc=0; bc<tot_block; bc++) {
													if (block_list[bc][2] != 10000)
														block_list[bc][2] = pow(rob_pos[0] - block_list[bc][0], 2) + pow(rob_pos[1] - block_list[bc][1], 2);
													if (min_dist > block_list[bc][2]) {
														min_dist = block_list[bc][2];
														min_dist_block_indx = bc;
													}
												}
//												printf("(1)Rob %d go to get block indx=%d\n", rob_int_name, (int)block_list[min_dist_block_indx][3]);
												block_list[min_dist_block_indx][2] = 10000;
												target[0] = block_list[min_dist_block_indx][0];
												target[1] = block_list[min_dist_block_indx][1];	
												target_block_indx = (int)block_list[min_dist_block_indx][4];

												rob_state = STATE_GET_GARBAGE;
												double r_val = drand();
												printf("(Grip prob_1) r_val=%.2f thre=%.2f\n", r_val, caripper_grip_qual);
												if (no_prob_flag == 1)
													r_val = 0;		

												if (switch_grip_fail == 0) {
													if (r_val >= caripper_grip_qual) {
														grip_fail_flag = 1;								
														printf("[[[Fail block grip!!]]\n");
													}
													else
														grip_fail_flag = 0;																				
												}
												else {												
													
													if (grip_fail_flag == 1)
														grip_fail_flag = 0;
													else
														grip_fail_flag = 1;
												}


												beh_state = BEH_GO_POSITION;
												move_type = FORWARD;
												dist_margin = 0.1;
												angle_margin = 0.1;													
											}
											else {											
//												printf("(2)Block cnt=%d / Left block=%d\n", block_cnt, left_block);
												if (block_cnt > 0) {																																	
													beh_state = BEH_STORE_OBJ;														
													target[0] = tray1_zone[curr_zone][0];
													target[1] = tray1_zone[curr_zone][1];
													target[2] = tray1_zone[curr_zone][2];
																										
													double r_val = drand();
													if (no_prob_flag == 1)
														r_val = 0;
													printf("(Load tray1 prob_1) r_val=%.2f / thres=%.2f\n", r_val, rob_cap[0][CAP_LOC] - cap_loc_dec);
													if (r_val >= rob_cap[0][CAP_LOC] - cap_loc_dec) {
														r_val -= (rob_cap[0][CAP_LOC] - cap_loc_dec);
														printf("[Error dist = %.2f\n", r_val);								
														if (curr_zone == 0)
															target[0] += r_val;
														else if (curr_zone == 1)
															target[1] -= r_val;
														else if (curr_zone == 2)
															target[0] -= r_val;
													}
													dist_margin = 0.1;
													angle_margin = 0.1;
													if (tray_full_flag == 1) {
														printf("[[Tray 1 is full so I can't store block to Tray1\n");
														wb_differential_wheels_set_speed(0, 0);
														dist_margin = 0.3;
														angle_margin = 3;
													}
													hold_dist = -MY_HALF_PI;
													wb_servo_set_position(tong_servo, -MY_HALF_PI);								
													wb_servo_set_position(tong_servo2, MY_HALF_PI);
												}
												else {
													beh_state = BEH_BACK_POSITION;
												}												
											}											
										}
									}
								}									
							}
							else {																	
								if (curr_zone == target_zone) {												
									if (task1_1_checker == 0) {
										strt_flag = 0;
										printf("\n\n(2) Start Recording dist & time of Task1_1!!\n");
										task1_1_checker = 1;										
									}
									zone_arrival = 1;
									dist_margin = 0.1;
									angle_margin = 0.1;
									double min_dist = 10000;

									for (int bc=0; bc<tot_block; bc++) {
										if (block_list[bc][2] != 10000)
											block_list[bc][2] = pow(rob_pos[0] - block_list[bc][0], 2) + pow(rob_pos[1] - block_list[bc][1], 2);
										if (min_dist > block_list[bc][2]) {
											min_dist = block_list[bc][2];
											min_dist_block_indx = bc;
										}
									}
//									printf("(2)Rob %d go to get block indx=%d\n", rob_int_name, (int)block_list[min_dist_block_indx][3]);
									block_list[min_dist_block_indx][2] = 10000;
									target[0] = block_list[min_dist_block_indx][0];
									target[1] = block_list[min_dist_block_indx][1];	
									target_block_indx = (int)block_list[min_dist_block_indx][4];
								}
								else {	
									curr_zone = get_zone(target, curr_zone, target_zone);
									dist_margin = 1;
									angle_margin = 1;
									zone_arrival = 0;	
									
									
								}
							}
						}
						else if (rob_state == STATE_GET_MASTER) {							
							if (block_cnt > 0)
								rob_cap[0][CAP_GRIP] = 0;
							else
								rob_cap[0][CAP_GRIP] = caripper_grip_qual;

							if (zone_arrival == 1) {
								wb_servo_set_position(tong_servo, 0);								
								wb_servo_set_position(tong_servo2, 0);
								hold_dist = 0;
								dist_margin = 0.7;
								angle_margin = 0.1;
								beh_state = BEH_STOP;
								arrive_flag = 1;								
								send_message[send_cnt][0] = ARRIVE_MESSAGE;
								send_message[send_cnt][1] = (double)my_leader;	//from						
								send_message[send_cnt][2] = rob_pos[0];
								send_message[send_cnt][3] = rob_pos[1];
								send_message[send_cnt][14] = 4;
								send_cnt++;
								//wb_emitter_send(emitter, send_message[send_cnt], 4*sizeof(double));																					
							}
							else {								
								if (curr_zone == target_zone) {
									if (task1_2_checker == 0) {
										strt_flag = 0;
										printf("\n\n(1) Start Recording dist & time of Task1_2!!\n");
										task1_2_checker = 1;										
									}
									dist_margin = 0.7;
									angle_margin = 0.1;
									zone_arrival = 1;
									target[0] = next_target[0];
									target[1] = next_target[1];	
																		
								}
								else {
									curr_zone = get_zone(target, curr_zone, target_zone);				
									dist_margin = 1;
									angle_margin = 1;
									zone_arrival = 0;
								}
							}							
						}
						else if (rob_state == STATE_ZONE_STAY) {
							if (move_type == FORWARD) {
								target[0] = rob_pos[0];
								target[1] = rob_pos[1];		
								if (curr_zone == 0)
									target[2] = -MY_HALF_PI;
								else if (curr_zone == 1)
									target[2] = 0;
								else if (curr_zone == 2)
									target[2] = MY_HALF_PI;
								angle_margin = 0.1;
								move_type = ROTATION;
							}
							else {
								rob_state = STATE_READY;
								beh_state = BEH_STOP;							
							}
						}
						else if (rob_state == STATE_SORT) {
							wb_differential_wheels_set_speed(0, 0);							
							if (sorting_step == -2) {
								push_dist = 0;
								double dist1 = pow(rob_pos[0] - station_node_left[0][0], 2) + pow(rob_pos[1] - station_node_left[0][1], 2);
								double dist2 = pow(rob_pos[0] - station_node_right[0][0], 2) + pow(rob_pos[1] - station_node_right[0][1], 2);
								if (dist1 < dist2) {
									target[0] = station_node_left[0][0];
									target[1] = station_node_left[0][1];
								}
								else {
									target[0] = station_node_right[0][0];
									target[1] = station_node_right[0][1];
								}
								move_type = FORWARD;
								sorting_step = -1;
								dist_margin = 0.1;
								angle_margin = 0.1;
								hold_dist = -MY_HALF_PI;
							}
							else if (sorting_step == -1) {					
								double dist1 = pow(rob_pos[0] - station_node_left[1][0], 2) + pow(rob_pos[1] - station_node_left[1][1], 2);
								double dist2 = pow(rob_pos[0] - station_node_right[1][0], 2) + pow(rob_pos[1] - station_node_right[1][1], 2);
								if (dist1 < dist2) {
									target[0] = station_node_left[1][0];
									target[1] = station_node_left[1][1];									
									if (curr_zone == 4)
										target[0] += 1;
								}
								else {
									target[0] = station_node_right[1][0];
									target[1] = station_node_right[1][1];
									if (curr_zone == 5)
										target[0] -= 1;
								}								
								move_type = FORWARD;
								sorting_step = -10;
								dist_margin = 0.1;
								angle_margin = 0.1;	
								hold_dist = -MY_HALF_PI;
							}
							else if (sorting_step == -10) {
								target[0] = rob_pos[0];
								target[1] = rob_pos[1];
								if (rob_pos[0] > 0)
									target[2] = -MY_HALF_PI;
								else
									target[2] = MY_HALF_PI;
								dist_margin = 10;
								angle_margin = 0.01;
								sorting_step = 0;
								move_type = ROTATION;
								hold_dist = 0;

								strt_flag = 0;
								printf("\n\n(3) Start Recording dist & time of Task3_1!!\n");
							}
							else if (sorting_step == 0) {
								dist_margin = 0.02;
								angle_margin = 0.1;
								target[0] = pick_block_pos[0];
								target[1] = pick_block_pos[1]-0.1;
								move_type = FORWARD;
								sorting_step = 1;
								hold_dist = 0;
							}
							else if (sorting_step == 1) {
								dist_margin = 1;
								angle_margin = 1;
								if (hold_dist > -hold_max) {
									hold_dist -= hold_step;									
									dist_margin = 0.5;
									angle_margin = 0.5;									
								}
								else {									
									hold_dist = 0;									
									dist_margin = 0.05;
									angle_margin = 0.1;										
									send_message[send_cnt][0] = WHAT_BLOCK_MESSAGE;
									send_message[send_cnt][1] = (double)rob_int_name;
									send_message[send_cnt][14] = 2;
									send_cnt++;										
									sorting_step = -4;									
								}								
							}							
							else if (sorting_step == 4) {
								dist_margin = 0.03;
								angle_margin = 0.1;
								push_dist = 0;

								if (tray_block_cnt[block_category] < max_tray_block) {
									target[0] = small_tray_pos[block_category][0];
									target[1] = small_tray_pos[block_category][1];
									//tray_block_cnt[block_category]++;
									move_type = FORWARD;
									sorting_step = 5;
									printf("Store block to Tray %d\n", block_category+2);
								}
								else {
									if (my_follower == -1) {
										auction_task = TASK3_2;										
										auction_data_size = 0;
										auction_state = AUCTION_ENABLE;
										sorting_step = -19;
									}
									else {																				
										tray_block_req[block_category] = 1;									
										send_message[send_cnt][0] = PULL_MESSAGE;
										send_message[send_cnt][1] = (double)rob_int_name;
										send_message[send_cnt][2] = (double)my_follower;
										send_message[send_cnt][3] = (double)block_category;										
										send_message[send_cnt][14] = 4;
										send_cnt++;										
										printf("(1)Carrier (rob %d)! Pull tray %d\n", my_follower, block_category+2);											
										sorting_step = -20;
									}
									wb_differential_wheels_set_speed(0, 0);
								}								
							}
							else if (sorting_step == 5) {								
								hold_dist = 0;
								if (push_dist < 0.5) 								
									push_dist += 0.02;																								
								else {																						
									send_message[send_cnt][0] = SHIFT_BLOCK_MESSAGE;
									send_message[send_cnt][1] = (double)block_category;
									send_message[send_cnt][2] = (double)tray_block_cnt[block_category];
									send_message[send_cnt][14] = 3;
									send_cnt++;									
									coll_cnt--;
									
									push_dist = 0;
									dist_margin = 0.03;
									angle_margin = 0.1;
									target[0] -= 0.5 * sin(rob_pos[2]);
									target[1] -= 0.5 * cos(rob_pos[2]);	
									rob_vel[2] = 0;
									move_type = BACKWARD;									
									printf("Rob %d move block to Tray %d / coll_cnt=%d\n", rob_int_name, block_category+2, coll_cnt);
									sorting_step = -12;
								}
							}
							else if (sorting_step == 6) {		
								push_dist = 0;
								full_tray = -1;
								for (int i=0; i<3; i++) {
									if (tray_block_cnt[i] >= max_tray_block && tray_block_req[i] == 0)
										full_tray = i;
								}		
								
								if (full_tray > -1) {
									tray_block_req[full_tray] = 1;									
									send_message[send_cnt][0] = PULL_MESSAGE;
									send_message[send_cnt][1] = (double)rob_int_name;
									send_message[send_cnt][2] = (double)my_follower;
									send_message[send_cnt][3] = (double)full_tray;										
									send_message[send_cnt][14] = 4;
									send_cnt++;										
									printf("Carrier (rob %d)! Pull tray %d\n", my_follower, full_tray+2);											
								}		
								
								if (coll_cnt == 0) {
									printf("Coll_cnt is ZERO!! Goto block pick position\n");
									sorting_step = 7;
									dist_margin = 0.02;
									angle_margin = 0.05;
									target[0] = sorting_out_pos[0];
									target[1] = sorting_out_pos[1];
									move_type = FORWARD;		
									hold_dist = -MY_HALF_PI;									
								}
								else
									sorting_step = 0;									
							}
							else if (sorting_step == 7) {								
								if (last_tray == 2) {
									sorting_step = 8;								
									curr_zone = 4;
									target[0] = station_node_left[1][0] + 1;
									target[1] = station_node_left[1][1];									
									printf("Go to station_node_left[1] since last tray is %d\n", last_tray);									
									move_type = FORWARD;									
									dist_margin = 0.1;
									angle_margin = 0.1;	
								}								
								else {
									sorting_step = 8;			
									curr_zone = 5;
									target[0] = station_node_right[1][0] - 1;
									target[1] = station_node_right[1][1];
									printf("Go to station_node_right[1] since last tray is %d\n", last_tray);
									move_type = FORWARD;									
									dist_margin = 0.1;
									angle_margin = 0.1;	
								}				
								last_tray = -1;
							}
							else if (sorting_step == 8) {	
								sorting_step = 9;
								beh_state = BEH_GO_POSITION;
								move_type = FORWARD;
								if (curr_task != -1) {
									prev_task = TASK3_1;
									curr_task = -1;	
									strt_flag = 1;
									printf("[[ Rob %d Complete Task3_1!! ]]\n", rob_int_name);
									printf("\n (1)Compete! strt_flag = 1\n");
								}								
								my_follower = -1;								
								send_message[send_cnt][0] = SORT_DONE_MESSAGE;
								send_message[send_cnt][1] = (double)rob_int_name;
								send_message[send_cnt][14] = 2;
								send_cnt++;
								if (curr_zone == 4) {
									target[0] = station_node_left[0][0];
									target[1] = station_node_left[0][1];
								}
								else {
									curr_zone = 5;
									target[0] = station_node_right[0][0];
									target[1] = station_node_right[0][1];
								}								
								tray_block_req[0] = 0;
								tray_block_req[1] = 0;
								tray_block_req[2] = 0;	
								dist_margin = 0.3;
								angle_margin = 3;
							}
							else if (sorting_step == 9) {
								sorting_step = -1;
								rob_state = STATE_ZONE_STAY;																
								beh_state = BEH_GO_POSITION;
								move_type = FORWARD;								
								int tmp_stay_indx = 8 - rob_num + rob_int_name - 1;
								if (curr_zone == 4) {
									target[0] = zone_stay[tmp_stay_indx][0];
									target[1] = zone_stay[tmp_stay_indx][1];
									curr_zone = 0;
								}
								else {
									target[0] = zone_stay[8*2+tmp_stay_indx][0];
									target[1] = zone_stay[8*2+tmp_stay_indx][1];
									curr_zone = 2;
								}
								target[2] = 0;
							}
							else if (sorting_step == 10) {
								wb_differential_wheels_set_speed(0, 0);
							}
							wb_servo_set_position(tong_servo, hold_dist);								
							wb_servo_set_position(tong_servo2, -hold_dist);
							wb_servo_set_position(push_servo, push_dist);							
						}
						else {
							beh_state = BEH_STOP;
						}
					}					
					break;		
				case BEH_STORE_OBJ:	
					if (tray_full_flag == 0) {
						
						avoid_flag = 1;
						block_avoid_flag = 1;
						//double dist_to_leader = pow(rob_pos[0] - tray1_center[0], 2) + pow(rob_pos[1] - tray1_center[1], 2);						
						double dist_to_leader = pow(rob_pos[0] - target[0], 2) + pow(rob_pos[1] - target[1], 2);						
						//if (dist_to_leader < 2.5) {
						if (dist_to_leader < 1.3) {
							block_avoid_flag = 0;							
						}
						tray1_avoid_flag = 0;
						reach = go_target(rob_pos, target, rob_vel, wheel_vel, angle_margin, dist_margin, move_type, avoid_flag, block_avoid_flag , tray1_avoid_flag);
						if (reach == 1) {
							rob_vel[0] = 0;
							rob_vel[1] = 0;
							rob_vel[2] = 0;
							wb_differential_wheels_set_speed(0, 0);																											
							if (move_type == FORWARD) {
								target[0] = rob_pos[0];
								target[1] = rob_pos[1];																					
								move_type = ROTATION;
								angle_margin = 0.05;
							}
							else {
								beh_state = BEH_PUSH_PANEL;
								push_max_dist = 0.55;
								if (curr_zone == 1)
									back_dist = 1.5;
								else 
									back_dist = 0.6;
								hold_dist = 0;		
								wb_servo_set_position(tong_servo, hold_dist);								
								wb_servo_set_position(tong_servo2, -hold_dist);								
							}
						}		
					}
					else {		
						avoid_flag = 1;
						block_avoid_flag = 1;
						tray1_avoid_flag = 1;
						int tmp_stay_indx = 8*curr_zone + (8 - rob_num + rob_int_name - 1);
						if (curr_zone == 1)
							tmp_stay_indx = 8*curr_zone + (rob_int_name - 1);
						target[0] = zone_stay[tmp_stay_indx][0];
						target[1] = zone_stay[tmp_stay_indx][1];
						
						reach = go_target(rob_pos, target, rob_vel, wheel_vel, angle_margin, dist_margin, move_type, avoid_flag, block_avoid_flag , tray1_avoid_flag);
						if (reach == 1) {
							if (move_type == FORWARD) {
								target[0] = rob_pos[0];
								target[1] = rob_pos[1];
								if (curr_zone == 0)
									target[2] = -MY_HALF_PI;
								else if (curr_zone == 1)
									target[2] = 0;
								else if (curr_zone == 2)
									target[2] = MY_HALF_PI;
								angle_margin = 0.1;
								move_type = ROTATION;
							}
							else {
								rob_vel[0] = 0;
								rob_vel[1] = 0;
								rob_vel[2] = 0;
								wb_differential_wheels_set_speed(0, 0);	
							}
						}
					}
					break;
				case BEH_PUSH_PANEL:
					if (push_step == 0) {						
						if (push_dist < push_max_dist)
							push_dist += 0.02;
						else
							push_step = 1;
					}
					else {
						if (push_dist > 0)
							push_dist -= 0.01;
						else {
							push_step = 0;
							push_dist = 0.0;							
							target[0] -= back_dist * sin(rob_pos[2]);
							target[1] -= back_dist * cos(rob_pos[2]);	
							rob_vel[2] = 0;
							beh_state = BEH_BACK_POSITION;
							move_type = BACKWARD;							
						}
					}
					wb_servo_set_position(push_servo, push_dist);
					break;
				case BEH_BACK_POSITION:				
					avoid_flag = 0;
					block_avoid_flag = 0;
					tray1_avoid_flag = 0;
					reach = go_target(rob_pos, target, rob_vel, wheel_vel, angle_margin, dist_margin, move_type, avoid_flag, block_avoid_flag, tray1_avoid_flag);
					if (reach == 1) {
						hold_dist = -MY_HALF_PI;
						wb_servo_set_position(tong_servo, -MY_HALF_PI);								
						wb_servo_set_position(tong_servo2, MY_HALF_PI);
						rob_vel[0] = 0;
						rob_vel[1] = 0;
						rob_vel[2] = 0;
						if (move_type == BACKWARD && (curr_task == TASK1_2 || my_follower == rob_int_name)) {
							target[0] = rob_pos[0];
							target[1] = rob_pos[1];
							target[2] = get_good_ang(rob_pos[2] + MY_PI);
							dist_margin = 10;
							angle_margin = 0.1;
							move_type = ROTATION;							
						}
						else {
							rob_cap[0][CAP_GRIP] = caripper_grip_qual;										
							if (rob_state == STATE_EMPTY_BIN || rob_state == STATE_NEW_TASK) {														
								tray_full_flag = 0;
																											
								send_message[send_cnt][0] = DONE_MESSAGE;
								send_message[send_cnt][1] = (double)curr_zone;
								send_message[send_cnt][2] = (double)rob_int_name;
								send_message[send_cnt][3] = (double)block_cnt;
								send_message[send_cnt][4] = empty_flag;
								send_message[send_cnt][14] = 5;
								send_cnt++;

			
								if (rob_state == STATE_EMPTY_BIN) {		
									if (empty_flag == 1) {
										empty_flag = 0;
										rob_state = STATE_ZONE_STAY;
										beh_state = BEH_GO_POSITION;
										move_type = FORWARD;								
										dist_margin = 0.1;
										angle_margin = 3;
										int tmp_stay_indx = 8*curr_zone + (8 - rob_num + rob_int_name - 1);
										if (curr_zone == 1)
											tmp_stay_indx = 8*curr_zone + (rob_int_name - 1);
										target[0] = zone_stay[tmp_stay_indx][0];
										target[1] = zone_stay[tmp_stay_indx][1];
										target[2] = 0;
										
										if (curr_task != -1) {
											prev_task = curr_task;
											curr_task = -1;
											strt_flag = 1;
											printf("\n (2)Compete! strt_flag = 1\n");
										}
										my_follower = -1;
										my_leader = -1;			
										printf("Reset leader1\n");	
										dist_margin = 0.1;
										angle_margin = 0.1;
									}
									else {
										if (my_leader == -1) {
											printf("Hull(2)! Leader left! I also give up task!\n");
											my_follower = -1;
											if (curr_task != -1) {
												prev_task = curr_task;
												curr_task = -1;
											}
											
											rob_state = STATE_ZONE_STAY;
											
											move_type = FORWARD;											
											if (curr_zone >=0 && curr_zone <= 2) {										
												beh_state = BEH_GO_POSITION;										
												int tmp_stay_indx = 8*curr_zone + (8 - rob_num + rob_int_name - 1);
												if (curr_zone == 1)
													tmp_stay_indx = 8*curr_zone + (rob_int_name - 1);
												target[0] = zone_stay[tmp_stay_indx][0];
												target[1] = zone_stay[tmp_stay_indx][1];											
												target[2] = 0;
												
												dist_margin = 0.1;
												angle_margin = 3;
											}
											else
												beh_state = BEH_STOP;
											
										}										
										else {								
											rob_state = STATE_GET_MASTER;
											beh_state = BEH_GO_POSITION;
											move_type = FORWARD;								
											
											dist_margin = 0.7;
											angle_margin = 0.1;
											target[0] = next_target[0];
											target[1] = next_target[1];
										}
									}
								}							
								else {														
									empty_flag = 0;
									if (curr_task == TASK1_2) {										
										my_leader = (int)assign_receive_data[2];																																	
//										printf("My leader is %d\n", my_leader);
										rob_state = STATE_GET_MASTER;	
										beh_state = BEH_STOP;
										if (come_flag == 1) {
											come_flag = 0;
	//										printf("Okay I am coming\n");
											if (curr_zone != target_zone) {
												rob_state = STATE_GET_MASTER;
												beh_state = BEH_GO_POSITION;
												move_type = FORWARD;
												zone_arrival = 0;						
												curr_zone = get_zone(target, curr_zone, target_zone);
												dist_margin = 1;
												angle_margin = 1;
											}
											else {						
												zone_arrival = 0;				
												if (rob_state != STATE_EMPTY_BIN) {
													rob_state = STATE_GET_MASTER;				
													beh_state = BEH_GO_POSITION;							
													move_type = FORWARD;							
													target[0] = rob_pos[0];
													target[1] = rob_pos[1];							
													dist_margin = 1;
												}													
											}
										}
									}
									else if (curr_task == TASK3_1) {										
										wb_servo_set_position(tong_servo, 0);								
										wb_servo_set_position(tong_servo2, 0);
										hold_dist = 0;
										grip_fail_flag = 0;
										
										tray_block_cnt[0] = (int)assign_receive_data[5];
										tray_block_cnt[1] = (int)assign_receive_data[6];
										tray_block_cnt[2] = (int)assign_receive_data[7];
										printf("[[Tray2 blk=%d / Tray3 blk=%d / Tray4 blk=%d\n", tray_block_cnt[0], tray_block_cnt[1], tray_block_cnt[2]);
										if (rob_pos[1] > 13.5) 
											sorting_step = -1;
										else {						
											sorting_step = -2;
											hold_dist = -MY_HALF_PI;
											wb_servo_set_position(tong_servo, hold_dist);								
											wb_servo_set_position(tong_servo2, -hold_dist);
										}
										coll_cnt = (int)assign_receive_data[4];
										printf("coll_cnt=%d\n", coll_cnt);
										rob_state = STATE_SORT;
										beh_state = BEH_GO_POSITION;					
										rob_vel[0] = 0;
										rob_vel[1] = 0;
										rob_vel[2] = 0;	
										
										if (sorting_step == -2) {
											if (curr_zone == 0 || curr_zone == 2) {
												double dist1 = pow(rob_pos[0] - station_node_left[0][0], 2) + pow(rob_pos[1] - station_node_left[0][1], 2);
												double dist2 = pow(rob_pos[0] - station_node_right[0][0], 2) + pow(rob_pos[1] - station_node_right[0][1], 2);
												if (dist1 < dist2) {
													target[0] = station_node_left[0][0];
													target[1] = station_node_left[0][1];
												}
												else {
													target[0] = station_node_right[0][0];
													target[1] = station_node_right[0][1];
												}
												sorting_step = -1;
											}								
											else {
												double dist1 = pow(rob_pos[0] - tray1_left[0][0], 2) + pow(rob_pos[1] - tray1_left[0][1], 2);
												double dist2 = pow(rob_pos[0] - tray1_right[0][0], 2) + pow(rob_pos[1] - tray1_right[0][1], 2);
												if (dist1 < dist2) {
													target[0] = tray1_left[0][0];
													target[1] = tray1_left[0][1];
												}
												else {
													target[0] = tray1_right[0][0];
													target[1] = tray1_right[0][1];
												}
												sorting_step = -2;
											}
											move_type = FORWARD;								
											dist_margin = 0.2;
											angle_margin = 0.2;
										}	
										else {
											target[0] = rob_pos[0];
											target[1] = rob_pos[1];
											target[2] = rob_pos[2];
											dist_margin = 5;
											angle_margin = 3;
										}
									}
									bid_state = BID_DISABLE;
								}
								block_cnt = 0;	
							}							
							else {							
								if (my_follower != rob_int_name) {
									beh_state = BEH_LOAD_BLOCK;
									load_obj_step = 1;
									target[0] = rob_pos[0];
									target[1] = rob_pos[1];
									target[2] = rob_pos[2];
									move_type = FORWARD;
								}
								else {
									int task_done_flag = 0;
									if (left_block == 0) {
										task_done_flag = 1; 
										rob_cap[0][CAP_GRIP] = caripper_grip_qual;
										rob_state = STATE_ZONE_STAY;
										
										beh_state = BEH_GO_POSITION;
										move_type = FORWARD;									
										int tmp_stay_indx = 8*curr_zone + (8 - rob_num + rob_int_name - 1);
										if (curr_zone == 1)
											tmp_stay_indx = 8*curr_zone + (rob_int_name - 1);
										target[0] = zone_stay[tmp_stay_indx][0];
										target[1] = zone_stay[tmp_stay_indx][1];
										target[2] = 0;
										
										if (curr_task != -1) {
											prev_task = curr_task;
											curr_task = -1;
											strt_flag = 1;
											printf("\n (3)Compete! strt_flag = 1\n");
										}
										printf("[[ Rob %d Complete Task1_1 and Task1_2!! ]]\n", rob_int_name);	
																			
										my_follower = -1;
										my_leader = -1;
										dist_margin = 0.1;
										angle_margin = 3;
									}
									else {
										dist_margin = 0.1;
										angle_margin = 0.1;									
										
										double min_dist = 10000;
										for (int bc=0; bc<tot_block; bc++) {
											if (block_list[bc][2] != 10000)
												block_list[bc][2] = pow(rob_pos[0] - block_list[bc][0], 2) + pow(rob_pos[1] - block_list[bc][1], 2);
											if (min_dist > block_list[bc][2]) {
												min_dist = block_list[bc][2];
												min_dist_block_indx = bc;
											}
										}
//										printf("(5)Rob %d go to get block indx=%d\n", rob_int_name, (int)block_list[min_dist_block_indx][3]);
										block_list[min_dist_block_indx][2] = 10000;
										target[0] = block_list[min_dist_block_indx][0];
										target[1] = block_list[min_dist_block_indx][1];
										target_block_indx = (int)block_list[min_dist_block_indx][4];
										
										rob_state = STATE_GET_GARBAGE;
										double r_val = drand();
										printf("(Grip prob_2) r_val=%.2f thre=%.2f\n", r_val, caripper_grip_qual);
										if (no_prob_flag == 1)
											r_val = 0;			
										
										if (switch_grip_fail == 0) {
											if (r_val >= caripper_grip_qual) {
												grip_fail_flag = 1;								
												printf("[[[Fail block grip!!]]\n");
											}
											else
												grip_fail_flag = 0;																				
										}
										else {												
		
											if (grip_fail_flag == 1)
												grip_fail_flag = 0;
											else
												grip_fail_flag = 1;
										}

										
										beh_state = BEH_GO_POSITION;
										move_type = FORWARD;																				
									}
									//double send_message[send_cnt][5];		
									send_message[send_cnt][0] = DONE_MESSAGE;
									send_message[send_cnt][1] = (double)curr_zone;
									send_message[send_cnt][2] = (double)rob_int_name;
									send_message[send_cnt][3] = (double)block_cnt;
									send_message[send_cnt][4] = (double)task_done_flag;
									send_message[send_cnt][14] = 5;
									send_cnt++;
									
									block_cnt = 0;						
								}
							}
						}
					}
					break;
				case BEH_LOAD_BLOCK:	
					avoid_flag = 0;
					block_avoid_flag = 0;
					tray1_avoid_flag = 0;
					reach = go_target(rob_pos, target, rob_vel, wheel_vel, angle_margin, dist_margin, move_type, avoid_flag, block_avoid_flag, tray1_avoid_flag);					
					if (grip_fail_flag == 1) {
						reach = 1;
						load_obj_step = 2;
					}
					if (reach == 1)
					{
						rob_vel[0] = 0;
						rob_vel[1] = 0;
						rob_vel[2] = 0;											
						hold_dist = 0;
						up_dist = 0;												
						if (load_obj_step == 0) {
							angle_margin = 0.1;
							beh_state = BEH_PUSH_PANEL;
							hold_dist = 0;		
							wb_servo_set_position(tong_servo, hold_dist);								
							wb_servo_set_position(tong_servo2, -hold_dist);
							push_max_dist = 0.4;
							back_dist = 0.5;
							wb_differential_wheels_set_speed(0, 0);
						}						
						else if (load_obj_step == 1) {
							target[0] = rob_pos[0];
							target[1] = rob_pos[1];
							target[2] = get_good_ang(rob_pos[2] + MY_PI);
							move_type = ROTATION;
							load_obj_step = 2;
						}
						else {
							rob_cap[0][CAP_GRIP] = caripper_grip_qual;
							load_obj_step = 0;							
														
													
							beh_state = BEH_GO_POSITION;							
							move_type = FORWARD;		
							block_cnt = 0;
							
							if (min_dist_block_indx != -1) {
								left_block--;
							}

							if (left_block > 0) {								
								beh_state = BEH_STOP;						
								empty_flag = 0;
								send_message[send_cnt][0] = LOAD_MESSAGE;
								send_message[send_cnt][1] = (double)my_follower;
								if (grip_fail_flag == 1) {
									send_message[send_cnt][1] = -1;
									beh_state = BEH_GO_POSITION;
								}
								send_message[send_cnt][2] = (double)empty_flag;
								send_message[send_cnt][3] = (double)curr_task;
								if (min_dist_block_indx != -1) {
								//	printf("(2)Send load message for block id[%d]\n", (int)block_list[min_dist_block_indx][3]);																	
									send_message[send_cnt][4] = block_list[min_dist_block_indx][3];
								}
								else {
									printf("(2)I did not find min block, so block indx is -1\n");
									send_message[send_cnt][4] = -1;
								}																
								send_message[send_cnt][5] = (double)grip_fail_flag;
								send_message[send_cnt][14] = 6;
								send_cnt++;
								hold_dist = -MY_HALF_PI;
								wb_servo_set_position(tong_servo, -MY_HALF_PI);								
								wb_servo_set_position(tong_servo2, MY_HALF_PI);
								
								hold_dist = 0;
//								printf("(2)Block cnt=%d / Left block=%d\n", block_cnt, left_block);

								rob_state = STATE_GET_GARBAGE;
								
								double r_val = drand();
								printf("(Grip prob_3) r_val=%.2f thre=%.2f\n", r_val, caripper_grip_qual);
								if (no_prob_flag == 1)
									r_val = 0;			
								
								if (switch_grip_fail == 0) {
									if (r_val >= caripper_grip_qual) {
										grip_fail_flag = 1;								
										printf("[[[Fail block grip!!]]\n");
									}
									else
										grip_fail_flag = 0;																				
								}
								else {												
									
									if (grip_fail_flag == 1)
										grip_fail_flag = 0;
									else
										grip_fail_flag = 1;
								}
								
							}
							else {
								printf("[[Left block is zero! state_zone_Stay!\n", left_block);
								
								empty_flag = 1;																	
								block_cnt = 0;
								//rob_cap[0][CAP_GRIP] = caripper_grip_qual;
								rob_state = STATE_ZONE_STAY;
								hold_dist = -MY_HALF_PI;
								wb_servo_set_position(tong_servo, -MY_HALF_PI);								
								wb_servo_set_position(tong_servo2, MY_HALF_PI);
								int tmp_stay_indx = 8*curr_zone + (8 - rob_num + rob_int_name - 1);
								if (curr_zone == 1)
									tmp_stay_indx = 8*curr_zone + (rob_int_name - 1);
								target[0] = zone_stay[tmp_stay_indx][0];
								target[1] = zone_stay[tmp_stay_indx][1];
								target[2] = 0;

								if (grip_fail_flag == 1) {
									beh_state = BEH_GO_POSITION;
									printf("Since Rob %d fail to grip last block, go zone stay!\n", rob_int_name);
								}
								else
									beh_state = BEH_STOP;

								move_type = FORWARD;
								
								dist_margin = 0.1;
								angle_margin = 3;
//								printf("(3)Send load message for block id[%d]\n", (int)block_list[min_dist_block_indx][3]);															
								send_message[send_cnt][0] = LOAD_MESSAGE;
								
								if (min_dist_block_indx != -1) { 
									send_message[send_cnt][1] = (double)my_follower;
									send_message[send_cnt][2] = (double)empty_flag;
									send_message[send_cnt][3] = (double)curr_task;									
									if (min_dist_block_indx != -1) {
//										printf("(2)Send load message for block id[%d]\n", (int)block_list[min_dist_block_indx][3]);																	
										send_message[send_cnt][4] = block_list[min_dist_block_indx][3];
									}
									else {
										printf("(2)I did not find min block, so block indx is -1\n");
										send_message[send_cnt][4] = -1;
									}
									send_message[send_cnt][5] = (double)grip_fail_flag;
									send_message[send_cnt][14] = 6;
									send_cnt++;
									printf("(3)Block cnt=%d / Left block=%d\n", block_cnt, left_block);
								}	
								if (empty_flag == 1) {
									my_follower = -1; 
									block_cnt = 0;
								}
							}
							
							
							
							//wb_emitter_send(emitter, send_message[send_cnt], 4*sizeof(double));
							if (empty_flag == 1) {
								my_follower = -1;						
								empty_flag = 0;
							}

							/*
							double r_val = drand();
							if (no_prob_flag == 1)
								r_val = 0;

							if (r_val >= rob_cap[0][CAP_GRIP]) {
								grip_fail_flag = 1;								
								printf("[[[Fail block grip!!, r_val=%.2f]]\n", r_val);
							}
							else {
								grip_fail_flag = 0;
								printf("[[[Normal block grip!!, r_val=%.2f]]\n", r_val);
							}							
							*/
						}		
					}
					//2.back
					break;
				case BEH_PICK_UP:
					if (pick_up_step == 0) {
						avoid_flag = 0;
						block_avoid_flag = 0;
						tray1_avoid_flag = 0;
						int front_reach = go_target(rob_pos, target, rob_vel, wheel_vel, angle_margin, dist_margin, move_type, avoid_flag, block_avoid_flag, tray1_avoid_flag);
						if (front_reach == 1) {
							rob_vel[0] = 0;
							rob_vel[1] = 0;
							rob_vel[2] = 0;
							pick_up_step = 1;							
						}
					}
					else if (pick_up_step == 1) {						
						if (hold_dist > -2.8) {
							hold_dist -= hold_step;									
							dist_margin = 0.5;
							angle_margin = 0.5;
							wb_differential_wheels_set_speed(0, 0);	
						}
						else {
							pick_up_step = 2;
							hold_dist = 0;
						}
						wb_servo_set_position(tong_servo, hold_dist);								
						wb_servo_set_position(tong_servo2, -hold_dist-0.02);
					}
					else {	
						hold_dist = -MY_HALF_PI;
						wb_servo_set_position(tong_servo, -MY_HALF_PI);								
						wb_servo_set_position(tong_servo2, MY_HALF_PI);
						
						block_cnt++;
						printf("[3]Block cnt=%d\n", block_cnt);
						dist_margin = 0.1;
						angle_margin = 0.1;									
						
							
						if (my_leader != -1) {
							send_message[send_cnt][0] = LOAD_DONE_MESSAGE;
							send_message[send_cnt][1] = (double)my_leader;
							send_message[send_cnt][14] = 2;
							send_cnt++;
							printf("[Send load_done_message to my leader %d\n", my_leader);
						}
						if (block_cnt < max_block_contain && empty_flag == 0 && rob_state != STATE_NEW_TASK) {							
							rob_state = STATE_GET_MASTER;
							beh_state = BEH_STOP;							
							dist_margin = 0.7;
							angle_margin = 0.1;
							if (empty_flag == 1) {
								my_leader = -1;
								
								if (curr_task != -1) {
									prev_task = curr_task;
									curr_task = -1;
									strt_flag = 1;
									printf("\n (4)Compete! strt_flag = 1\n");
								}
								printf("Reset leader2 and reset task\n");
							}
							if (my_leader == -1) {
								printf("Hull(3)! Leader left! I also give up task!\n");
								my_follower = -1;
								
								if (curr_task != -1) {
									prev_task = curr_task;
									curr_task = -1;
									strt_flag = 1;
									printf("\n (5)Compete! strt_flag = 1\n");
								}
								rob_cap[0][CAP_GRIP] = caripper_grip_qual;
								rob_state = STATE_ZONE_STAY;
								
								move_type = FORWARD;
								
								
								if (curr_zone >=0 && curr_zone <= 2) {										
									beh_state = BEH_GO_POSITION;										
									int tmp_stay_indx = 8*curr_zone + (8 - rob_num + rob_int_name - 1);
									if (curr_zone == 1)
										tmp_stay_indx = 8*curr_zone + (rob_int_name - 1);
									target[0] = zone_stay[tmp_stay_indx][0];
									target[1] = zone_stay[tmp_stay_indx][1];
									target[2] = 0;

									dist_margin = 0.1;
									angle_margin = 3;
								}
								else
									beh_state = BEH_STOP;
							}
						}
						else {															
							tot_block += block_cnt;
							pick_up_step = 0;							
							rob_state = STATE_EMPTY_BIN;		 
							beh_state = BEH_STORE_OBJ;								
							target[0] = tray1_zone[curr_zone][0];
							target[1] = tray1_zone[curr_zone][1];
							target[2] = tray1_zone[curr_zone][2];		

							double r_val = drand();
							if (no_prob_flag == 1)
								r_val = 0;
							printf("(Load tray1 prob_2) r_val=%.2f / thres=%.2f\n", r_val, rob_cap[0][CAP_LOC] - cap_loc_dec);							
							if (r_val >= rob_cap[0][CAP_LOC] - cap_loc_dec) {
								r_val -= (rob_cap[0][CAP_LOC] - cap_loc_dec);
								printf("[Error dist = %.2f\n", r_val);								
								if (curr_zone == 0)
									target[0] += r_val;
								else if (curr_zone == 1)
									target[1] -= r_val;
								else if (curr_zone == 2)
									target[0] -= r_val;
							}
							dist_margin = 0.1;
							angle_margin = 0.1;

							if (tray_full_flag == 1) {
								printf("[_1_[Tray 1 is full so I can't store block to Tray1\n");
								dist_margin = 0.3;
								angle_margin = 3;
								move_type = FORWARD;
							}		
							hold_dist = -MY_HALF_PI;
							wb_servo_set_position(tong_servo, -MY_HALF_PI);								
							wb_servo_set_position(tong_servo2, MY_HALF_PI);
						}	
					}																
					break;
				case BEH_STOP:					
					wb_differential_wheels_set_speed(0, 0);     					
					break;				
			}		
		}
		
		if (auction_state == AUCTION_ENABLE) {
			//double send_message[send_cnt][11];//auction_message / auction_task / auctioneer_id / ...
			send_message[send_cnt][0] = AUCTION_MESSAGE;				
			send_message[send_cnt][1] = (double)auction_task;
			send_message[send_cnt][2] = (double)rob_int_name;			
			send_message[send_cnt][3] = (double)team_auc_flag;
			for (int i=0; i<auction_data_size; i++)
				send_message[send_cnt][4+i] = auction_data[i];
			send_message[send_cnt][14] = 4 + auction_data_size;
			send_cnt++;			
			auction_state = AUCTION_WAIT_BIDDER;
			
			double bid_value[2];						
			double esti_dist = 0;
			int task_available = 0;
			if (auction_task == TASK1_2) {
				task_available = get_bid_values(rob_cap, task_req2, esti_dist, curr_energy, bid_value);
			}
			if (task_available == 1) {
				int t_indx = get_task_indx(auction_task);
				receive_bidders[bid_cnt][0] = (double)auction_task;//task
				receive_bidders[bid_cnt][1] = (double)rob_int_name;//auctioneer_id
				receive_bidders[bid_cnt][2] = (double)rob_int_name;//bidder_id												
				receive_bidders[bid_cnt][3] = bid_value[0];//quality				
				receive_bidders[bid_cnt][4] = bid_value[1];//cost	
				receive_bidders[bid_cnt][5] = task_dist[t_indx];//cost	
				receive_bidders[bid_cnt][6] = task_time[t_indx];//cost	
				receive_bidders[bid_cnt][7] = bid_weight;//cost	
				receive_bidders[bid_cnt][8] = curr_energy;
				bidder_ID[bid_cnt] = rob_int_name;				
				printf("[cnt=%d T=%d/id=%d(me)/q=%.2f/c=%.2f/e=%.2f/d2=%.2f/t2=%.2f]\n", bid_cnt, 
					(int)receive_bidders[bid_cnt][0], (int)receive_bidders[bid_cnt][2], receive_bidders[bid_cnt][3], 
					receive_bidders[bid_cnt][4], receive_bidders[bid_cnt][8], 
					receive_bidders[bid_cnt][5], receive_bidders[bid_cnt][6]);										
				bid_cnt++;				
				if (bid_state != BID_WAIT_FOLLOWER)
					bid_state = BID_WAIT_ACCEPT;
			}
		}
		else if (auction_state == AUCTION_WAIT_BIDDER) {
			wait_cnt++;	
			if (wait_cnt > AUCTION_WAIT_TIME) {
				wait_cnt = 0;
				if (bid_cnt > 0)
					auction_state = AUCTION_GET_BIDDER;				
				else {
					auction_state = AUCTION_ENABLE;
					//printf("[[No bidder! Auction again]]\n");					
				}
			}		
		}		
		else if (auction_state == AUCTION_GET_BIDDER) {						
			double bid_utility[NUM_ROBOT];											
			int winner = 0;
			double winner_utility = 0;
			double winner_qual = 0;
			double winner_cost = 0;
			double winner_engy = 0;

			if (test_mode >= 3 && test_mode <= 4)
				bid_weight = get_ave_bid_weight(bid_weight, receive_bidders, bid_cnt);
			if (bid_cnt == 1) {
				winner = (int)receive_bidders[0][2];
				winner_qual = receive_bidders[0][3];
				winner_cost = receive_bidders[0][4];				
				winner_engy = receive_bidders[0][8];//energy
				printf("Only one bidder, %d is winner! / bid_weight=%.2f\n", winner, bid_weight);				
				winner_utility = 0.5 * (bid_weight + 1.0);
			}
			else {								
				if (test_mode == 4) {
					double eng_ave = 0;
					double eng_std = 0;
					double tmp_sum = 0;
					for (int g=0; g<bid_cnt; g++) {
						eng_ave += receive_bidders[g][8];//energy
					}
					eng_ave /= bid_cnt;
					for (g=0; g<bid_cnt; g++)
						tmp_sum += pow(eng_ave - receive_bidders[g][8], 2);
					eng_std = sqrt(tmp_sum/(double)bid_cnt);
					s_weight = s_weight_max*(find_min(eng_std, std_max)/std_max);
					printf("eng_std = %.2f / S_weight = %.2f\n", eng_std, s_weight);
				}
				double max_utility = 0;
				get_utility(receive_bidders, bid_utility, bid_weight, s_weight, bid_cnt);
				for (int i=0; i<bid_cnt; i++) {
					if (max_utility < bid_utility[i]) {
						max_utility = bid_utility[i];
						winner = (int)receive_bidders[i][2];//bidder_id						
						winner_qual = receive_bidders[i][3];//quality
						winner_cost = receive_bidders[i][4];//cost
						winner_engy = receive_bidders[i][8];//energy
					}
				}
				if (winner != receive_bidders[bid_cnt-1][2] && max_utility == bid_utility[bid_cnt-1]) {
					int winner_idx = irand(bid_cnt);
					max_utility = bid_utility[winner_idx];
					winner = (int)receive_bidders[winner_idx][2];
					winner_qual = receive_bidders[winner_idx][3];//quality
					winner_cost = receive_bidders[winner_idx][4];//cost
					winner_engy = receive_bidders[winner_idx][8];//energy
					printf("Select winner by random! Winner is %d\n", winner);
				}
				winner_utility = max_utility;
				printf("%d bidders, %d is winner! / bid_weight=%.2f\n", bid_cnt, winner, bid_weight);
			}

			get_d2t2(receive_bidders, ave_d2t2, bid_cnt);
			
			bid_cnt = 0;
			if (bid_state == BID_WAIT_FOLLOWER) {								
				final_winner = winner;
				final_winner_utility = winner_utility;

				if (final_winner != rob_int_name) {
					printf("[[I bid my auctioned task1_2 but rejected!\n");
					int t_indx = get_task_indx(TASK1_2);
					update_cnt[t_indx]++;
					printf("\nReceived d2=%.2f / t2=%.2f (cnt=%d)\n", ave_d2t2[0], ave_d2t2[1], update_cnt[t_indx]);																
					update_cost(TASK1_2, ave_d2t2[0], ave_d2t2[1], task_dist, task_time, 1, update_cnt[t_indx]);					
				}
				double bid_value[2];
				double esti_dist = get_dist(rob_pos, bid_task, curr_zone, -1);
				int task_available = 0;
				task_available = get_bid_values(rob_cap, task_req1, esti_dist, curr_energy, bid_value);			

				int t_indx = get_task_indx(bid_task);
				send_message[send_cnt][0] = BID_MESSAGE;					
				send_message[send_cnt][1] = (double)bid_task;
				send_message[send_cnt][2] = 9;
				send_message[send_cnt][3] = (double)rob_int_name;					
				send_message[send_cnt][4] = (bid_value[0] + winner_qual);
				send_message[send_cnt][5] = (bid_value[1] + winner_cost);
				send_message[send_cnt][6] = task_dist[t_indx];
				send_message[send_cnt][7] = task_time[t_indx];
				send_message[send_cnt][8] = bid_weight;
				send_message[send_cnt][9] = curr_energy + winner_engy;
				send_message[send_cnt][14] = 10;	
				send_cnt++;
				bid_state = BID_WAIT_ACCEPT;
				printf("Sum (%.2f + %.2f), (%.2f + %.2f), (%.2f + %.2f) \n", bid_value[0], winner_qual, bid_value[1], winner_cost,
					curr_energy, winner_engy);
				printf("[[Rob %d bids Task %d of auctioneer %d: q=%.2f/c=%.2f/e=%.2f/d2=%.2f/t2=%.2f\n", 
					rob_int_name, bid_task, 9, send_message[send_cnt-1][4], send_message[send_cnt-1][5], send_message[send_cnt-1][9],
					task_dist[t_indx], task_time[t_indx]);									
				
				bid_task = -1;
			}
			else {				
				send_message[send_cnt][0] = ASSIGN_MESSAGE;			
				send_message[send_cnt][1] = (double)auction_task;			//task
				send_message[send_cnt][2] = (double)rob_int_name;			//auctioneer_id
				send_message[send_cnt][3] = (double)winner;				//bidder_id
				send_message[send_cnt][4] = ave_d2t2[0];
				send_message[send_cnt][5] = ave_d2t2[1];			
				send_message[send_cnt][6] = bid_weight;	
				send_message[send_cnt][7] = winner_utility;	
				send_message[send_cnt][14] = 8;
				send_cnt++;
				my_follower = winner;														
				printf("[1]Assign robot %d task %d, (%.2f, %.2f)]\n", winner, auction_task, rob_pos[0], rob_pos[1]);						
				printf("[6] D2T2=(%.2f / %.2f)\n", ave_d2t2[0], ave_d2t2[1]);

				if (auction_task == TASK1_2 && rob_state == STATE_GET_SLAVE && my_follower != rob_int_name) {		
					send_message[send_cnt][0] = COME_MESSAGE;
					send_message[send_cnt][1] = (double)rob_int_name;	//from
					send_message[send_cnt][2] = (double)my_follower;			
					send_message[send_cnt][3] = rob_pos[0];
					send_message[send_cnt][4] = rob_pos[1];						
					send_message[send_cnt][5] = (double)curr_zone;	
					send_message[send_cnt][14] = 6;
					send_cnt++;										
					printf("[And send come_message!!]\n");
				}
				else if (auction_task == TASK3_2) {
					printf("I got a follower rob %d\n", winner);
					if (sorting_step == -11)
						sorting_step = 6;					
					else if (sorting_step == -19)
						sorting_step = 4;
				}

				if (auction_task == TASK1_2 && winner == rob_int_name) {					
					curr_pre_task = curr_task;
					printf("Store prev task %d\n", curr_pre_task);
					curr_task = auction_task;
					printf("[[I am the auctioned winner!\n");										
					block_cnt = 0;						
					printf("Reset block cnt=0\n");
					my_leader = rob_int_name;												
					my_follower = rob_int_name;	
					target[0] = rob_pos[0];
					target[1] = rob_pos[1];
					dist_margin = 10;
					angle_margin = 10;
					rob_state = STATE_GET_GARBAGE;							
					beh_state = BEH_GO_POSITION;
					move_type = FORWARD;
					
					
					send_message[send_cnt][0] = ASSIGN_MESSAGE;			
					send_message[send_cnt][1] = TASK1_2;			//task
					send_message[send_cnt][2] = (double)rob_int_name;			//auctioneer_id
					send_message[send_cnt][3] = -1;								//bidder_id
					send_message[send_cnt][4] = ave_d2t2[0];
					send_message[send_cnt][5] = ave_d2t2[1];			
					send_message[send_cnt][6] = bid_weight;			
					send_message[send_cnt][14] = 7;
					send_cnt++;				
					bid_cnt = 0;
					//bid_state = BID_DISABLE;
					auction_task = -1;
					if (bid_state == BID_WAIT_ACCEPT)
						bid_state = BID_DISABLE;
					printf("[1] D2T2=(%.2f / %.2f)\n", ave_d2t2[0], ave_d2t2[1]);
				}								
				auction_task = -1;
			}
			auction_state = AUCTION_DISABLE;
			 
							
		}

		if (wb_receiver_get_queue_length(receiver) > 0 && rob_state != STATE_INIT) {				
			const void* buffer = wb_receiver_get_data(receiver);        			
			double* receive_message;			
			receive_message = (double*)buffer;		
			
			if (receive_message[0] == AUCTION_MESSAGE && (int)receive_message[1] == TASK2_1 && tray_full_flag == 0) {
				tray_full_flag = 1;
				printf("[[Tray1 is full now!]]\n");
				if (beh_state == BEH_STORE_OBJ && move_type == ROTATION)
					move_type = FORWARD;
			}
			if (receive_message[0] == INIT_MESSAGE) {
				rob_num = (int)receive_message[1];
				full_record = (int)receive_message[2];
				test_mode = (int)receive_message[3];

				real_termination_time = (int)receive_message[4];
				int time_gap = (int)floor((double)real_termination_time/(double)10);
				for (int n=0; n<11; n++) {
					store_time[n] = time_gap * n + 1;
					printf("%d ", store_time[n]);
				}
				printf("\n");
				double T1 = 1200;
				int tra = 800;
				for (int x=0; x<real_termination_time; x++) {	
					if (x < tra)
						G[x] = 0.5 - 0.5*cos(0.7*2*3.1415*(double)x/T1);		
					else
						G[x] = G[tra-1] - 0.00005 * (x - tra);	
					G[x] *= (1 + (drand()-0.5)*0.05);
				}

				get_rob_cap(rob_int_name, rob_num, rob_cap);
				caripper_grip_qual = rob_cap[0][CAP_GRIP];

				if (test_mode == 0) {//qbta
					bid_weight = 1;
					s_weight = 0;
				}
				else if (test_mode == 1) {//cbta
					bid_weight = 0;				
					s_weight = 0;
				}
				else if (test_mode == 2) { //ebta
					bid_weight = 0;
					s_weight = 1;
				}
				else if (test_mode >= 3 && test_mode <= 4) {
					bid_weight = scalar * (exp(s_factor*(pow(G[0], sqare_num) - 1.0)) - bias_bid_weight);
					double v_rand = 0.5*drand();
					if (rand()%2 == 0)
						bid_weight += v_rand;
					else
						bid_weight -= v_rand;
					if (bid_weight > 1)
						bid_weight = 1;
					else if (bid_weight < 0)
						bid_weight = 0;
				}
				else if (test_mode == 5) {//price test
					bid_weight = receive_message[5];
					printf("[[ Mode is 5 and Received bid weigh is %.2f\n", bid_weight);									
				}
				else if (test_mode == 6) {//subsidy test					
					s_weight = receive_message[5];
					bid_weight = receive_message[6];
					printf("[[ Mode is 5 and Received s_weigh is %.2f\n", s_weight);
				}

				double tmp_dist[3];
				tmp_dist[0] = pow(rob_pos[0] - zone_stay[(8 - rob_num + rob_int_name-1)][0], 2) + pow(rob_pos[1] - zone_stay[(8 - rob_num + rob_int_name-1)][1], 2);
				tmp_dist[1] = pow(rob_pos[0] - zone_stay[8+(rob_int_name-1)][0], 2) + pow(rob_pos[1] - zone_stay[8+(rob_int_name-1)][1], 2);
				tmp_dist[2] = pow(rob_pos[0] - zone_stay[16+(8 - rob_num + rob_int_name-1)][0], 2) + pow(rob_pos[1] - zone_stay[16+(8 - rob_num + rob_int_name-1)][1], 2);
				double tmp_min_dist = 10000;
				int min_indx = 0;
				for (int i=0; i<3; i++) {
					if (tmp_min_dist > tmp_dist[i]) {
						tmp_min_dist = tmp_dist[i];
						min_indx = i;
					}
				}
				curr_zone = min_indx;				
				printf("Rob %d is in zone %d\n", rob_int_name, curr_zone);
								
			}
			else if (receive_message[0] == AUCTION_MESSAGE) {
				int tmp_received_task = (int)receive_message[1];				
				int tmp_bid_enable_flag = 0;
				if (curr_task == -1) {
					tmp_bid_enable_flag = 1;
				}
				else if (curr_task < tmp_received_task && tmp_received_task > TASK1_ZONE3_3 
					&& (!(curr_task <= TASK1_ZONE3_3 && tmp_received_task == TASK1_2)) 
					&& (!(curr_task == TASK3_1 && tmp_received_task == TASK3_2))) {    				
					
					if (arrive_flag != 1) {
						tmp_bid_enable_flag = 1;
						//printf("[[Auctioned Task is High priority! curr=%d < auc=%d\n", curr_task, (int)receive_message[1]);
					}
					
					if (rob_state == STATE_GET_GARBAGE) {
				
						double tmp_dist = real_dist(rob_pos, target, 1);
						//printf("Tmp dist=%.2f\n", tmp_dist);
						if (tmp_dist > 1)
							tmp_bid_enable_flag = 1;
						else
							tmp_bid_enable_flag = 0;
					}
				}
				if (beh_state == BEH_STORE_OBJ || beh_state == BEH_LOAD_BLOCK || beh_state == BEH_PUSH_PANEL || beh_state == BEH_BACK_POSITION)
					tmp_bid_enable_flag = 0;
			
				if (tmp_bid_enable_flag == 1) {	
					int task_available = 0;					
					double esti_dist = 0;
					double bid_value[2] = {0, 0};

					if (tmp_received_task < TASK1_2 || tmp_received_task == TASK3_1) {							
						esti_dist = get_dist(rob_pos, tmp_received_task, curr_zone, -1);
						task_available = get_bid_values(rob_cap, task_req1, esti_dist, curr_energy, bid_value);							
					}
					if (tmp_received_task == TASK1_2) {
						leader_zone = (int)receive_message[4];		
						esti_dist = get_dist(rob_pos, tmp_received_task, curr_zone, leader_zone);
						task_available = get_bid_values(rob_cap, task_req2, esti_dist, curr_energy, bid_value);							
					}											
					if (task_available == 1) {						
						double tmp_q = -1;
						double tmp_c = -1;
						if ((int)receive_message[3] == INDI_AUCTION) {
							tmp_q = bid_value[0];
							tmp_c = bid_value[1];
						}
						int checker = -1;
						if (task_cnt > 0)
							checker = get_match(tmp_received_task, (int)receive_message[2], task_cnt, tmp_q, tmp_c, bid_weight, esti_dist, receive_tasks);

						int bid_store_flag = 0;
						
						if (checker == -1 || checker == -2) {
							printf("New task! curr_task %d / new_task %d from auctioneer %d\n", curr_task, tmp_received_task, (int)receive_message[2]);							
							if (checker == -1) {
								printf("Highest priority task!! (task_cnt=%d) remove previous tasks!\n", task_cnt);
								task_cnt = 0;
								bid_store_flag = 1;
							}
							else if (checker == -2) {
								printf("Same priority task!\n");
								printf("Task %d (team form) is stored to the bidding memory!\n", tmp_received_task);
								bid_store_flag = 1;								
							}
							receive_tasks[task_cnt][0] = receive_message[1];						
							receive_tasks[task_cnt][1] = receive_message[2];
							receive_tasks[task_cnt][2] = 0;//bidding flag
							receive_tasks[task_cnt][3] = receive_message[3];
							receive_tasks[task_cnt][4] = bid_value[0];
							receive_tasks[task_cnt][5] = bid_value[1];
							receive_tasks[task_cnt][6] = esti_dist;
							task_cnt++;
							bid_wait_cnt = BID_WAIT_TIME;		
						}
						else if (checker == -3) {
							printf("Task exchange in memory! Task %d util is higher than tasks in memory\n", tmp_received_task);
							bid_store_flag = 1;
						}
						if (bid_store_flag == 1) {
							if (tmp_received_task < TASK1_2) {
								printf("This is TASK1_1 (%d)! So, Auction TASK1_2\n", tmp_received_task);									
								bid_state = BID_WAIT_FOLLOWER;
								if (auction_state != AUCTION_DISABLE) {
									printf("Oh my god! I already auctioned Task1_2! Reject bidders!\n");						
									my_follower = -1;
									send_message[send_cnt][0] = ASSIGN_MESSAGE;			
									send_message[send_cnt][1] = (double)TASK1_2;				//task
									send_message[send_cnt][2] = (double)rob_int_name;			//auctioneer_id
									send_message[send_cnt][3] = -1;								//fake bidder_id
									send_message[send_cnt][4] = ave_d2t2[0];
									send_message[send_cnt][5] = ave_d2t2[1];
									send_message[send_cnt][6] = bid_weight;			
									send_message[send_cnt][14] = 7;
									send_cnt++;							
									bid_cnt = 0;
									printf("[2] D2T2=(%.2f / %.2f)\n", ave_d2t2[0], ave_d2t2[1]);
								}
								bid_task = tmp_received_task;
								
								if (tmp_received_task <= TASK1_ZONE1_3)
									target_zone = 0;
								else if (tmp_received_task <= TASK1_ZONE2_3)
									target_zone = 1;
								else if (tmp_received_task <= TASK1_ZONE3_3)
									target_zone = 2;
								auction_task = TASK1_2;
								auction_data[0] = (double)target_zone;																
								auction_data_size = 1;
								auction_state = AUCTION_ENABLE;	
								team_auc_flag = TEAM_FORM_AUCTION;										
							}															
						}
					}									
				}
			}
			else if (receive_message[0] == BID_MESSAGE) {				
				if (auction_state == AUCTION_WAIT_BIDDER) {//receive_bidders: (0)task / (1)auctioneer_id / (2)bidder_id / (3)quality / (4)cost
					if ((int)receive_message[1] == auction_task && (int)receive_message[2] == rob_int_name) {
						
						int tmp_rob_ID = (int)receive_message[3];
						int ID_check = find_ID(bid_cnt, tmp_rob_ID, bidder_ID);
						if (ID_check == 0) {
							for (int i=0; i<9; i++)
								receive_bidders[bid_cnt][i] = receive_message[i+1];								
							bidder_ID[bid_cnt] = tmp_rob_ID;
							printf("[cnt=%d T=%d/id=%d/q=%.2f/c=%.2f/e=%.2f/d2=%.2f/t2=%.2f/Wb=%.2f]\n", bid_cnt, 
								(int)receive_bidders[bid_cnt][0], (int)receive_bidders[bid_cnt][2], receive_bidders[bid_cnt][3], 
								receive_bidders[bid_cnt][4], receive_bidders[bid_cnt][8], 
								receive_bidders[bid_cnt][5], receive_bidders[bid_cnt][6], receive_bidders[bid_cnt][7]);										
							bid_cnt++;
						}
					}				
				}
			}
			else if (receive_message[0] == ASSIGN_MESSAGE) {//(0)assign_message / (1)auction_task / (2)auctioneer_id / (3)bidder_id													
				int rcv_task = (int)receive_message[1];
				int rcv_actioneer = (int)receive_message[2];//auctioneer id
				int rcv_winner = (int)receive_message[3];
				int match_check_flag = find_match(rcv_task, rcv_actioneer, task_cnt, receive_tasks);
				if (rcv_winner == rob_int_name && match_check_flag == 0) {
					
					send_message[send_cnt][0] = ABANDON_MESSAGE;
					send_message[send_cnt][1] = (double)rcv_task;
					send_message[send_cnt][2] = (double)rob_int_name;
					send_message[send_cnt][14] = 3;
					send_cnt++;
					printf("I (=bidder) reject the accept task %d\n", rcv_task);
				}
				
				if (match_check_flag == 1) {
					if (bid_state == BID_WAIT_ACCEPT || bid_state == BID_WAIT_FOLLOWER) {
						if (rcv_winner == rob_int_name) {
							wait_flag = 0;
							double rcv_d2 = receive_message[4];
							double rcv_t2 = receive_message[5];
							int t_indx = get_task_indx(rcv_task);
							update_cnt[t_indx]++;
							printf("\nReceived d2=%.2f / t2=%.2f (cnt=%d)\n\n", rcv_d2, rcv_t2, update_cnt[t_indx]);													
							update_cost(rcv_task, rcv_d2, rcv_t2, task_dist, task_time, 1, update_cnt[t_indx]);													

							if (arrive_flag == 1) {
								printf("***** I just accepted task %d but give up !!\n", rcv_task);
								send_message[send_cnt][0] = ABANDON_MESSAGE;
								send_message[send_cnt][1] = (double)rcv_task;
								send_message[send_cnt][2] = (double)rob_int_name;
								send_message[send_cnt][14] = 3;
								send_cnt++;								
							}
							else {
								printf("Rob %d accepted Task %d!\n", rob_int_name, rcv_task);
								come_cnt = 0;
								prev_task = curr_task;								
								
								if (curr_task == -1)
									curr_task = rcv_task;
								else {
									auction_state = AUCTION_DISABLE;
									printf("I give up task %d\n", curr_task);
									if (curr_task == TASK1_2 && my_follower == rob_int_name) {
										printf("I also give up task %d\n", curr_pre_task);
										curr_task = curr_pre_task;
										curr_pre_task = -1;								
									}
									send_message[send_cnt][0] = ABANDON_MESSAGE;
									send_message[send_cnt][1] = (double)curr_task;
									send_message[send_cnt][2] = (double)rob_int_name;
									send_message[send_cnt][14] = 3;
									send_cnt++;							
									curr_task = rcv_task;							
								}

								my_leader = -1;
								my_follower = -1;
								if (curr_task < TASK1_2) {
									my_follower = final_winner;
//									printf("My follower is %d\n", my_follower);	
								}
								else if (curr_task == TASK1_2) {
									my_leader = (int)receive_message[2];															
//									printf("My leader is %d\n", my_leader);	
									task1_2_checker = 0;
								}
								
								if (block_cnt > 0) {
									for (int i = 0; i<10; i++)
										assign_receive_data[i] = receive_message[i];							
									if (beh_state != BEH_PUSH_PANEL && beh_state != BEH_BACK_POSITION) {
										rob_state = STATE_NEW_TASK;
										beh_state = BEH_STORE_OBJ;										
										target[0] = tray1_zone[curr_zone][0];
										target[1] = tray1_zone[curr_zone][1];
										target[2] = tray1_zone[curr_zone][2];
										
										double r_val = drand();
										if (no_prob_flag == 1)
											r_val = 0;
										
										printf("(Carry tray prob_3) r_val=%.2f / thres=%.2f\n", r_val, rob_cap[0][CAP_LOC] - cap_loc_dec);
										if (r_val >= rob_cap[0][CAP_LOC] - cap_loc_dec) {
											r_val -= (rob_cap[0][CAP_LOC] - cap_loc_dec);
											if (curr_zone == 0)
												target[0] += r_val;
											else if (curr_zone == 1)
												target[1] -= r_val;
											else if (curr_zone == 2)
												target[0] -= r_val;
										}
										
										dist_margin = 0.1;
										angle_margin = 0.1;								
										move_type = FORWARD;	
									}
								}
								else {
									if (rob_state != STATE_ZONE_STAY && rob_state != STATE_READY) {									
										if (fabs(rob_pos[0]) < 4 && rob_pos[1] >= 12) {
											if (rob_pos[0] > 0)
												curr_zone = 3;
											else
												curr_zone = 5;
										}									
										else if (rob_pos[1] > 5.5) {																
											if (rob_pos[0] > 0) 
												curr_zone = 0;
											else
												curr_zone = 2;								
										}					
										else if (rob_pos[1] <= 5.5) {
											if (fabs(rob_pos[0]) < 1.5)
												curr_zone = 1;
											else if (rob_pos[0] > 0)
												curr_zone = 0;
											else
												curr_zone = 2;
										}
										printf("[[***]] Let's find my curr_zone = %d!\n", curr_zone);
									}
									if (curr_task >= 0 && curr_task < TASK1_2) {
										task1_1_checker = 0;
										wb_servo_set_position(tong_servo, 0);								
										wb_servo_set_position(tong_servo2, 0);
										hold_dist = 0;
										
										min_dist_block_indx = -1;
										tot_block = 0;							
										//printf("[[[Received block info: ");
										for (int i=0; i<MAX_TASK_BLOCK; i++) {
											if (receive_message[3*(i+2)+2] > -9999) {
												block_list[tot_block][0] = receive_message[3*(i+2)+2];//x
												block_list[tot_block][1] = receive_message[3*(i+2)+3];//z	
												block_list[tot_block][2] = 0;
												block_list[tot_block][3] = (double)((int)receive_message[3*(i+2)+4]%10);//indx
												block_list[tot_block][4] = (receive_message[3*(i+2)+4] - block_list[tot_block][3])/10.0;
												tot_block++;									
											}
										}							
//										printf("] Tot=Left=%d\n", tot_block);
										left_block = tot_block;																				
										hold_dist = 0;																															
											
										rob_state = STATE_GET_GARBAGE;	
										double r_val = drand();
										printf("(Grip prob_5) r_val=%.2f thre=%.2f\n", r_val, caripper_grip_qual);
										if (no_prob_flag == 1)
											r_val = 0;
										
										if (switch_grip_fail == 0) {
											if (r_val >= caripper_grip_qual) {
												grip_fail_flag = 1;								
												printf("[[[Fail block grip!!]]\n");
											}
											else
												grip_fail_flag = 0;																				
										}
										else {												
											
											if (grip_fail_flag == 1)
												grip_fail_flag = 0;
											else
												grip_fail_flag = 1;
										}

										
										beh_state = BEH_GO_POSITION;
										move_type = FORWARD;

										if (curr_task <= TASK1_ZONE1_3)
											target_zone = 0;
										else if (curr_task <= TASK1_ZONE2_3)
											target_zone = 1;
										else if (curr_task <= TASK1_ZONE3_3)
											target_zone = 2;
										
										//printf("Rob %d dont have any block! find closest block!\n", rob_int_name);										
										if (curr_zone == target_zone) {											
											strt_flag = 0;
											printf("\n\n(1) Start Recording dist & time of Task1_1!!\n");
											task1_1_checker = 1;										
											
											zone_arrival = 1;
											dist_margin = 0.1;
											angle_margin = 0.1;
											double min_dist = 10000;
											for (int i=0; i<tot_block; i++) {									
												block_list[i][2] = pow(rob_pos[0] - block_list[i][0], 2) + pow(rob_pos[1] - block_list[i][1], 2);
												if (min_dist > block_list[i][2]) {
													min_dist = block_list[i][2];
													min_dist_block_indx = i;
												}									
											}
//											printf("(3)Rob %d go to get block indx=%d\n", rob_int_name, (int)block_list[min_dist_block_indx][3]);
											block_list[min_dist_block_indx][2] = 10000;
											target[0] = block_list[min_dist_block_indx][0];
											target[1] = block_list[min_dist_block_indx][1];	
											target_block_indx = (int)block_list[min_dist_block_indx][4];
										}
										else {	
											curr_zone = get_zone(target, curr_zone, target_zone);		
											zone_arrival = 0;
											dist_margin = 1;
											angle_margin = 1;								
										}
										
										if (final_winner == rob_int_name) {
											curr_pre_task = curr_task;											
											printf("Shit!! I got Task1_1 & Task1_2! curr_prev_task=%d\n", curr_pre_task);
											send_message[send_cnt][0] = ASSIGN_MESSAGE;			
											send_message[send_cnt][1] = TASK1_2;			//task
											send_message[send_cnt][2] = (double)rob_int_name;			//auctioneer_id
											send_message[send_cnt][3] = -1;
											send_message[send_cnt][4] = ave_d2t2[0];
											send_message[send_cnt][5] = ave_d2t2[1];																
											send_message[send_cnt][6] = bid_weight;			
											send_message[send_cnt][14] = 7;
											send_cnt++;				
											bid_cnt = 0;
											printf("[3] D2T2=(%.2f / %.2f)\n", ave_d2t2[0], ave_d2t2[1]);

										}
										else {
											
											send_message[send_cnt][0] = ASSIGN_MESSAGE;			
											send_message[send_cnt][1] = TASK1_2;			//task
											send_message[send_cnt][2] = (double)rob_int_name;			//auctioneer_id
											send_message[send_cnt][3] = (double)final_winner;				//bidder_id
											send_message[send_cnt][4] = ave_d2t2[0];
											send_message[send_cnt][5] = ave_d2t2[1];
											send_message[send_cnt][6] = bid_weight;	
											send_message[send_cnt][7] = final_winner_utility;	
											send_message[send_cnt][14] = 8;											
											send_cnt++;
											my_follower = final_winner;
											
											printf("[2]Assign robot %d task %d (team formation)]\n", final_winner, auction_task);								
											printf("[4] D2T2=(%.2f / %.2f)\n", ave_d2t2[0], ave_d2t2[1]);
											bid_cnt = 0;
											auction_task = -1;
											auction_state = AUCTION_DISABLE;
										}
										
									}
									else if (curr_task == TASK1_2) {							
										rob_state = STATE_GET_MASTER;																			
										beh_state = BEH_STOP;																																					
									}					
									else if (curr_task == TASK3_1) {
										wb_servo_set_position(tong_servo, 0);								
										wb_servo_set_position(tong_servo2, 0);
										hold_dist = 0;
										grip_fail_flag = 0;

										coll_cnt = (int)receive_message[8];
										tray_block_cnt[0] = (int)receive_message[9];
										tray_block_cnt[1] = (int)receive_message[10];
										tray_block_cnt[2] = (int)receive_message[11];
										printf("[[%d blocks (%d, %d, %d)\n", coll_cnt, tray_block_cnt[0], tray_block_cnt[1], tray_block_cnt[2]);
										init_coll_cnt =  coll_cnt;
										if (rob_pos[1] > 13.5)
											sorting_step = -1;
										else {						
											sorting_step = -2;
											hold_dist = -MY_HALF_PI;
											wb_servo_set_position(tong_servo, hold_dist);								
											wb_servo_set_position(tong_servo2, -hold_dist);
										}
										
										
										rob_state = STATE_SORT;
										beh_state = BEH_GO_POSITION;					
										rob_vel[0] = 0;
										rob_vel[1] = 0;
										rob_vel[2] = 0;	
										
										if (sorting_step == -2) {
											if (curr_zone == 0 || curr_zone == 2) {
												double dist1 = pow(rob_pos[0] - station_node_left[0][0], 2) + pow(rob_pos[1] - station_node_left[0][1], 2);
												double dist2 = pow(rob_pos[0] - station_node_right[0][0], 2) + pow(rob_pos[1] - station_node_right[0][1], 2);
												if (dist1 < dist2) {
													target[0] = station_node_left[0][0];
													target[1] = station_node_left[0][1];
												}
												else {
													target[0] = station_node_right[0][0];
													target[1] = station_node_right[0][1];
												}
												sorting_step = -1;
											}								
											else {
												double dist1 = pow(rob_pos[0] - tray1_left[0][0], 2) + pow(rob_pos[1] - tray1_left[0][1], 2);
												double dist2 = pow(rob_pos[0] - tray1_right[0][0], 2) + pow(rob_pos[1] - tray1_right[0][1], 2);
												if (dist1 < dist2) {
													target[0] = tray1_left[0][0];
													target[1] = tray1_left[0][1];
												}
												else {
													target[0] = tray1_right[0][0];
													target[1] = tray1_right[0][1];
												}
												sorting_step = -2;
											}
											move_type = FORWARD;								
											dist_margin = 0.2;
											angle_margin = 0.2;
										}	
										else {
											target[0] = rob_pos[0];
											target[1] = rob_pos[1];
											target[2] = rob_pos[2];
											dist_margin = 5;
											angle_margin = 3;
										}
									}
									bid_state = BID_DISABLE;
								}
							}							
						}
						else {
							printf("Rob %d Rejected for task %d! winner is %d\n", rob_int_name, rcv_task, (int)receive_message[3]);																				
							if (rcv_task < TASK1_2) {
								send_message[send_cnt][0] = ASSIGN_MESSAGE;			
								send_message[send_cnt][1] = (double)TASK1_2;				//task
								send_message[send_cnt][2] = (double)rob_int_name;			//auctioneer_id
								send_message[send_cnt][3] = -1;						//bidder_id
								send_message[send_cnt][4] = ave_d2t2[0];
								send_message[send_cnt][5] = ave_d2t2[1];																
								send_message[send_cnt][6] = bid_weight;			
								send_message[send_cnt][14] = 7;
								send_cnt++;							
								bid_cnt = 0;
								auction_task = -1;
								auction_state = AUCTION_DISABLE;
								printf("[5] D2T2=(%.2f / %.2f)\n", ave_d2t2[0], ave_d2t2[1]);
							}
						}
																	
						task_cnt = find_shift(task_cnt, rcv_task, rcv_actioneer, receive_tasks);	
						if (test_mode >= 3 && test_mode <= 4 ) {
							bid_weight = update_bid_weight(bid_weight, receive_message[6]);
							printf("\nBid weight is %.2f\n", bid_weight);
						}
					}					
					else {
						task_cnt = find_shift(task_cnt, rcv_task, rcv_actioneer, receive_tasks);
						printf("I did not bid task %d yet, but it already assigned to other robot, so I remove it!\n", rcv_task);
					}
				}				
				if (task_cnt == 0)
					bid_state = BID_DISABLE;
			}
			else if (receive_message[0] == ARRIVE_MESSAGE && (int)receive_message[1] == rob_int_name) {
				wait_flag = 0;
				double rot_ang = atan2(receive_message[2]-rob_pos[0], receive_message[3]-rob_pos[1]);
				target[0] = rob_pos[0];
				target[1] = rob_pos[1];					
				target[2] = rot_ang;
				beh_state = BEH_LOAD_BLOCK; 
				move_type = ROTATION;	
				angle_margin = 0.05;
				load_obj_step = 0;
			}
			else if (receive_message[0] == COME_MESSAGE) {
				if ((int)receive_message[2] == rob_int_name && my_leader != -1) {
					come_cnt++;
					//printf("Leader asks me to come!\n");
					target_zone = (int)receive_message[5];					
					next_target[0] = receive_message[3];
					next_target[1] = receive_message[4];														
					if (rob_state != STATE_NEW_TASK) {					
//						printf("Okay I am coming\n");
						if (curr_zone != target_zone) {
							rob_state = STATE_GET_MASTER;
							beh_state = BEH_GO_POSITION;
							move_type = FORWARD;
							zone_arrival = 0;						
							curr_zone = get_zone(target, curr_zone, target_zone);						
							dist_margin = 1;
							angle_margin = 1;
						}
						else {						
							zone_arrival = 1;				
							if (rob_state != STATE_EMPTY_BIN) {
								rob_state = STATE_GET_MASTER;				
								beh_state = BEH_GO_POSITION;							
								move_type = FORWARD;															
								target[0] = next_target[0];
								target[1] = next_target[1];							
								dist_margin = 0.7;								
							}
							else {
								printf("\nSorry I can't go! I need to empty first!\n");
								send_message[send_cnt][0] = WAIT_MESSAGE;
								send_message[send_cnt][1] = (double)my_leader;
								send_message[send_cnt][14] = 2;
								send_cnt++;
							}
						}
					}
					else {
						come_flag = 1;
						printf("Sorry I need to empty first!\n");
					}
				}				
			}
			else if (receive_message[0] == WAIT_MESSAGE && (int)receive_message[1] == rob_int_name) {
				printf("Okay I will wait!\n");
				wait_flag = 1;
			}
			else if (receive_message[0] == LOAD_MESSAGE) {
				if ((int)receive_message[1] == rob_int_name && my_leader != -1) {
					arrive_flag = 0;
					beh_state = BEH_PICK_UP;
					rob_cap[0][CAP_GRIP] = 0;
					pick_up_step = 0;
					target[0] += 0.5 * sin(rob_pos[2]);
					target[1] += 0.5 * cos(rob_pos[2]);	
					if ((int)receive_message[2] == 1) {
						printf("[[Leader asked me to empty and complete Task1_2!!\n");
						empty_flag = 1;																		
						if ((int)receive_message[5] == 1) {
							rob_state = STATE_GET_MASTER;
							pick_up_step = 2;
							block_cnt--;
						}
					}
				}
			}
			else if (receive_message[0] == LOAD_DONE_MESSAGE && (int)receive_message[1] == rob_int_name) {
				if (left_block > 0) {
					double min_dist = 10000;
					for (int bc=0; bc<tot_block; bc++) {
						if (block_list[bc][2] != 10000)
							block_list[bc][2] = pow(rob_pos[0] - block_list[bc][0], 2) + pow(rob_pos[1] - block_list[bc][1], 2);
						if (min_dist > block_list[bc][2]) {
							min_dist = block_list[bc][2];
							min_dist_block_indx = bc;
						}
					}
//					printf("(4)Rob %d go to get block indx=%d\n", rob_int_name, (int)block_list[min_dist_block_indx][3]);
					block_list[min_dist_block_indx][2] = 10000;
					target[0] = block_list[min_dist_block_indx][0];
					target[1] = block_list[min_dist_block_indx][1];	
					target_block_indx = (int)block_list[min_dist_block_indx][4];
					rob_state = STATE_GET_GARBAGE;
					double r_val = drand();
					/*
					printf("(Grip prob_6) r_val=%.2f thre=%.2f\n", r_val, caripper_grip_qual);
					if (no_prob_flag == 1)
						r_val = 0;		
					*/

					/*
					if (r_val >= caripper_grip_qual) {
						grip_fail_flag = 1;								
						printf("[[[Fail block grip!!]]\n");
					}
					else
						grip_fail_flag = 0;	
					*/
				}
				else {
					printf("[[ Rob %d Complete Task1_1 (=%d)!! ]]\n", rob_int_name, curr_task);
					if (curr_task != -1) {
						prev_task = curr_task;
						curr_task = -1;	
						strt_flag = 1;
						printf("\n (6)Compete! strt_flag = 1\n");
					}
				}
				beh_state = BEH_GO_POSITION;
				move_type = FORWARD;		
			}
			else if (receive_message[0] == ABANDON_MESSAGE) {
				if (curr_task <= TASK1_ZONE3_3 && (int)receive_message[1] == TASK1_2 && (int)receive_message[2] == my_follower) {
					my_follower = -1;
					if (rob_state == STATE_GET_SLAVE) {
						rob_state = STATE_GET_GARBAGE;
						double r_val = drand();
						printf("(Grip prob_7) r_val=%.2f thre=%.2f\n", r_val, caripper_grip_qual);
						if (no_prob_flag == 1)
							r_val = 0;	
						
						if (switch_grip_fail == 0) {
							if (r_val >= caripper_grip_qual) {
								grip_fail_flag = 1;								
								printf("[[[Fail block grip!!]]\n");
							}
							else
								grip_fail_flag = 0;																				
						}
						else {												
							
							if (grip_fail_flag == 1)
								grip_fail_flag = 0;
							else
								grip_fail_flag = 1;
						}

						
						beh_state = BEH_GO_POSITION;
						printf("My follower abandoned TASK1_2!!\n");
						wait_flag = 0;
					}
				}				
				else if (curr_task == TASK3_1 && (int)receive_message[1] == TASK3_2 && (int)receive_message[2] == my_follower) {
					printf("My follower %d gave up during TASK3_1!\n", my_follower);
					my_follower = -1;
				}
				else if ((int)receive_message[2] == my_leader) {					
					send_message[send_cnt][0] = ABANDON_MESSAGE;
					send_message[send_cnt][1] = (double)curr_task;
					send_message[send_cnt][2] = (double)rob_int_name;
					send_message[send_cnt][14] = 3;
					send_cnt++;
					my_follower = -1;
					my_leader = -1;

					printf("My leader abandoned task!! I abandon my task too\n");	
					if (rob_state == STATE_GET_MASTER && beh_state == BEH_STOP) {
						printf("Hull(4)! Leader left! I also give up task!\n");
						my_follower = -1;
						if (curr_task != -1) {
							prev_task = curr_task;
							curr_task = -1;
							strt_flag = 1;
							printf("\n (7)Compete! strt_flag = 1\n");
						}
						if (block_cnt > 2)
							rob_cap[0][CAP_GRIP] = 0;
						else
							rob_cap[0][CAP_GRIP] = caripper_grip_qual;

						rob_state = STATE_ZONE_STAY;
						wb_servo_set_position(tong_servo, -MY_HALF_PI);								
						wb_servo_set_position(tong_servo2, MY_HALF_PI);
						move_type = FORWARD;						
						
						if (curr_zone >=0 && curr_zone <= 2) {										
							beh_state = BEH_GO_POSITION;										
							int tmp_stay_indx = 8*curr_zone + (8 - rob_num + rob_int_name - 1);
							if (curr_zone == 1)
								tmp_stay_indx = 8*curr_zone + (rob_int_name - 1);
							target[0] = zone_stay[tmp_stay_indx][0];
							target[1] = zone_stay[tmp_stay_indx][1];
							target[2] = 0;

							dist_margin = 0.1;
							angle_margin = 3;
						}
						else
							beh_state = BEH_STOP;
					}
				}
			}			
			else if (receive_message[0] == TRAY_BACK_MESSAGE) {
				tray_full_flag = 0;
				wait_flag = 0;
				printf("[[Tray1 is back!!\n");
				if (beh_state == BEH_STORE_OBJ) {
					move_type = FORWARD;
					target[0] = tray1_zone[curr_zone][0];
					target[1] = tray1_zone[curr_zone][1];
					target[2] = tray1_zone[curr_zone][2];
					
					double r_val = drand();
					if (no_prob_flag == 1)
						r_val = 0;
					printf("(3)r_val=%.2f / thres=%.2f\n", r_val, rob_cap[0][CAP_LOC] - cap_loc_dec);
					if (r_val >= rob_cap[0][CAP_LOC] - cap_loc_dec) {
						r_val -= (rob_cap[0][CAP_LOC] - cap_loc_dec);
						if (curr_zone == 0)
							target[0] += r_val;
						else if (curr_zone == 1)
							target[1] -= r_val;
						else if (curr_zone == 2)
							target[0] -= r_val;
					}					
					dist_margin = 0.1;
					angle_margin = 0.1;
				}
			}
			else if (receive_message[0] == BLOCK_IS_MESSAGE && (int)receive_message[1] == rob_int_name) {
				block_category = (int)receive_message[2];
				if (no_prob_flag == 0)
					block_category = get_block_category(block_category, rob_cap[0][CAP_RECOG]);											
				sorting_step = 4;
				printf("Super says the block is %d (real=%d)\n", block_category, (int)receive_message[2]);
			}
			else if (receive_message[0] == PULL_DONE_MESSAGE && curr_task == TASK3_1) {
				int tmp_tray = (int)receive_message[1];
				int tmp_block_num = (int)receive_message[2];
				tray_block_cnt[tmp_tray] = tmp_block_num;
				printf("]]Tray %d has %d blocks!\n", tmp_tray+2, tmp_block_num);								
				if (tmp_block_num >= max_tray_block) {
					send_message[send_cnt][0] = PULL_MESSAGE;
					send_message[send_cnt][1] = (double)rob_int_name;
					send_message[send_cnt][2] = (double)my_follower;
					send_message[send_cnt][3] = (double)tmp_tray;										
					send_message[send_cnt][14] = 4;
					send_cnt++;										
					printf("Carrier (rob %d)! Pull tray %d\n", my_follower, tmp_tray+2);					
				}
				else {
					tray_block_req[tmp_tray] = 0;		
					if (sorting_step == -20 && block_category == tmp_tray) {
						sorting_step = 4;
						printf("Very well! go back to sorting!!\n");
					}
				}
			}			
			else if (receive_message[0] == MOVE_TRAY_MESSAGE && (curr_task == TASK3_1 || prev_task == TASK3_1)) {
				int tmp_tray = (int)receive_message[1];
//				printf("My follower carry tray %d\n", tmp_tray+2);
				last_tray = tmp_tray;
			}		
			else if (receive_message[0] == BLOCK_NUM_MESSAGE && curr_task == TASK3_1) {
				int tmp_tray = (int)receive_message[1];
				int tmp_block_num = (int)receive_message[2];
				tray_block_cnt[tmp_tray] = tmp_block_num;
				printf("[[2]]Tray %d has %d blocks!\n", tmp_tray+2, tmp_block_num);					
				if (tray_block_cnt[0] >= max_tray_block || tray_block_cnt[1] >= max_tray_block || tray_block_cnt[2] >= max_tray_block) {
					if (my_follower != -1)
						sorting_step = 6;									
					else {
						
						printf("Now I need a follower to move tray!\n");
						sorting_step = -11;																				
						auction_task = TASK3_2;										
						auction_data_size = 0;
						auction_state = AUCTION_ENABLE;										
					}									
				}
				else
					sorting_step = 6;
			}			
		}
		if (bid_state == BID_WAIT_ACCEPT && (curr_task == TASK1_2 || curr_task == TASK3_1))
			wb_differential_wheels_set_speed(0, 0);

	
		if (send_cnt > 0) {			
			wb_emitter_send(emitter, send_message[0], (int)(send_message[0][14])*sizeof(double));
			send_cnt--;
			for (int s=0; s<send_cnt; s++) {
				for (int s2=0; s2< send_message[s+1][14]; s2++)
					send_message[s][s2] = send_message[s+1][s2];
				send_message[s][14] = send_message[s+1][14];
			}			
		}			
		
		
		dist_measure += (fabs(rob_vel[0]) * time_sec);
		time_measure += time_sec;
		if ((rob_state == STATE_GET_SLAVE && wait_flag == 1 && tray_full_flag == 1) || (beh_state == BEH_STORE_OBJ && tray_full_flag == 1)) {
			dist_measure -= (fabs(rob_vel[0]) * time_sec);
			time_measure -= time_sec;
			/*
			if (rob_state == STATE_GET_SLAVE && wait_flag == 1 && tray_full_flag == 1)
				printf(" I wait follower!!\n");
			else
				printf(" I wait tray1 back!!\n");
			*/
		}		
								
		if (strt_flag == 0) {
			dist_measure = 0;
			time_measure = 0;
			strt_flag = -1;
			printf("\nRecord starting point\n\n");
		}
		else if (strt_flag == 1) {	
			int set_cnt = 0;
			if (prev_task < TASK1_2)
				set_cnt = tot_block;	
			else if (prev_task == TASK1_2)
				set_cnt = come_cnt;
			else if (prev_task == TASK3_1)
				set_cnt = init_coll_cnt;				
			else
				set_cnt = 1;
			
			int t_indx = get_task_indx(prev_task);
			update_cnt[t_indx]++;
			printf("\nReal update of task %d (cnt=%d)\n", prev_task, update_cnt[t_indx]);			
			update_cost(prev_task, dist_measure, time_measure, task_dist, task_time, set_cnt, update_cnt[t_indx]);
			strt_flag = -1;			
		}

		time += time_sec;
		int sec = (int) time % 60;//0~59
		int min = (int) (time / 60);
		time_cnt = (int) time;

		if (test_mode != -1) {		
			double tmp_eng = get_energy_consumption(curr_task, rob_cap, task_req0, task_req1, task_req2, task_req3, rob_vel[0], time_sec);
			curr_energy -= tmp_eng;
			energy_consumption += tmp_eng;
			E = (curr_energy - min_energy)/(max_energy - min_energy);			
			if (prev_sec != sec) {
				prev_sec = sec;
				if (test_mode >= 3 && test_mode <= 4 ) {
					bid_weight = update_bid_weight(bid_weight, W_measure);		
					W_measure = scalar * (exp(s_factor*(pow(G[time_cnt], sqare_num) - 1.0)) - bias_bid_weight);
					double v_rand = 0.5*drand();
					if (rand()%2 == 0)
						W_measure += v_rand;
					else
						W_measure -= v_rand;
				}
				int t_indx = get_task_indx(curr_task);							
				if (time_cnt == store_time[t_cnt]) {
					if (test_mode >= 3 && test_mode <= 4 ) {
						bid_weight = update_bid_weight(bid_weight, W_measure);		
						if (test_mode == 4) {
							w_data[t_cnt] = bid_weight;
							v_data[t_cnt] = G[time_cnt];
						}
					}
					energy_data[t_cnt] = curr_energy;					
					t_cnt++;
				}				
			}
		}
		if (real_termination_time > 0 && time_cnt >= real_termination_time) {
			if (full_record == 1) {
				char f_name[30];							
				if (test_mode != 5 && test_mode != 6) {					
					sprintf(f_name, "..\\energy_%d_%d_%d.txt", rob_int_name, rob_num, test_mode);
					energy_file = fopen(f_name, "a+");					
					for (int t=0; t<11; t++) 
						fprintf(energy_file, "%f\n", energy_data[t]);								
					fclose(energy_file);
					if (test_mode == 4) {
						sprintf(f_name, "..\\w_%d_%d_%d.txt", rob_int_name, rob_num, test_mode);
						w_file = fopen(f_name, "a+");																		
						sprintf(f_name, "..\\v_%d_%d_%d.txt", rob_int_name, rob_num, test_mode);
						v_file = fopen(f_name, "a+");																		
						for (t=0; t<11; t++) {
							fprintf(w_file, "%f\n", w_data[t]);	
							fprintf(v_file, "%f\n", v_data[t]);
						}
						fclose(w_file);						
						fclose(v_file);						
					}
				}
				else {
					sprintf(f_name, "..\\eng_%d_%d.txt", rob_int_name, rob_num);					
					fin_eng_file = fopen(f_name, "a+");
					fprintf(fin_eng_file, "%f\n", curr_energy);
					fclose(fin_eng_file);
				}				
			}
			wb_differential_wheels_set_speed(0, 0);
			break;
		}

		wb_receiver_next_packet(receiver);		
		wb_robot_step(time_step);				
	}
	wb_robot_cleanup();
	return 0;
}




