#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define EVENT_PERIOD		300 
#define MAX_TERMINATION_TIME	3001
#define NUM_ROBOT			8	//4,5,6,7,8

enum {STATE_INIT, STATE_READY, STATE_AUCTION, STATE_WAIT_BIDDER, STATE_GET_BIDDER, STATE_REMOVE_TASK, STATE_ASSIGN, STATE_NO_AUCTION};	
enum {TYPE_GARBAGE, TYPE_RECYCLE, TYPE_STONE};
enum {TASK1_ZONE1_1, TASK1_ZONE1_2, TASK1_ZONE1_3, TASK1_ZONE2_1, TASK1_ZONE2_2, TASK1_ZONE2_3, 
      TASK1_ZONE3_1, TASK1_ZONE3_2, TASK1_ZONE3_3, TASK1_2, TASK2_1, TASK2_2, TASK3_1, TASK3_2, TASK4_1, TASK4_2};

#define AUCTION_WAIT_TIME 100


#define NUM_BLOCK		60
#define MAX_TASK_BLOCK	7
#define NUM_EACH_BLOCK  NUM_BLOCK/3 
#define NUM_ZONE_TASK   NUM_EACH_BLOCK - 1	
											



#define TEAM_FORM_AUCTION 0
#define INDI_AUCTION	  1



//통신 header
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

static WbDeviceTag emitter;
static WbDeviceTag receiver;
static int time_step = 0;
static double time_sec = 0;

static void initialize()
{    
	wb_robot_init();  
	time_step = (int)wb_robot_get_basic_time_step();  
	time_sec = (double) time_step / 1000;
	emitter = wb_robot_get_device("emitter");
	receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, time_step);
}

double find_max(double a, double b) { if (a >= b) return a; else return b; }
double find_min(double a, double b) { if (a < b)  return a; else return b; }

int find_shift(int cnt, int target, int* people)
{
	int tmp_sft = 0;
	for (int i=0; i<cnt; i++) {
		if (people[i] == target) {
			//printf("Remove %d\n", target);
			tmp_sft = 1;
		}
		if (i<cnt-1)
			people[i] = people[i+tmp_sft];
	}
	if (tmp_sft == 1)
		cnt--;
	return cnt;
}

int find_shift(int cnt, int target, int target2, int target3, int* people, int* people2, int* people3)
{
	int tmp_sft = 0;
	for (int i=0; i<cnt; i++) {
		if (people[i] == target && people2[i] == target2 && people3[i] == target3) {
			//printf("Remove %d\n", target);
			tmp_sft = 1;
		}
		if (i<cnt-1) {
			people[i] = people[i+tmp_sft];
			people2[i] = people2[i+tmp_sft];
			people3[i] = people3[i+tmp_sft];
		}
	}
	if (tmp_sft == 1)
		cnt--;
	return cnt;
}


void bid_bubble_sort(int tmp_bid_cnt, double (*tmp_bid_list)[4], int* indx_sort)
{
	double val_exchange = 0;
	int idx_exchange = 0;		
	double val_bid_list[NUM_ROBOT];

	for (int i=0; i<tmp_bid_cnt; i++) {	
		indx_sort[i] = i;		//index
		val_bid_list[i] = tmp_bid_list[i][2];	//value
		//printf("robot %d cost %d\n", result_sort[i], val_bid_list[i]);		
	}
	for (i=0; i<tmp_bid_cnt; i++) {	
		for (int j=1; j<tmp_bid_cnt; j++) {				
			if (val_bid_list[j-1] > val_bid_list[j]) {
				val_exchange = val_bid_list[j-1];
				val_bid_list[j-1] = val_bid_list[j];
				val_bid_list[j] = val_exchange;

				idx_exchange = indx_sort[j-1];
				indx_sort[j-1] = indx_sort[j];
				indx_sort[j] = idx_exchange;

				//printf("Exchange %d and %d\n", j-1, j);
			}
		}
	}	
}

double get_distance(double* p1, double* p2, int sq_flag)
{
	double dist = 0;
	double distX = (p1[0]-p2[0]) * (p1[0]-p2[0]);
	double distY = (p1[1]-p2[1]) * (p1[1]-p2[1]);
	if (sq_flag == 1)
		dist = sqrt(distX + distY);
	else
		dist = distX + distY;
	return dist;
}

void get_char_num(int id, char* tmp_name)
{	
	char tmp[1];
	if ((id + 1) >= 10) {				
		if ((id + 1) < 20) {
			tmp_name[1] = '1';
			itoa(id - 9, tmp, 10);			
		}
		else if ((id + 1) < 30) {
			tmp_name[1] = '2';			
			itoa(id - 19, tmp, 10);
		}
		else if ((id + 1) < 40) {
			tmp_name[1] = '3';
			itoa(id - 29, tmp, 10);
		}
		else if ((id + 1) < 50) {
			tmp_name[1] = '4';
			itoa(id - 39, tmp, 10);
		}
		else if ((id + 1) < 60) {
			tmp_name[1] = '5';
			itoa(id - 49, tmp, 10);
		}
		else {
			tmp_name[1] = '6';			
			itoa(id - 59, tmp, 10);
		}
		tmp_name[2] = tmp[0];
		if ((id + 1)%10 == 0) {
			tmp_name[2] = '0';
			//tmp_name[3] = '\0';
		}							
		tmp_name[3] = '\0';
	}
	else {
		itoa(id + 1, tmp, 10);
		tmp_name[1] = tmp[0];
		tmp_name[2] = '\0';
	}
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

int main()
{
	initialize();   
	srand( (unsigned)time(NULL) );	
	int  supervisor_state = STATE_INIT;
	int activate_auctioneer = 1;
	if (activate_auctioneer == 0)
		supervisor_state = STATE_NO_AUCTION;
	WbNodeRef block[NUM_BLOCK];	
	const double* b_tran;
	WbFieldRef block_tran[NUM_BLOCK];	
	double block_pos[NUM_BLOCK][3];	
	/*
	int zone_task[NUM_ZONE_TASK][4] = {
		{9, 16, 22, -1},//zone 1		
		{5, 18, 27, -1},//zone 1		
		{3, 13, 24, 17},//zone 1		
		{6, 10, 26, -1},//zone 2
		{1, 15, 28, -1},//zone 2
		{4, 11, 19, 21},//zone 2
		{7, 14, 25, -1},//zone 3
		{8, 20, 23, -1},//zone 3
		{2, 12, 29, 30} //zone 3
	};
	*/
	int zone_task[NUM_ZONE_TASK][7] = {
		{9, 16, 22, 31, 40, 49, -1},//zone 1		
		{5, 18, 27, 32, 41, 50, 51},//zone 1		
		{3, 13, 24, 17, 33, 42, 58},//zone 1		
		{6, 10, 26, 34, 43, 52, -1},//zone 2
		{1, 15, 28, 35, 44, 53, 54},//zone 2
		{4, 11, 19, 21, 36, 45, 59},//zone 2
		{7, 14, 25, 37, 46, 55, -1},//zone 3
		{8, 20, 23, 38, 47, 56, 57},//zone 3
		{2, 12, 29, 30, 39, 48, 60} //zone 3
	};

	double tray_area[3][4] = {{0.45, 1.12 ,18.7, 19.3}, {-0.35, 0.35, 19.5, 20.2}, {-1.12, -0.45, 18.7, 19.3}};
	
	double rob_init_pos[NUM_ROBOT][3] = {{8, 0, -0.5}, {6, 0, -0.5}, {1, 0, -4}, {-1, 0, -4}, {-6, 0, -0.5}, {-8, 0, -0.5}};//일단 6대. 12대까지 해야 됨.
	
	int left_zone_task[NUM_ZONE_TASK];
	int unallo_task[NUM_ZONE_TASK+1];
	int zone_task_cnt = 0;
	for (int i=0; i<NUM_ZONE_TASK+1; i++) {
		//left_zone_task[i] = i;
		unallo_task[i] = -1;
	}
	int assigned_task[NUM_ZONE_TASK+1];
	int assigned_cnt = 0;
	int allo_cnt = 0;
	int allo_task[NUM_ZONE_TASK];
	
	const char garbage_name[] = "G0";	
	char tmp_name[5];

	int remove_block[NUM_BLOCK];
	int remove_block_cnt = 0;
	for (i=0; i<3; i++) {	
		sprintf(tmp_name, "%s", garbage_name);		
		for (int j=0; j<NUM_EACH_BLOCK/2; j++) {
			int indx = i*NUM_EACH_BLOCK/2 + j;
			for (int k=0; k<2; k++) {
				int indx2 = indx + k*30;
				get_char_num(indx2, tmp_name);			
				block[indx2] = wb_supervisor_node_get_from_def(tmp_name);					
				block_tran[indx2] = wb_supervisor_node_get_field(block[indx2], "translation");	
				remove_block[indx2] = -1;
				b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[indx2]);
				block_pos[indx2][0] = b_tran[0];				
				block_pos[indx2][1] = -4999.95;
				block_pos[indx2][2] = b_tran[2];
				//wb_supervisor_field_set_sf_vec3f(block_tran[indx2], block_pos[indx2]);
			}
		}		
	}

	WbNodeRef rob[NUM_ROBOT];
	WbFieldRef rob_field_tran;
	WbFieldRef rob_field_rot;	
	double rob_rot[4] = {0, 1, 0, 3.1415};

	int tmp_rob_num = NUM_ROBOT;
	int tmp_rob_indx[NUM_ROBOT];
	for (i=0; i<NUM_ROBOT; i++)
		tmp_rob_indx[i] = i;
	

	WbNodeRef tray1 = wb_supervisor_node_get_from_def("T1");
	WbFieldRef tray1_field_rot = wb_supervisor_node_get_field(tray1, "rotation");	
	WbFieldRef tray1_field_tran = wb_supervisor_node_get_field(tray1, "translation");	
	const double* t_tran = wb_supervisor_field_get_sf_vec3f(tray1_field_tran);
	double tray_rot[4][4] = {{0, 1, 0, 0}, {0, 1, 0, -3.1415}, {0, 1, 0, 1.5708}, {0, 1, 0, 0}};		
	double tray1_init_pos[3] = {0, 0, 7};//x,y,z
	double tray1_fin_pos[3] = {0, 0, 14.16};//x,y,z
	wb_supervisor_field_set_sf_rotation(tray1_field_rot, tray_rot[0]);
	wb_supervisor_field_set_sf_vec3f(tray1_field_tran, tray1_init_pos);

	WbNodeRef gate = wb_supervisor_node_get_from_def("Gate");
	WbFieldRef gate_field_tran = wb_supervisor_node_get_field(gate, "translation");		
	double gate_pos[3] = {0, 0.2, 13.6};		
	wb_supervisor_field_set_sf_vec3f(gate_field_tran, gate_pos);
	
	WbNodeRef small_tray[3];
	small_tray[0] = wb_supervisor_node_get_from_def("T2");	
	small_tray[1] = wb_supervisor_node_get_from_def("T3");
	small_tray[2] = wb_supervisor_node_get_from_def("T4");
	WbFieldRef small_tray_field_rot[3];	
	WbFieldRef small_tray_field_tran[3];	
	const double* small_t_tran[3];
	for (i=0; i<3; i++) {
		small_tray_field_tran[i]= wb_supervisor_node_get_field(small_tray[i], "translation");	
		small_tray_field_rot[i] = wb_supervisor_node_get_field(small_tray[i], "rotation");	
		small_t_tran[i] = wb_supervisor_field_get_sf_vec3f(small_tray_field_tran[i]);
	}
	double small_tray_pos[3][3] = {{0.8, 0, 19}, {0, 0, 19.82}, {-0.8, 0, 19}};
	


	double rob_left_pos[3] = {0.7, 0, 13.94};
	double rob_right_pos[3] = {-0.7, 0, 13.94};
	double rob_ang[4] = {0, 1, 0, 0};
	
	
	double out_block_pos[31][2] = { {0, 17.8}, {0, 17.2}, {0, 16.6},
	{0, 16.2}, {0.2, 16.2}, {-0.2, 16.2}, {0.4, 16.2}, {-0.4, 16.2}, {0.6, 16.2}, {-0.6, 16.2},
	{0, 15.9}, {0.2, 15.9}, {-0.2, 15.9}, {0.4, 15.9}, {-0.4, 15.9}, {0.6, 15.9}, {-0.6, 15.9},
	{0, 15.6}, {0.2, 15.6}, {-0.2, 15.6}, {0.4, 15.6}, {-0.4, 15.6}, {0.6, 15.6}, {-0.6, 15.6},		
	{0, 15.3}, {0.2, 15.3}, {-0.2, 15.3}, {0.4, 15.3}, {-0.4, 15.3}, {0.6, 15.3}, {-0.6, 15.3}};

	double room1_3_block_pos[35][2] = {{0, 0}, {-0.2, 0}, {-0.4, 0}, {-0.6, 0}, {-0.8, 0}, {-1, 0}, {-1.2, 0},
	{0, -0.2}, {-0.2, -0.2}, {-0.4, -0.2}, {-0.6, -0.2}, {-0.8, -0.2}, {-1, -0.2}, {-1.2, -0.2},
	{0, -0.4}, {-0.2, -0.4}, {-0.4, -0.4}, {-0.6, -0.4}, {-0.8, -0.4}, {-1, -0.4}, {-1.2, -0.4},
	{0, -0.8}, {-0.2, -0.8}, {-0.4, -0.8}, {-0.6, -0.8}, {-0.8, -0.8}, {-1, -0.8}, {-1.2, -0.8},
	{0, -1}, {-0.2, -1}, {-0.4, -1}, {-0.6, -1}, {-0.8, -1}, {-1, -1}, {-1.2, -1}};
	
	
	
	double room2_block_pos[35][2] = {{1.2, 24}, {1.2, 24.2}, {1.2, 24.4}, {1.2, 24.6}, {1.2, 24.8}, {1.2, 25}, {1.2, 25.2},
	{1.4, 24}, {1.4, 24.2}, {1.4, 24.4}, {1.4, 24.6}, {1.4, 24.8}, {1.4, 25}, {1.4, 25.2},
	{1.6, 24}, {1.6, 24.2}, {1.6, 24.4}, {1.6, 24.6}, {1.6, 24.8}, {1.6, 25}, {1.6, 25.2},
	{1.8, 24}, {1.8, 24.2}, {1.8, 24.4}, {1.8, 24.6}, {1.8, 24.8}, {1.8, 25}, {1.8, 25.2},
	{2, 24}, {2, 24.2}, {2, 24.4}, {2, 24.6}, {2, 24.8}, {2, 25}, {2, 25.2}};

	int bid_cnt = 0;	
	int prev_bid_cnt = 0;
	double receive_bidders[NUM_ROBOT][9];//receive_bidders: (0)task / (1)auctioneer_id / (2)bidder_id / (3)quality / (4)cost / (5)d2 / (6)t2 / (7)bid_weight / 
	int winner_cnt = 0;
	double final_winners[NUM_ZONE_TASK][29];//(0)task / (1)auctioneer_id / (2)bidder_id / (3,4,5)x1,z1 / (6,7,8)x2,z2 / (9,10,11)x3,z3 /(12,13,14)x4,z4
	int curr_assign_cnt = 0;

	double time = 0;
	int period_cnt = 0;
	int time_cnt = 0;
	int prev_time_cnt = time_cnt;
	int prev_time_cnt2 = time_cnt;


	
	
	int unallo_cnt = 0;			
	int broadcast_cnt = 0;	
	int wait_cnt = 0;	
	
	//int num_left_zone = NUM_ZONE_TASK;
	int left_zone_cnt = 0;

	int rob_int_name = 9;
	int init_cnt = 0;
	int left_zone[3] = {0, 1, 2};
	
	int zone_mask[3] = {1, 1, 1};	
	//int zone_mask[3] = {1, 0, 0};	
	int task_prob[3][3] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};		
	//int task_prob[3][3] = {{0, 1, 0}, {0, 1, 0}, {1, 1, 1}};
	int tot_block = 0;

	int max_tray1_block = 9;//10;	
	int max_small_tray_block = 3;
	
	int store_block[NUM_BLOCK];
	int tmp_store_block[NUM_BLOCK];
	int complete_block[12]; 
	int complete_cnt = 0;
	int coll_cnt = 0;
	int tmp_coll_cnt = 0;
	int lift_flag = 0;
	int sorter_ID = -1;
		

	int sorter_flag = 0;
	int send_cnt = 0;	
	double send_message[10][30];
	int tray_block_cnt[3] = {0, 0, 0};
	

	int err_block_cnt = 0;
	int tot_block_cnt[3] = {0, 0, 0}; 

	period_cnt = EVENT_PERIOD;


	
	int task_seq[7] = {0, 1, 4, 8};//0~8	
	int test_max = 1;
	int test_cnt = 0;	
	
	

	int set_pos_flag = 0;
	int bidder_ID[NUM_ZONE_TASK][NUM_ROBOT];
	int room_cnt[3] = {0, 0, 0};	
	int lost_block_cnt[2] = {0, 0};
	int tmp_coll_cnt2 = 0;	
	int block_category = 0;

	
	int grip_cnt = 0;
	int grip_fail_cnt = 0;
	double grip_scale = 0.15;
	double grip_success_rate = 0.8;

	
	int recog_cnt = 0;
	int recog_fail_cnt = 0;
	double recog_scale = 0.3;
	double recog_success_rate = 0.8;

	
	int carry_cnt = 0;
	int carry_fail_cnt = 0;
	double carry_scale = 0.3;
	double carry_success_rate = 0.8;

	int carry_auctioner = -1;
	
	
	int no_prob_flag = 0;

	double ave_d2t2[2] = {0, 0};


	int rob_num = 8;//4,5,6,7,8		
	int test_mode = 0;//[0:QBTA, 1:CBTA, 2:YBTA, 3:QCBTA, 4:QCYBTA], [5:quality_weight test], [6:subsidy_weight test]
	int full_record = 1;
	int num_sim_iteration = 20;
	int real_termination_time = 3000 - 1; 



	double testing_bid_weight = 0;
	double testing_s_weight = 0;  
	
	FILE* sim_cnt_file;
	int sim_cnt = 0;
	sim_cnt_file = fopen("val.txt", "r");		
	fscanf(sim_cnt_file, "%d", &sim_cnt);
	fclose(sim_cnt_file);	
	
	if (test_mode == 5) {				
		int proc1 = (int)sim_cnt/num_sim_iteration;
		double proc2 = 2.5 * proc1;
		testing_bid_weight = proc2*0.10;
		testing_s_weight = 0;
		printf("Bid weight=%f\n", testing_bid_weight);
		num_sim_iteration *= 5;
	}
	else if (test_mode == 6) {		
		int proc1 = (int)sim_cnt/num_sim_iteration;
		double proc2 = 2.5 * proc1;
		testing_bid_weight = 0.5;
		testing_s_weight = proc2*0.10;				
		printf("S weight=%f\n", testing_s_weight);
		num_sim_iteration *= 5;
	}
	

	char f_name[30];
	FILE* err_file;	
	FILE* w_file;
	FILE* v_file;
	FILE* fin_err_file;
	
	int store_time[11];
	int time_gap = (int)floor((double)real_termination_time/(double)10);
	for (int n=0; n<11; n++)
		store_time[n] = time_gap * n + 1;
	double w_data[11];
	double v_data[11];
	double err_data[11][3];
	int t_cnt = 0;
	
	double G[MAX_TERMINATION_TIME];
	double T1 = 1200;
	int tra = 800;
	for (int x=0; x<real_termination_time; x++) {	
		if (x < tra)
			G[x] = 0.5 - 0.5*cos(0.7*2*3.1415*(double)x/T1);		
		else
			G[x] = G[tra-1] - 0.00005 * (x - tra);	
		G[x] *= (1 + (drand()-0.5)*0.05);
	}	


	double W_min = 0.1;
	double W_max = 0.9;
	double sqare_num = 0.5;
	double bias_bid_weight = 0.5; // mu (0 <= bias_bid_weight < 1)
	double s_factor = 0.5878;// = log(Wmax/((1-bias_bid_weight)*Wmin + bias_bid_weight*Wmax))
	double W_measure = 0;
	double scalar = (W_max/(1-bias_bid_weight));
	
	double bid_weight = 0;
	double s_weight = 0;
	double s_weight_max = 0.9;
	double std_max = 450;

	if (test_mode == 0) {//qbta
		bid_weight = 1;
		s_weight = 0;
	}
	else if (test_mode == 1) {//cbta
		bid_weight = 0;				
		s_weight = 0;
	}
	else if (test_mode == 2) {//ebta
		bid_weight = 0;
		s_weight = 1;
	}
	else if (test_mode >= 3 && test_mode <= 4)
		bid_weight = scalar * (exp(s_factor*(pow(G[0], sqare_num) - 1.0)) - bias_bid_weight);
	else if (test_mode == 5) {
		bid_weight = testing_bid_weight;
		s_weight = 0;
	}
	else if (test_mode == 6) {
		bid_weight = testing_bid_weight;
		s_weight = testing_s_weight;
	}

	int curr_task = -1;

	while(1) {   
		if (supervisor_state == STATE_INIT) {						
			init_cnt++;
			if (init_cnt == 1)
				printf("Initialize Supervisor!!\n");
			else if (init_cnt > 10) {				
				printf("OKAY\n");	
				supervisor_state = STATE_AUCTION;
				send_message[send_cnt][0] = INIT_MESSAGE;
				send_message[send_cnt][1] = (double)rob_num;
				send_message[send_cnt][2] = (double)full_record;
				send_message[send_cnt][3] = (double)test_mode;
				send_message[send_cnt][4] = (double)real_termination_time;				
				send_message[send_cnt][29] = 5;
				if (test_mode == 5) {
					send_message[send_cnt][5] = bid_weight;
					send_message[send_cnt][29] = 6;
				}
				else if (test_mode == 6) {
					send_message[send_cnt][5] = s_weight;
					send_message[send_cnt][6] = bid_weight;
					send_message[send_cnt][29] = 7;
				}
				send_cnt++;
			}				
		}		
		else {
			time_cnt = (int) time;	
						
			if (zone_task_cnt < NUM_ZONE_TASK && time_cnt != prev_time_cnt) {
				prev_time_cnt = time_cnt;
				if (period_cnt >= EVENT_PERIOD) {		
					left_zone_cnt = 0;
					for (int i=0; i<3; i++) {//행 												
						int chk = 0;
						for (int j=0; j<3; j++) {
							chk = task_prob[i][j] * zone_mask[i];
							if (chk == 1) {
								left_zone_task[left_zone_cnt] = 3*i + j;
								left_zone_cnt++;
							}
						}					
					}
					if (left_zone_cnt > 0) {
						int rand_zone_task = irand(left_zone_cnt);
						int sel_zone_task = left_zone_task[rand_zone_task];

						if (test_max > 0) {							
							if (test_cnt < test_max)
								sel_zone_task = task_seq[test_cnt];
							test_cnt++;							
						}			
		
						int sel_zone = (int)floor(sel_zone_task/3.0);
						int sel_ztask = sel_zone_task%3;
												

						
						task_prob[sel_zone][sel_ztask] = 0;
						zone_mask[sel_zone] = 0;
						zone_mask[sel_zone] = 0;
						zone_mask[sel_zone] = 0;
						for (i=0; i<MAX_TASK_BLOCK; i++) {
							int tmp_indx = zone_task[sel_zone_task][i];
							if (tmp_indx != -1) {
								tmp_indx--;
								block_pos[tmp_indx][1] = 0.05;
								wb_supervisor_field_set_sf_vec3f(block_tran[tmp_indx], block_pos[tmp_indx]);
							}										
						}
						unallo_task[unallo_cnt] = sel_zone_task;					
						unallo_cnt++;			
						period_cnt = 0;
						printf("@@@@@ Task=%d\n", sel_zone_task);
					}
				}
				else
					period_cnt++;  
			}
		}

		int rb_cnt = 0;
		for (int rb = 0; rb<remove_block_cnt; rb++) {			
			int tmp_indx = remove_block[rb];
			if (tmp_indx != -1) {
				b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[tmp_indx]);
				//if (b_tran[1] < 0.04) {
				if (b_tran[1] < 0.1) {
					block_pos[tmp_indx][0] = 11.5 - tmp_indx*0.2;
					block_pos[tmp_indx][1] = 0.05;
					block_pos[tmp_indx][2] = -5.5;
					wb_supervisor_field_set_sf_vec3f(block_tran[tmp_indx], block_pos[tmp_indx]);
//					printf("++++ Remove %d!!\n", tmp_indx);
					rb_cnt++;
				}
			}
		}
		remove_block_cnt -= rb_cnt;

		
		if (wb_receiver_get_queue_length(receiver) > 0) {
			const void* buffer =  wb_receiver_get_data(receiver);   
			double* receive_message;			
			receive_message = (double*)buffer;
			
			if (receive_message[0] == BID_MESSAGE && supervisor_state == STATE_WAIT_BIDDER){//receive_bidders: (0)task / (1)auctioneer_id / (2)bidder_id / (3)quality / (4)cost								
				for (i=0; i<unallo_cnt; i++) {
					if ((int)receive_message[1] == unallo_task[i] && (int)receive_message[2] == 9) {
						int tmp_rob_ID = (int)receive_message[3];
						int ID_check = find_ID(bid_cnt, tmp_rob_ID, bidder_ID[i]);
						if (ID_check == 0) {
							for (int j=0; j<9; j++)
								receive_bidders[bid_cnt][j] = receive_message[j+1];							
							
							bidder_ID[i][bid_cnt] = tmp_rob_ID;							
							printf("[cnt=%d T=%d/id=%d/q=%.2f/c=%.2f/d2=%.2f/t2=%.2f/Wb=%.2f]\n", bid_cnt, 
							(int)receive_bidders[bid_cnt][0], (int)receive_bidders[bid_cnt][2], receive_bidders[bid_cnt][3], 
							receive_bidders[bid_cnt][4], receive_bidders[bid_cnt][5], receive_bidders[bid_cnt][6], receive_bidders[bid_cnt][7]);										
							bid_cnt++;
						}
					}
				}										
			}			
			else if (receive_message[0] == ABANDON_MESSAGE) {				
				int tmp_task = (int)receive_message[1];
				
				if (tmp_task < TASK1_2 || tmp_task == TASK2_1 || tmp_task == TASK3_1) {
					int abandon_task = (int)receive_message[1];
					printf("Robot %d abandoned task %d\n", (int)receive_message[2], abandon_task);
					if (tmp_task != TASK2_1 && tmp_task != TASK3_1) {
						zone_task_cnt--;
						printf("Tot assign cnt=%d\n", zone_task_cnt);									
						
						int tmp_sft = 0;
						for (i=0; i<assigned_cnt; i++) {
							if (assigned_task[i] == abandon_task) {
								tmp_sft = 1;
								unallo_task[unallo_cnt] = assigned_task[i];
								unallo_cnt++;				
								printf("Put rejected task %d back to store_task!\n", assigned_task[i]);						
							}										
							assigned_task[i] = assigned_task[i+tmp_sft];						
						}				
						assigned_cnt--;											
					}
					else {
						printf("[new]Put rejected task %d back to unallo_task!\n", abandon_task);		
						unallo_task[unallo_cnt] = abandon_task;
						unallo_cnt++;
						supervisor_state = STATE_AUCTION;
					}
				}
			}	
			else if (receive_message[0] == DONE_MESSAGE) {
				if (carry_auctioner == -1) {
					carry_cnt += (int)receive_message[3];
					tot_block = 0;
					int done_zone = (int)receive_message[1];
//					printf("Done zone is %d!\n", done_zone);
					for (int i=0; i<NUM_BLOCK; i++) {					
						b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[i]);
						if (b_tran[1] > 0) {
							if (fabs(b_tran[0]) <= 0.88 && b_tran[2] >= 6.05 && b_tran[2] <= 7.95) {
								tot_block++;														
//								printf("Block x=%.2f / z=%.2f Tot=%d\n", b_tran[0], b_tran[2], tot_block);							
							}
							else {
								double x_range[2] = {0, 0};
								double z_range[2] = {0, 0};
								if (done_zone == 0) {
									x_range[0] = 0.88;
									x_range[1] = 1.4;
									z_range[0] = 6.05;
									z_range[1] = 7.95;
								}
								else if (done_zone == 1) {
									x_range[0] = -0.88;
									x_range[1] = 0.88;
									z_range[0] = 5.2;
									z_range[1] = 6.05;
								}
								else if (done_zone == 2) {
									x_range[0] = -1.4;
									x_range[1] = -0.88;
									z_range[0] = 6.05;
									z_range[1] = 7.95;
								}								
								if ((b_tran[0] > x_range[0] && b_tran[0] < x_range[1]) && (b_tran[2] > z_range[0] && b_tran[2] < z_range[1])) {
									carry_fail_cnt++;
									printf("Block %d is out of Tray 1\n", i);									
									block_pos[i][0] = 11.5 - i*0.2;
									block_pos[i][1] = 0.05;
									block_pos[i][2] = -5.5;
									wb_supervisor_field_set_sf_vec3f(block_tran[i], block_pos[i]);
									lost_block_cnt[1]++;
									printf("Fail to load!! %d\n", lost_block_cnt[1]);
								}								
							}
						}
					}
					//printf("@@ Tot block =%d\n", tot_block);
					if (tot_block >= max_tray1_block) {
						printf("[[Tray 1 is full now (%d)\n", tot_block);					
						printf("[[Auction Task2_1]]\n");
						unallo_task[unallo_cnt] = TASK2_1;					
						unallo_cnt++;
						supervisor_state = STATE_AUCTION;	
						carry_auctioner = (int)receive_message[2];
					}
					if (carry_cnt != 0)
						carry_success_rate = carry_success_rate + carry_scale* ((double)(carry_cnt - carry_fail_cnt) / (double)carry_cnt - carry_success_rate);		
				}
				if ((int)receive_message[4] == 1) {
					if (period_cnt > EVENT_PERIOD * 0.9) 
						period_cnt = EVENT_PERIOD * 0.5;
					zone_mask[(int)receive_message[1]] = 1;
//					printf("[[[[[ Zone %d is cleared\n", (int)receive_message[1]+1);
//					printf("[[[[[ Zone %d is cleared\n", (int)receive_message[1]+1);
					printf("[[[[[ Zone %d is cleared\n", (int)receive_message[1]+1);						
				}				
			}
			else if (receive_message[0] == EMPTY_MESSAGE) {		
				lift_flag = 2;		
				carry_auctioner = -1;
			}
			else if (receive_message[0] == TRAY_BACK_MESSAGE || receive_message[0] == PUSH_TOGETHER_MESSAGE) {					 			
				wb_supervisor_field_set_sf_rotation(tray1_field_rot, tray_rot[0]);
				wb_supervisor_field_set_sf_vec3f(tray1_field_tran, tray1_init_pos);								
			}
			else if (receive_message[0] == PUSH_TOGETHER_MESSAGE) {
				set_pos_flag = 0;
			}
			else if (receive_message[0] == WHAT_BLOCK_MESSAGE) {					
				if (store_block[0] < 10 || (store_block[0] >= 30 && store_block[0] < 40))				
					block_category = TYPE_GARBAGE;
				else if (store_block[0] < 20 || (store_block[0] >= 40 && store_block[0] < 50))				
					block_category = TYPE_RECYCLE;
				else
					block_category = TYPE_STONE;		
				send_message[send_cnt][0] = BLOCK_IS_MESSAGE;
				send_message[send_cnt][1] = receive_message[1];
				send_message[send_cnt][2] = (double)block_category;										
				send_message[send_cnt][29] = 3;
				send_cnt++;
				
			}			
			else if (receive_message[0] == SHIFT_BLOCK_MESSAGE) {
				recog_cnt++;
				int tmp_indx = (int)receive_message[1];					
				double tmp_block_pos[3];											
				if (tmp_indx != block_category)
					recog_fail_cnt++;
				
				b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[store_block[0]]);
				if (b_tran[0] >= tray_area[tmp_indx][0] && b_tran[0] <= tray_area[tmp_indx][1] && b_tran[1] > 0 && b_tran[2] >= tray_area[tmp_indx][2] && b_tran[2] <= tray_area[tmp_indx][3]) {
					tray_block_cnt[tmp_indx]++;												
					complete_block[complete_cnt] = store_block[0];
					complete_cnt++;						
					printf("[[Received Number of Tray %d: %d\n", tmp_indx+2, tray_block_cnt[tmp_indx]);
				}
				else {
					printf("[[Block is failed to load to Tray %d\n", tmp_indx);					
					tmp_block_pos[0] = 11.5 - i*0.2;
					tmp_block_pos[1] = 0.05;
					tmp_block_pos[2] = -5.5;
					wb_supervisor_field_set_sf_vec3f(block_tran[store_block[0]], tmp_block_pos);
				}
				
				coll_cnt--;
				for (int i=0; i<coll_cnt; i++) {
					store_block[i] = store_block[i+1];
					tmp_block_pos[0] = out_block_pos[i][0];
					tmp_block_pos[1] = 0.06;					
					tmp_block_pos[2] = out_block_pos[i][1];
					wb_supervisor_field_set_sf_vec3f(block_tran[store_block[i]], tmp_block_pos);
					//printf("Shift Block %d\n", i);
				}
				
				send_message[send_cnt][0] = BLOCK_NUM_MESSAGE;
				send_message[send_cnt][1] = (double)tmp_indx;
				send_message[send_cnt][2] = (double)tray_block_cnt[tmp_indx];
				send_message[send_cnt][29] = 3;
				send_cnt++;				
				recog_success_rate = recog_success_rate + recog_scale * ((double)(recog_cnt - recog_fail_cnt) / (double)recog_cnt - recog_success_rate);
			}
			else if (receive_message[0] == REMOVE_BLOCK_MESSAGE) {												
				int tmp_tray = (int)receive_message[1];
				int tmp_block_cnt = 0;
				double tmp_block_tran[3];
				
				//carry_cnt += max_small_tray_block;

				if (tmp_tray == 0) {					
					for (int  i=0; i<complete_cnt; i++) {
						b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[complete_block[i]]);
						if (b_tran[0] >= 4.6 && b_tran[1] > 0 && b_tran[2] >= 18.5 && b_tran[2] <= 19.5) {
							tmp_block_tran[0] = room1_3_block_pos[room_cnt[0]][0] + 6.5;
							tmp_block_tran[1] = 0.051;
							tmp_block_tran[2] = room1_3_block_pos[room_cnt[0]][1] + 18.1;
							room_cnt[0]++;
							wb_supervisor_field_set_sf_vec3f(block_tran[complete_block[i]], tmp_block_tran);
//							printf("++++ Room %d accepts block %d\n", tmp_tray, i);
							if ((complete_block[i] > -1 && complete_block[i] < 10) || (complete_block[i] >= 30 && complete_block[i] < 40))				
								tot_block_cnt[0]++;
							else
								err_block_cnt++;
							complete_block[i] = -1;
							tmp_block_cnt++;
						}												
					}					
				}
				else if (tmp_tray == 1) {					
					for (int  i=0; i<complete_cnt; i++) {
						b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[complete_block[i]]);
						if (b_tran[1] > 0 && b_tran[2] >= 23.5) {							
							tmp_block_tran[0] = room2_block_pos[room_cnt[1]][0] - 0.3;
							tmp_block_tran[1] = 0.051;
							tmp_block_tran[2] = room2_block_pos[room_cnt[1]][1];
							room_cnt[1]++;
							wb_supervisor_field_set_sf_vec3f(block_tran[complete_block[i]], tmp_block_tran);
//							printf("++++ Room %d accepts block %d\n", tmp_tray, i);
							if ((complete_block[i] >= 10 && complete_block[i] < 20) || (complete_block[i] >= 40 && complete_block[i] < 50))
								tot_block_cnt[1]++;
							else
								err_block_cnt++;
							complete_block[i] = -1;
							tmp_block_cnt++;
						}
					}				
				}
				else if (tmp_tray == 2) {					
					for (int  i=0; i<complete_cnt; i++) {
						b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[complete_block[i]]);												  
						if (b_tran[0] <= -4.6 && b_tran[1] > 0 && b_tran[2] >= 18.5 && b_tran[2] <= 19.5) {
							tmp_block_tran[0] = room1_3_block_pos[room_cnt[2]][0] - 5.1;
							tmp_block_tran[1] = 0.051;
							tmp_block_tran[2] = room1_3_block_pos[room_cnt[2]][1] + 18.1;
							room_cnt[2]++;
							wb_supervisor_field_set_sf_vec3f(block_tran[complete_block[i]], tmp_block_tran);
//							printf("++++ Room %d accepts block %d\n", tmp_tray, i);
							if (complete_block[i] >= 20 || (complete_block[i] >= 50 && complete_block[i] < 60))
								tot_block_cnt[2]++;
							else
								 err_block_cnt++;
							complete_block[i] = -1;
							tmp_block_cnt++;
						}						
					}				
				}
				
				tray_block_cnt[tmp_tray] -= tmp_block_cnt;
				carry_fail_cnt += tray_block_cnt[tmp_tray];
				printf("Tray %d left block =%d\n", tmp_tray+2, tray_block_cnt[tmp_tray]);
				int tmp_cnt = 0;
				int tmp_block[12];
				for (int i=0; i<complete_cnt; i++) {
					if (complete_block[i] != -1) {
						tmp_block[tmp_cnt] = complete_block[i];
						tmp_cnt++;
					}
				}
				for (i=0; i<tmp_cnt; i++)
					complete_block[i] = tmp_block[i];
				complete_cnt = tmp_cnt;

				send_message[send_cnt][0] = LEFT_BLOCK_MESSAGE;				
				send_message[send_cnt][1] = (double)tray_block_cnt[tmp_tray];
				send_message[send_cnt][29] = 2;
				send_cnt++;

			}
			else if (receive_message[0] == LOAD_MESSAGE) {	
				grip_cnt++;
				int task_indx = (int)receive_message[3];
				int block_indx = (int)receive_message[4];								
				if (block_indx != -1) {
					printf("Task indx(0~8)=%d / block indx(0~7)=%d\n", task_indx, block_indx);
					if ((int)receive_message[5] == 1) {									
						grip_fail_cnt++;
						int tmp_indx = zone_task[task_indx][block_indx];					
						printf("Block %d pick fail!\n", tmp_indx);
						tmp_indx--;
						remove_block[remove_block_cnt] = tmp_indx;
						remove_block_cnt++;			
						lost_block_cnt[0]++;
						printf("Fail to pick!! %d (indx=%d)\n", lost_block_cnt[0], tmp_indx);
						
						block_pos[tmp_indx][0] = 11.5 - tmp_indx*0.2;
						block_pos[tmp_indx][1] = 0.05;
						block_pos[tmp_indx][2] = -5.5;
						
						wb_supervisor_field_set_sf_vec3f(block_tran[tmp_indx], block_pos[tmp_indx]);
					}
					zone_task[task_indx][block_indx] = -1;
				}
				if ((int)receive_message[1] == -1 && (int)receive_message[2] == 1) {
					period_cnt = 0;
					int zone_indx = 0;
					if (task_indx < 3)
						zone_indx = 0;
					else if (task_indx < 6)
						zone_indx = 1;
					else
						zone_indx = 2;
					zone_mask[zone_indx] = 1;
//					printf("[[[[[ Zone %d is cleared\n", zone_indx+1);
//					printf("[[[[[ Zone %d is cleared\n", zone_indx+1);
					printf("[[[[[ Zone %d is cleared\n", zone_indx+1);			
				}
				if (grip_cnt != 0)
					grip_success_rate = grip_success_rate + grip_scale * ((double)(grip_cnt - grip_fail_cnt) / (double)grip_cnt - grip_success_rate);
			}
			else if (receive_message[0] == PULL_DONE_MESSAGE) {
				int tmp_tray = (int)receive_message[1];				
				if (tmp_tray < 0 || tmp_tray > 2) { 
					printf("Break here!!\n");
					printf("Break here!!\n");
					printf("Break here!!\n");
					printf("Break here!!\n");
					printf("Break here!!\n");
				}
				wb_supervisor_field_set_sf_vec3f(small_tray_field_tran[tmp_tray], small_tray_pos[tmp_tray]);
				wb_supervisor_field_set_sf_rotation(small_tray_field_rot[tmp_tray], tray_rot[tmp_tray+1]);														
			}
			else if (receive_message[0] == SORT_DONE_MESSAGE) {
				sorter_flag = 0;
				sorter_ID = -1;
				if (coll_cnt > 0) {
					unallo_task[unallo_cnt] = TASK3_1;					
					unallo_cnt++;
					supervisor_state = STATE_AUCTION;
				}
			}
			else if (receive_message[0] == RECOG_MESSAGE) {
				recog_cnt++;
				if (receive_message[1] == 1) {
					recog_fail_cnt++;
					printf("Recog fail\n");
				}				
				recog_success_rate = recog_success_rate + recog_scale * ((double)(recog_cnt - recog_fail_cnt) / (double)recog_cnt - recog_success_rate);				
			}
		}
		
		
		if (supervisor_state == STATE_AUCTION) {
			if (unallo_cnt > 0) {								
				if (broadcast_cnt >= unallo_cnt) {				
					supervisor_state = STATE_WAIT_BIDDER;
					broadcast_cnt = 0;						
				}
				else {				
					//printf("[   [ Auction Task %d ]    ]\n", unallo_task[broadcast_cnt]);
					send_message[send_cnt][0] = AUCTION_MESSAGE;				
					send_message[send_cnt][1] = (double)unallo_task[broadcast_cnt];								
					send_message[send_cnt][2] = 9;
					send_message[send_cnt][3] = INDI_AUCTION;
					send_message[send_cnt][29] = 4;
					send_cnt++;
					broadcast_cnt++;					
				}				
			}			
		}
		else if (supervisor_state == STATE_WAIT_BIDDER) {
			wait_cnt++;							
			if (wait_cnt > AUCTION_WAIT_TIME) {						
				wait_cnt = 0;				
				if (bid_cnt > 0) 
					supervisor_state = STATE_GET_BIDDER;
				else {
					supervisor_state = STATE_AUCTION;
					//printf("[[No bidder! Auction again]]\n");
				}
			}				
			//printf("Wait %d\n", wait_cnt);
		}
		else if (supervisor_state == STATE_GET_BIDDER) {					
			int chop_cnt = 0;
			int tmp_idx = -1;
			for (int i=0; i<unallo_cnt; i++) {
				int tmp_bid_cnt = 0;
				int fin_cnt = 0;
				double tmp_bid_list[NUM_ROBOT][9];//receive_bidders: (0)task / (1)auctioneer_id / (2)bidder_id / (3)quality / (4)cost
				for (int j=0; j<bid_cnt; j++) {
					if (unallo_task[i] == receive_bidders[j][0]) {
						for (int k=0; k<9; k++)
							tmp_bid_list[tmp_bid_cnt][k] = receive_bidders[j][k];						
						tmp_bid_cnt++;
					}					
				}	
				//printf("for task %d, bidder number is %d\n", unallocated_task[i], tmp_bid_cnt);
				chop_cnt += tmp_bid_cnt;															
				if (tmp_bid_cnt > 0) { 							
					int winner = 0;
					double winner_utility = 0;
					if (test_mode >= 3 && test_mode <= 4 )
						bid_weight = get_ave_bid_weight(bid_weight, tmp_bid_list, tmp_bid_cnt);
					if (tmp_bid_cnt == 1) {
						winner = (int)tmp_bid_list[0][2];								
						printf("Only one bidder, %d is winner for task %d! / bid_weight=%.2f\n", winner, unallo_task[i], bid_weight);
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
						double bid_utility[NUM_ROBOT];																		
						get_utility(tmp_bid_list, bid_utility, bid_weight, s_weight, tmp_bid_cnt);
						double max_utility = 0;
						for (int t=0; t<tmp_bid_cnt; t++) {
							if (max_utility < bid_utility[t]) {
								max_utility = bid_utility[t];
								winner = (int)tmp_bid_list[t][2];//bidder_id
							}
						}	
						if (winner != receive_bidders[tmp_bid_cnt-1][2] && max_utility == bid_utility[tmp_bid_cnt-1]) {
							int winner_idx = irand(tmp_bid_cnt);
							max_utility = bid_utility[winner_idx];
							winner = (int)tmp_bid_list[winner_idx][2];							
							printf("Select winner by random! Winner is %d\n", winner);
						}
						winner_utility = max_utility;
						printf("%d bidders, %d is winner! / bid_weight=%.2f\n", tmp_bid_cnt, winner, bid_weight);
					}			

					get_d2t2(tmp_bid_list, ave_d2t2, tmp_bid_cnt);
					printf("[[Average of d2 =%.2f / t2=%.2f (bid_cnt=%d)]]\n", ave_d2t2[0], ave_d2t2[1], bid_cnt);

					final_winners[winner_cnt][0] = (double)unallo_task[i];//task
					final_winners[winner_cnt][1] = 9;//auctioneer_id
					final_winners[winner_cnt][2] = (double)winner;//bidder_id	
					final_winners[winner_cnt][3] = ave_d2t2[0];
					final_winners[winner_cnt][4] = ave_d2t2[1];
					final_winners[winner_cnt][5] = bid_weight;
					final_winners[winner_cnt][6] = winner_utility;
					if (unallo_task[i] == TASK3_1) {						
						sorter_ID = winner;
						final_winners[winner_cnt][7] = (double)coll_cnt;								
					}
					else if (unallo_task[i] < TASK1_2) {
						int tmp_sel_zone_task = unallo_task[i];//task(zone)
						for (int j=0; j<MAX_TASK_BLOCK; j++) {
							int tmp_indx = zone_task[tmp_sel_zone_task][j];//1~
							if (tmp_indx != -1) {
								tmp_indx--;
								b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[tmp_indx]);
								block_pos[tmp_indx][0] = b_tran[0];
								block_pos[tmp_indx][2] = b_tran[2];
								final_winners[winner_cnt][3*(j+2)+1]   = block_pos[tmp_indx][0];//x
								final_winners[winner_cnt][3*(j+2)+2] = block_pos[tmp_indx][2];//z
								final_winners[winner_cnt][3*(j+2)+3] = (double)(tmp_indx*10 + j);						
							}	
							else {
								final_winners[winner_cnt][3*(j+2)+1] = -10000;
								final_winners[winner_cnt][3*(j+2)+2] = -10000;
								final_winners[winner_cnt][3*(j+2)+3] = -10000;
							}
						}
					}
					
					
					winner_cnt++;
					
					if (unallo_task[i] == TASK2_1 || unallo_task[i] == TASK3_1) {						
						tmp_idx = i;
					}
					else {
						allo_task[allo_cnt] = unallo_task[i];					
						allo_cnt++;															
					}
				}			
			}
			if (tmp_idx > -1) {
				for (i=tmp_idx; i<unallo_cnt-1; i++) {
					unallo_task[i] = unallo_task[i+1];				
				}
				unallo_cnt--;
			}						
			bid_cnt = 0; 
			if (chop_cnt > 0) {	
				supervisor_state = STATE_REMOVE_TASK;
			}
			else {				
				printf("Unexpected Bidder! I will chop it!!\n");								
				winner_cnt = 0;
				curr_assign_cnt = 0;
				allo_cnt = 0;
				supervisor_state = STATE_AUCTION;
				//auc_state = HALT;
			}
		}		
		else if (supervisor_state == STATE_REMOVE_TASK) {
			for (int i=0; i<allo_cnt; i++) {
				int sft=0;
				for (int j=0; j<unallo_cnt; j++) {
					if (allo_task[i] == unallo_task[j]) {
						sft=1;				
						assigned_task[assigned_cnt] = allo_task[i];					
						assigned_cnt++;
					}
					unallo_task[j] = unallo_task[j+sft];
				}
			}	
			unallo_cnt -= allo_cnt;			
			allo_cnt = 0;				
			supervisor_state = STATE_ASSIGN;				
		}
		else if (supervisor_state == STATE_ASSIGN) {//(0)header / (1)task / (2)auctioneer_id / (3)bidder_id / (4,5)x1,z1 / (6,7)x2,z2 / (8,9)x3,z3 /(10,11)x4,z4			
			send_message[send_cnt][0] = ASSIGN_MESSAGE;					
			for (i=0; i<7; i++)
				send_message[send_cnt][i+1] = final_winners[curr_assign_cnt][i];					
			
			if (send_message[send_cnt][1] < TASK1_2) {
				zone_task_cnt++;
				//for (i=0; i<(MAX_TASK_BLOCK+2)*3; i++)
				for (i=0; i<28; i++)
					send_message[send_cnt][i+1] = final_winners[curr_assign_cnt][i];					
				printf("[Assign robot %d task %d] Tot=%d\n", (int)send_message[send_cnt][3], (int)send_message[send_cnt][1], zone_task_cnt);							
				send_message[send_cnt][29] = 29;
				send_cnt++;				
			}						
			else if (send_message[send_cnt][1] == TASK2_1) {				
				printf("[Assign robot %d Task2_1]\n", (int)send_message[send_cnt][3]);			
				send_message[send_cnt][29] = 8;
				send_cnt++;
			}
			else if (send_message[send_cnt][1] == TASK3_1) {				
				send_message[send_cnt][8] = final_winners[curr_assign_cnt][7];
				send_message[send_cnt][9] = (double)tray_block_cnt[0];
				send_message[send_cnt][10] = (double)tray_block_cnt[1];
				send_message[send_cnt][11] = (double)tray_block_cnt[2];
				printf("[Assign robot %d Task3_1] num_block=%d d2=%.2f t2=%.2f\n", (int)send_message[send_cnt][3], 
					(int)final_winners[curr_assign_cnt][5], send_message[send_cnt][4], send_message[send_cnt][5]);			
				send_message[send_cnt][29] = 13;
				send_cnt++;
				sorter_flag = 1;
			}			
			curr_task = (int)send_message[send_cnt-1][1];
			curr_assign_cnt++;		
			//printf("Tot assign cnt=%d\n", tot_assign_cnt);
			//printf("assgn cnt %d winner cnt %d\n", assign_cnt, winner_cnt);
			if (curr_assign_cnt == winner_cnt) {	
				winner_cnt = 0;
				curr_assign_cnt = 0;
				supervisor_state = STATE_AUCTION;
			}			
		}
		
		if (lift_flag == 2) {				
			lift_flag = 3;
			tmp_coll_cnt = 0;
			carry_cnt += tot_block;
			printf("[[[]]] Arrange block:");
			for (int i=0; i<NUM_BLOCK; i++) {					
				b_tran = wb_supervisor_field_get_sf_vec3f(block_tran[i]);														
				if (fabs(b_tran[0]) <= 0.88 && b_tran[2] >= 12.7 && b_tran[2] < 14.2) {			
					tmp_store_block[tmp_coll_cnt] = i;			
					printf("%d / ", i);
					tmp_coll_cnt++;
					
					block_pos[i][0] = 11.5 - i*0.2;
					block_pos[i][1] = 0.05;
					block_pos[i][2] = -5.5;
					
					wb_supervisor_field_set_sf_vec3f(block_tran[i], block_pos[i]);
				}
			}						
			printf("\n");
			tmp_coll_cnt2 = tmp_coll_cnt;
			tot_block -= tmp_coll_cnt;
			carry_fail_cnt += tot_block;
			printf("Left blocks in Tray1 = %d\n", tot_block);
			if (carry_cnt != 0)
				carry_success_rate = carry_success_rate + carry_scale * ((double)(carry_cnt - carry_fail_cnt) / (double)carry_cnt - carry_success_rate);
		}
		else if (lift_flag == 3) {
			if (gate_pos[2] < 15.6)
				gate_pos[2] += 0.03;
			else {
				gate_pos[2] = 15.6;
				lift_flag = 4;
			}
		}
		else if (lift_flag == 4) {
			double tmp_block_pos[3];
	 		for (i=0; i<tmp_coll_cnt2; i++) {								
				if (no_prob_flag == 1)
					store_block[i+coll_cnt] = tmp_store_block[i];				
				else {
					int r_val = irand(tmp_coll_cnt);
					store_block[i+coll_cnt] = tmp_store_block[r_val];				
					tmp_coll_cnt = find_shift(tmp_coll_cnt, tmp_store_block[r_val], tmp_store_block);
				}
				tmp_block_pos[0] = out_block_pos[i+coll_cnt][0];
				tmp_block_pos[1] = 0.06;
				tmp_block_pos[2] = out_block_pos[i+coll_cnt][1];
				wb_supervisor_field_set_sf_vec3f(block_tran[store_block[i+coll_cnt]], tmp_block_pos);
			}					
						
			broadcast_cnt = 0;
			
			coll_cnt += tmp_coll_cnt2;
			tmp_coll_cnt = 0;
			printf("[[Left coll cnt=%d]]\n", coll_cnt);
			
			if (coll_cnt > 0 && sorter_flag == 0) {
		 		unallo_task[unallo_cnt] = TASK3_1;					
				unallo_cnt++;
				supervisor_state = STATE_AUCTION;
			}		
			lift_flag = 5;
		}		
		else if (lift_flag == 5) {
			if (gate_pos[2] > 13.6)
				gate_pos[2] -= 0.03;
			else {
				gate_pos[2] = 13.6;
				lift_flag = -1;
				send_message[send_cnt][0] = EMPTY_DONE_MESSAGE;
				send_message[send_cnt][29] = 1;
				send_cnt++;	
			}
		}		
		
		wb_supervisor_field_set_sf_vec3f(gate_field_tran, gate_pos);
					
		if (send_cnt > 0) {			
			wb_emitter_send(emitter, send_message[0], (int)(send_message[0][29])*sizeof(double));
			send_cnt--;
			for (int s=0; s<send_cnt; s++) {
				for (int s2=0; s2< send_message[s+1][29]; s2++)
					send_message[s][s2] = send_message[s+1][s2];
				send_message[s][29] = send_message[s+1][29];
			}			
		}

		
		
		time += time_sec;// time_step = 32(ms)
		int sec = (int) time % 60;				
		int min = (int) (time / 60);
		char time_string[64];

	
		int check_tot_block = tot_block_cnt[0]+tot_block_cnt[1]+tot_block_cnt[2]+err_block_cnt+lost_block_cnt[0]+lost_block_cnt[1]+tot_block+tray_block_cnt[0]+tray_block_cnt[1]+tray_block_cnt[2];

		sprintf(time_string, "%02d:%02d / event time=%d", min, sec, period_cnt);
		wb_supervisor_set_label(2, time_string, 0.1, 0.01, 0.07, 0x000000, 0.0); //id, x, y, size, color, transparency							

		sprintf(time_string, "Good=%d/Bad=%d/Lost=(%d,%d)", tot_block_cnt[0]+tot_block_cnt[1]+tot_block_cnt[2], err_block_cnt, lost_block_cnt[0], lost_block_cnt[1]);
		wb_supervisor_set_label(3, time_string, 0.6, 0.01, 0.07, 0x000000, 0.0); //id, x, y, size, color, transparency							

		sprintf(time_string, "Tray1,2,3,4=(%d, %d, %d, %d)", tot_block, tray_block_cnt[0], tray_block_cnt[1], tray_block_cnt[2]);
		wb_supervisor_set_label(4, time_string, 0.6, 0.06, 0.07, 0x000000, 0.0); 							

		
		sprintf(time_string, "Tot=%d", check_tot_block);
		wb_supervisor_set_label(5, time_string, 0.6, 0.11, 0.07, 0x000000, 0.0); 							

		sprintf(time_string, "Grip=%.2f Sort=%.2f Carry=%.2f", grip_success_rate, recog_success_rate, carry_success_rate);
		wb_supervisor_set_label(6, time_string, 0.1, 0.06, 0.07, 0x000000, 0.0); 							

		
		if (time_cnt != prev_time_cnt2) {
			prev_time_cnt2 = time_cnt;
			W_measure = scalar * (exp(s_factor*(pow(G[time_cnt], sqare_num) - 1.0)) - bias_bid_weight);
			double v_rand = 0.5*drand();
			if (rand()%2 == 0)
				W_measure += v_rand;
			else
				W_measure -= v_rand;

			if (test_mode >= 3 && test_mode <= 4 )
				bid_weight = update_bid_weight(bid_weight, W_measure);
									
			if (full_record == 1 && time_cnt == store_time[t_cnt]) {						
				err_data[t_cnt][0] = grip_success_rate;
				err_data[t_cnt][1] = recog_success_rate;
				err_data[t_cnt][2] = carry_success_rate;			
				if (test_mode >= 3 && test_mode <= 4 ) {
					w_data[t_cnt] = bid_weight;			
					v_data[t_cnt] = G[time_cnt];
				}
				t_cnt++;
			}
		}
		if (time_cnt >= real_termination_time) {
			if (full_record == 1) {				
				if (test_mode != 5 && test_mode != 6) {
					sprintf(f_name, "..\\err_%d_%d.txt", rob_num, test_mode);
					err_file = fopen(f_name, "a+");	
					for (int t=0; t<11; t++) 
						fprintf(err_file, "%f %f %f\n", err_data[t][0], err_data[t][1], err_data[t][2]);
					fclose(err_file);
					
					if (test_mode == 4 ) {
						sprintf(f_name, "..\\w_%d_%d_%d.txt", rob_num+1, rob_num, test_mode);
						w_file = fopen(f_name, "a+");						
						sprintf(f_name, "..\\v_%d_%d_%d.txt", rob_num+1, rob_num, test_mode);
						v_file = fopen(f_name, "a+");						
						for (t=0; t<11; t++) {					
							fprintf(w_file, "%f\n", w_data[t]);									
							fprintf(v_file, "%f\n", v_data[t]);
						}
						fclose(w_file);			
						fclose(v_file);
					}
				}
				
				else if (test_mode == 5) {
					sprintf(f_name, "..\\fin_err_%d.txt", rob_num);
					fin_err_file = fopen(f_name, "a+");	
					fprintf(fin_err_file, "%f %f %f\n", grip_success_rate, recog_success_rate, carry_success_rate);								
					fclose(fin_err_file);
				}
			}
			sim_cnt++;						
			
			if (full_record == 1) {
				sim_cnt_file = fopen("val.txt", "w");				
				if (sim_cnt < num_sim_iteration) {
					fprintf(sim_cnt_file, "%d", sim_cnt);
					fclose(sim_cnt_file);
					wb_supervisor_simulation_revert();
					while(1);
				}
				else {
					fprintf(sim_cnt_file, "%d", 0);
					fclose(sim_cnt_file);
					break;
				}			
			}
		}

		wb_receiver_next_packet(receiver);
 		wb_robot_step(time_step);
	}   	
	wb_robot_cleanup();
	return 0;
}

  