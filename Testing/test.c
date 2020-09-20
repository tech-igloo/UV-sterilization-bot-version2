#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static double prev_time = 0;                     //stores the previous time step, gets updated to current time after sysCall_sensing
static double timeDiff = 0;                      //time difference between two simulation time steps
static double prev_timeDiff = 0;
static double val_left_prev = 0;                 //total angle rotated by left wheel till previous time step
static double val_right_prev = 0 ;               //total angle rotated by right wheel till previous time step
static double dist_traversed = 0;                //distance travlled by bot along straight line(+ve for forward and -ve for negative)
static double angle_rotated = 0;                 //angle rotated by bot(+ve for clockwise and -ve for anticlockwise)
static double lin_vel = 0;                       //linear velocity of the bot
static double ang_vel = 0;                       //angular velocity of the bot
static double prev_lin_vel = 0;                       //linear velocity of the bot
static double prev_ang_vel = 0;                       //angular velocity of the bot
static int point_index = 0;                     //index for accessing the appropriate point from points list
static double accumulated_error = 0;             //integral term in PID formula
static double current_error = 0;                 //current error in PID formula
static double prev_error = 0;                    //error in the previous time step
static int pid_flag = 0;                        //flag that signifies that the PID controller's job is done
static double angle_required = 0;                //angle that the bot needs to rotate to align itself with its destination point
static double dist_required = 0;                 //distance from stop point that the bot needs to travel to reach the destination point
static int rotating_flag = 1;                   //flag that signifies whether the bot should rotate or move along a straight line
static double* points = NULL;                    //stores the points in the path
static int points_len = 0;                      //length of the path
static double stop_point[2] = {0};                 //the position of the bot from which distance travlled is being calculated
static double prev_point[2] = {0};               //the next point that the bot needs to travel to
static double current_point[2] = {0};            //current co-ordinates of the bot*/
static int obstacle_flag[5] = {0};
static double time_flag = 0;
static int detect_flag = 0;
static double minDist = 0.2;
static double proxSensDist[5] = {0.5};
static int timestep = 1;
// static FILE* f_sensing;
// f_sensing = fopen("./sensor_data.txt", "r");
static double val_left = 0;
static double val_right = 0;
static int encoderResolution = 1000;
#define RPP (2*M_PI)/encoderResolution;
static double totalPosLeft = 0;
static double totalPosRight = 0;
static double leftJointPosition = 0;
static double rightJointPosition = 0;
static double leftJointvel = 0;
static double rightJointvel = 0;
static double timeOfActuation = 0;
static double prevTimeofActuation = 0; 
static double verify_dist_traversed = 0;                //distance travlled by bot along straight line(+ve for forward and -ve for negative)
static double verify_angle_rotated = 0;
static double verify_current_point[2] = {0};
static int run=0;
static int prev_res[5] = {0};
static double prev_distance[5] = {0};
static double test_var = 0;


void forward(double vel){
	leftJointPosition = (timeOfActuation-prevTimeofActuation)*leftJointvel;
	rightJointPosition = (timeOfActuation - prevTimeofActuation)*rightJointvel;
	leftJointvel = vel;
	rightJointvel = vel;
	prevTimeofActuation = timeOfActuation;
}

void rotate(double vel){
	leftJointPosition = (timeOfActuation-prevTimeofActuation)*leftJointvel;
	rightJointPosition = (timeOfActuation - prevTimeofActuation)*rightJointvel;
	leftJointvel = vel;
	rightJointvel = -vel;
	prevTimeofActuation = timeOfActuation;
}

void stop(){
	leftJointPosition = (timeOfActuation-prevTimeofActuation)*leftJointvel;
	rightJointPosition = (timeOfActuation - prevTimeofActuation)*rightJointvel;
	leftJointvel = 0;
	rightJointvel = 0;
	prevTimeofActuation = timeOfActuation;
}


double getAngleMinusAlpha(double angle, double alpha){
	double sinAngle0=sin(angle);
	double sinAngle1=sin(alpha);
	double cosAngle0=cos(angle);
	double cosAngle1=cos(alpha);
	double sin_da=sinAngle0*cosAngle1-cosAngle0*sinAngle1;
	double cos_da=cosAngle0*cosAngle1+sinAngle0*sinAngle1;
	return atan2(sin_da, cos_da);
}


double* create_coord_array(int n){
// int main(){
	int path_number = 0;
	//int n = 3;
    int line_length = 0;
    double val = 0;
    char c;
    FILE* f_r = fopen("./test.txt", "r");
    if(f_r == NULL){
        printf("Error opening file temp.txt\n");
    }
    // Have to handle the invalid path case
    int counter_to_path_start=0;
    int space_count = 0;
    for(;;){
        if(path_number+1==n){
            while(1){
                c = fgetc(f_r);
                if(c == ' ')
                    space_count+=1;
                if( c == EOF || c == '\n' ){
                    ++line_length;
                    break;
                }
                ++line_length;
            }
            fseek(f_r, -(line_length+1), SEEK_CUR);
            //c = fgetc(f_r);
            //printf("%c THE CHAR", c);
            char* path = (char *)calloc(line_length, sizeof(char));
            strcpy(path, "\0");
            double* points_array = (double *)calloc((space_count+1), sizeof(double));
            fgets(path, line_length, f_r);
            path[line_length-1] = '\0';
            int current_coord = 0;
            char* temp_token = strtok(path, "\t");
            char* temp_path = strstr(path," ");
            int iters = 0;
            while(iters<space_count){
                val = atof(temp_token);//1000.0;
                points_array[current_coord] = val;
                printf("%f\t", points_array[current_coord]);
                current_coord++;
                temp_token = strtok(temp_path+1, "\t");
                printf("%s THIS IS TEMP TOK\n" , temp_token);
            	temp_path = strstr(temp_path+1," ");
                printf("%s THIS IS TEMP PATH\n" , temp_path);
                iters++;
            }
            printf("%f\n", points_array[current_coord]);
            points_array[current_coord] = atof(temp_token);
            printf("%f\n", points_array[current_coord]);
            printf("%s IM HERE AFTER TOK\n", temp_token);
            points_len = current_coord;
            return points_array;
        }else{
            for(;;){
                counter_to_path_start += 1;
                c = fgetc(f_r);
                if( c == EOF || c == '\n' )
                    break;
            }
            path_number+=1;
        }
        if(c==EOF){
        	break;
        }
    }
    return NULL;
}

void update_stopPoint(){
    stop_point[0] = current_point[0];
    stop_point[1] = current_point[1];
}

void init_pid(){
    current_error = 0;
    accumulated_error = 0;
    prev_error = 0;
    pid_flag = 0;
}

void update_points(){
    dist_required = sqrt(pow(points[point_index*2+1] - prev_point[1],2)+pow(points[point_index*2] - prev_point[0],2));
    if (points[point_index*2]-prev_point[0] >= 0){
        angle_required = atan((points[point_index*2+1]-prev_point[1])/(points[point_index*2]-prev_point[0]))*180/M_PI;
    }
    else{
        angle_required = -atan((points[point_index*2]-prev_point[0])/(points[point_index*2+1]-prev_point[1]))*180/M_PI;
        if(points[point_index*2+1]-prev_point[1] >= 0)
            angle_required = angle_required + 90;
        else
            angle_required = angle_required - 90;
    }
    prev_point[0] = points[point_index*2];
    prev_point[1] = points[point_index*2+1];
    point_index  = point_index + 1;
    if(point_index >= (points_len+1)/2){
        point_index = (points_len+1)/2-1;
        return;
    }
    dist_traversed = 0;
    init_pid();

}

double get_PID_angle(double current_val, double target_val){
    current_error = target_val - current_val;
    accumulated_error = accumulated_error + current_error*timeDiff;
    double val = 0;
    if(timeDiff != 0)
        val = 0.1*current_error + 0.0005*accumulated_error + 0.001*(current_error-prev_error)/timeDiff;
    else if(timeDiff==0)
        val = 0.1*current_error + 0.0005*accumulated_error;
    if(val>5)
        val = 5;
    else if(val<-5)
        val = -5;
    printf("VALROT: %.17g", val );
    if(val<0.005&&val>-0.005){
        pid_flag = 1;
    }
    prev_error = current_error;
    return val;
}

double get_PID_dist(double current_val, double target_val){
    current_error = target_val-current_val;
    accumulated_error = accumulated_error + current_error*timeDiff; 
    double val = 500*current_error + 0.1*accumulated_error;
    if(val > 5)
        val = 5;
    else if(val < -5)
        val = -5;
    if (val < 0.1&&val>-0.1)
        pid_flag = 1;
    prev_error = current_error;
    return val;
}

void normal_motion(){
    if(rotating_flag==1){
        double val = get_PID_angle(angle_rotated, angle_required);
        if(pid_flag==1){
            init_pid();
            stop();
            rotating_flag = 0;
        }else{
            rotate(val);
        }
    }else{
        double val = get_PID_dist(dist_traversed, dist_required);
        printf("VAL: %.17g\n", val);
        if(pid_flag==1){
            init_pid();
            stop();
            update_points();
            update_stopPoint();
            rotating_flag = 1;
        }else{
            forward(val);
        }
    }
}

void recalculate(){
    dist_required = sqrt(pow(prev_point[1]-current_point[1], 2) + pow(prev_point[0]-current_point[0], 2));    
    if(prev_point[0]-current_point[0] >= 0){
        angle_required = atan((prev_point[1]-current_point[1])/(prev_point[0]-current_point[0]))*180/M_PI;
    }
    else{
        angle_required = -atan((prev_point[0]-current_point[0])/(prev_point[1]-current_point[1]))*180/M_PI;
        if(prev_point[1]-current_point[1] >= 0)
            angle_required = angle_required + 90;
        else 
            angle_required = angle_required - 90;
    }
    update_stopPoint();
    dist_traversed = 0;
    init_pid();
    rotating_flag = 1;
}

void sensing_loop(FILE* f_sensing){
	int path_number = 0;
	//int n = 3;
    int line_length = 0;
    double val = 0;
	int counter_to_path_start=0;
    int space_count = 0;
	char c = '\0';
	while(1){
	    c = fgetc(f_sensing);
	    printf("%c", c);
	    if(c == ' '){
	        space_count+=1;
	    }
	    if( c == EOF || c == '\n' ){
	    	if(c == EOF){
	    		run = 1;
	    	}
	        ++line_length;
	        break;
	    }
	    ++line_length;
	}
	fseek(f_sensing, -(line_length+1), SEEK_CUR);
	//c = fgetc(f_sensing);
	char* path = (char *)calloc(line_length, sizeof(char));
    //strcpy(path, "\0");
    double* points_array = (double *)calloc((space_count+1), sizeof(double));
    fgets(path, line_length, f_sensing);
    c = fgetc(f_sensing);
    path[line_length-1] = '\0';
    path[line_length-1] = '\0';
    int current_coord = 0;
    //char temp_string[1] = {'\t'};
    char* temp_token = strtok(path, "\t");
    char* temp_path = strstr(path, " ");
    int iters = 0;
    while(iters<space_count){
        val = atof(temp_token);//1000.0;
        points_array[current_coord] = val;
        //printf("%.17g \t", points_array[current_coord]);
        current_coord++;
        temp_token = strtok(temp_path+1, "\t");
    	temp_path = strstr(temp_path+1," ");
        iters++;
    }
    points_array[current_coord] = atof(temp_token);
    // for(int i=0;i<space_count+1; i++){
    // 	printf("%.17g THESE ARE THE VALUES READ\n", points_array[i]);
    // }
    verify_dist_traversed = points_array[0];
    if(verify_dist_traversed==0.2274984320097){
        verify_angle_rotated = points_array[1];
        verify_angle_rotated = points_array[1];
        verify_current_point[0] = points_array[2];
        verify_current_point[1] = points_array[3];
        timeDiff = points_array[4];
        prev_time = points_array[5];
        lin_vel = points_array[6];
        ang_vel = points_array[7];
    }else{
        verify_angle_rotated = points_array[1];
        verify_current_point[0] = points_array[2];
        verify_current_point[1] = points_array[3];
        timeOfActuation = points_array[4];
        timeDiff = points_array[5];
        prev_time = points_array[6];
        lin_vel = points_array[7];
        ang_vel = points_array[8];
    }
    for(int i=0;i<5;i++){
    	obstacle_flag[i] = 0;
    }
    for(int i=0;i<5;i++){
        double res;
        double distance;
        if(verify_dist_traversed==0.2274984320097){
    	    res = points_array[8+2*i];
    	    distance = points_array[8+2*i+1];
        }else{
            res = points_array[9+2*i];
    	    distance = points_array[9+2*i+1];
        }
    	if(prev_res[i]>0&&prev_distance[i]<minDist&&prev_distance[i]!=-1){
    		proxSensDist[i] = prev_distance[i];
    		obstacle_flag[i] = 1;
    		if(i==1||i==2||i==3)
    			detect_flag = 1;
    	}
        prev_res[i] = res;
        prev_distance[i] = distance;
    }
}


void actuation(){
    test_var = atan(1)*180/M_PI;
	dist_traversed = dist_traversed + lin_vel*timeDiff;
	angle_rotated = angle_rotated + ang_vel*timeDiff*180.0/M_PI;
    prev_timeDiff = timeDiff;
    prev_ang_vel = ang_vel;
    prev_lin_vel = lin_vel;
	if(dist_traversed-verify_dist_traversed<1e-9&&dist_traversed-verify_dist_traversed>(-1e-9)){
		printf("MATCHED\n");
	}else{
        printf("DISTANCE NOT MATCHING %.17g %.17g\n", dist_traversed, verify_dist_traversed);
	}
	if(abs(angle_rotated-verify_angle_rotated)>1e-9){
		printf("ANGLE NOT MATCHING %.17g %.17g\n", angle_rotated, verify_angle_rotated );
	}else{
		printf("MATCHED\n");
	}
    printf("%.17g \n", dist_traversed-verify_dist_traversed);
	current_point[0] = stop_point[0] + dist_traversed*cos(angle_rotated*(M_PI/180.0)); //calculates co-ordinates of the current point using the point where the orientation of the bot was last changed

    current_point[1] = stop_point[1] + dist_traversed*sin(angle_rotated*(M_PI/180.0));
    if(abs(current_point[0]-verify_current_point[0])>1e-9){
		printf("POINT 0 NOT MATCHING %.17g %.17g\n", current_point[0], verify_current_point[0]);
	}else{
		printf("MATCHED\n");
	}
	if(abs(current_point[1]-verify_current_point[1])>1e-9){
		printf("POINT 1 NOT MATCHING %.17g %.17g\n",  current_point[1], verify_current_point[1]);
	}else{
		printf("MATCHED\n");
	}
    if(verify_dist_traversed==0.19978958480504){
        printf("YAY\n");
    }
    if(verify_dist_traversed==2.3485447120809){
        printf("YAY\n");
    }
    if(verify_dist_traversed==2.3472252431663){
        printf("YAY\n");
    }
    if(prev_time==16.85000038147){
        printf("YAY\n");
    }
    if((prev_time>=time_flag+0.5)&&(prev_time<time_flag+1))
    	forward(1);
    if(detect_flag==0&&prev_time>=time_flag+1){
    	normal_motion();
    }else if(detect_flag==1&&prev_time>=time_flag+1){
    	if(obstacle_flag[1]==1&&obstacle_flag[3]==0){
	    	time_flag = prev_time;
	    	rotate(-1);
	    	dist_traversed = 0;
	    	update_stopPoint();
    	}else if(obstacle_flag[3]==1&&obstacle_flag[1]==0){
    		time_flag = prev_time;
    		rotate(1);
    		dist_traversed = 0;
	    	update_stopPoint();
    	}else if(obstacle_flag[1]==1&&obstacle_flag[3]==1){
    		time_flag = prev_time;
    		rotate(1);
    		dist_traversed = 0;
	    	update_stopPoint();
    	}else if(obstacle_flag[1]==0&&obstacle_flag[3]==0&&obstacle_flag[2]==1){
    		time_flag = prev_time;
    		rotate(1);
            dist_traversed = 0;
            update_stopPoint();
        }else if(obstacle_flag[0]==1||obstacle_flag[4]==1){
            time_flag = prev_time;
            forward(5);

    	}else if(obstacle_flag[0]==0&&obstacle_flag[4]==0){
    		recalculate();
    		detect_flag = 0;
    	}
    }
}

int main(){
	FILE* f_sensing = fopen("./sensor_data.txt", "r");
	points = create_coord_array(4);
	printf("%d COORDINATES GENERATED\n", run);
    update_points();
	while(run==0){
		printf("INSIDE RUN\n");
		actuation();
		sensing_loop(f_sensing);
	}
	//double* a = create_coord_array(3);
}

