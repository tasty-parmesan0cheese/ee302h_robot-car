/*

A0, A1 : photo resistors
A2, A3, A4 : IR sensors
D7, D8 : digital output for motors

*/

#define LINE_THRESHOLD 500 		//the threshold to determine white or black
#define RED_THRESHOLD 700 		//the threshold to determine white or red
#define PHOTORES_L 0			//which analog inputs we use for photo resistors
#define PHOTORES_R 1
#define IR_L 3					//which analog inputs we use for IR sensors
#define IR_MID 4
#define IR_R 5
#define OUT_L 7 				// This 2 lines defines which 2 digital ports we use for output to H bridge
#define OUT_R 8 				// by only changing this number, the program will adjust everywhere
#define STOP_DISTANCE 100			// the distance at which the robot will stop when it is about to touch the wall

typedef enum{					//defines a new variable-type 'DIRECTION', which can contain only 4 values: FORWARD, LEFT, RIGHT, STOP
	FORWARD, LEFT, RIGHT, HALT
}DIRECTION, *PDIRECTION;	

typedef enum{
	WHITE, BLACK
}COLOR, *PCOLOR;				//defines a new variable-type 'COLOR', which can contain only 2 values: WHITE and BLACK

typedef enum{
	ROTATE, STOP
}MOTOR_STATUS, *PMOTOR_STATUS;

typedef enum{
	LINETRACE, WALL_STOP, WALLTRACE, END
}PHASE, *PPHASE;			

int line_sensor[2]; 			//to put photo resistor values.line_sensor[0] = left, line_sensor[1] = right
COLOR color[2]; 
const int threshold = LINE_THRESHOLD;
const int stop_distance = STOP_DISTANCE;

int ir_sensor[3];				//to put IR sensor values. [0] = left, [1] = middle. [2] = right

DIRECTION direction;			//create a spefic variable 'direction', which has the type 'DIRECTION'
MOTOR_STATUS motorL, motorR;	//status of each motor. Each of them can contain either 'ROTATE' or 'STOP'
PHASE phase;					//the variable 'phase', which has the type 'PHASE' will contain the current phase of robot

void update_line_sensors(void){

	line_sensor[0] = analogRead(PHOTORES_L);
	line_sensor[1] = analogRead(PHOTORES_R);

	for(int i = 0; i < 2; i++){
		if(line_sensor[i] > threshold){
			color[i] = WHITE;
		}else{
			color[i] = BLACK;
		}
	}
	return;
}

void update_wall_sensors(void){

	ir_sensor[0] = analogRead(IR_L);
	ir_sensor[1] = analogRead(IR_MID);
	ir_sensor[2] = analogRead(IR_R);
	return;
}

void decide_direction_linetrace(void){

	if(color[0] == BLACK){	//if left photoresistor is on the line, go slightly left
		direction = LEFT;
		return;
	}else if(color[1] == BLACK){	//if right photoresistor is on the line, go slightly left
		direction = RIGHT;
		return;
	}else{
		direction = FORWARD;
	}
	return;
}

void decide_direction_walltrace(void){

	if(ir_sensor[0] > ir_sensor[2]){
		direction = LEFT;				// if right side of the robot is closer to the wall, turn slightly left
	}else if(ir_sensor[2] > ir_sensor[0]){
		direction = RIGHT;				//  if left side of the robot is closer to the wall, turn slightly right
	}else{
		direction = FORWARD;
	}
	return;

}

void update_motor_status(void){

	if(direction == FORWARD){
		motorL = ROTATE;
		motorR = ROTATE;
	}
	if(direction == RIGHT){
		motorL = ROTATE;
		motorR = STOP;
	}
	if(direction == LEFT){
		motorL = STOP;
		motorR = ROTATE;
	}
	if(direction == HALT){
		motorL = STOP;
		motorR = STOP;
	}
	return;
}

void pwm(void){							// pulse width modulation // we won't use this, unless we want to change the speed of motor

	static int i = 0; 					// a 'static' local variable keeps its value until the next time this function is called 
	if(i == 0){
		if(motorL == ROTATE){
			digitalWrite(OUT_L, HIGH);
		}
		if(motorR == ROTATE){
			digitalWrite(OUT_R, HIGH);
		}
	}
	if(i == 1){
		if(motorL == ROTATE){
			digitalWrite(OUT_L, LOW);
		}
		if(motorR == ROTATE){
			digitalWrite(OUT_R, LOW);
		}
	}
	i = !i;
	delay(0.5); 							//wait 500 microseconds here
	return;
}

void operate_motor(void){

	if(motorL == ROTATE){
		digitalWrite(OUT_L, LOW);		//putting a LOW voltage to the H bridge rotates the motor
	}else{
		digitalWrite(OUT_L, HIGH);		//putting a HIGH voltage to the H bridge stops the motor
	}
	if(motorR == ROTATE){
		digitalWrite(OUT_R, LOW);
	}else{
		digitalWrite(OUT_R, HIGH);
	}
	return;
}

void check_wall(void){

	ir_sensor[1] = analogRead(IR_MID);	//get the value of middle IR sensor
	if(ir_sensor[1] <= stop_distance){
		phase = WALL_STOP;
	}
	return;
}

void check_red_tape(void){

	update_line_sensors();
	if(line_sensor[0] <= RED_THRESHOLD || line_sensor[1] <= RED_THRESHOLD){
		direction = HALT;
		update_motor_status();
		operate_motor();
		phase = END;
	}
	return;
}

void wait_until_TA_removes_wall(void){

	while(ir_sensor[1] <= STOP_DISTANCE){
		direction = HALT;
		update_motor_status();
		operate_motor();
		update_wall_sensors();			//until the wall is removed, the robot won't move
	}									
	phase = WALLTRACE;
	direction = FORWARD;
	return;
}

void setup(){
	pinMode(OUT_L, OUTPUT);
	pinMode(OUT_R, OUTPUT);
	phase = LINETRACE;					//at first, the phase is LINETRACE
}

void loop(){

	switch(phase){						//depending on the value of variable 'phase', change the action
		case LINETRACE:
			update_line_sensors();
			decide_direction_linetrace();
			update_motor_status();
			operate_motor();
			check_wall();
			break;
		case WALL_STOP:
			wait_until_TA_removes_wall();
			break;
		case WALLTRACE:
			update_wall_sensors();
			decide_direction_walltrace();
			update_motor_status();
			operate_motor();
			check_red_tape();
			break;
		case END:
			break;
	}

}

