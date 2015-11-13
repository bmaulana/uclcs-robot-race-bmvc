#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "picomms.h"

const int PHASE1_SPEED = 30, PHASE1_SLOW = 5, PHASE2_SPEED = 127, SQUARE_DIST = 60, RIGHT_ANGLE_TURN_ENC = 210, MAX_DIST_TO_WALL = 40, PATH_MAX_LENGTH = 63, START_OFFSET = 15, MAP_SIZE = 8000, FRONT_WALL_MIN_DIST = 20;
const double CM_TO_ENCODER = (1 / (9.5 * M_PI)) * 360, WIDTH = 22.5, RADIANS_TO_DEGREES = 180 / M_PI, PHASE2_STEER = 1.2, PHASE2_STOP = 20.0, PHASE2_GOAL = 30.0;
//cm to encoder = based on diameter of wheel

int leftprev, rightprev;
double xpos = 0.0, ypos = 0.0, bearing = 0.0;

void positioncalc()
{
	//calculates robot position in each step/call.

	int leftenc, rightenc;
	get_motor_encoders(&leftenc, &rightenc);

	//gets change in left encoder and right encoder readings, convert to cm, and calculate angle turned by robot
	double leftdiff = (leftenc - leftprev) / CM_TO_ENCODER;
	double rightdiff = (rightenc - rightprev) / CM_TO_ENCODER;
	double anglediff = (leftdiff - rightdiff) / WIDTH;

	//saves current encoder values to global variables for next function call.
	leftprev = leftenc;
	rightprev = rightenc;

	//calculate change in x- and y- position.
	double deltax = 0.0, deltay = 0.0;	
	if(anglediff != 0) 
	{
		double radiusl = leftdiff / anglediff;
		double radiusr = rightdiff / anglediff;
		double radius = (radiusl + radiusr) / 2;

		deltax = (radius * cos(bearing)) - (radius * cos(bearing + anglediff));
		deltay = (radius * sin(bearing + anglediff)) - (radius * sin(bearing));
	}
	else if(anglediff == 0)
	{
		double dist = (leftdiff + rightdiff) / 2;
		deltax = dist * sin(bearing);
		deltay = dist * cos(bearing);
	}

	//saves current position and bearing to global variables.
	bearing += anglediff;
	if(bearing < 0) { bearing += 2 * M_PI; }
	else if(bearing > 2 * M_PI) { bearing -= 2 * M_PI; }
	xpos += deltax;
	ypos += deltay;

	//checks accurancy of the robot's calculated current position vs its real position
	//log_trail();
	//set_point(xpos, ypos);
}

double getDistance(double x1, double y1, double x2, double y2)
{
	//gets distance between two points
	double xDiff = x2 - x1;
	double yDiff = y2 - y1;
	return sqrt(xDiff * xDiff + yDiff * yDiff);
}

double getAngle(double x1, double y1, double x2, double y2) 
{
	//gets angle between two points and the vertical axis
	double xDiff = x2 - x1;
	double yDiff = y2 - y1;
	double angle_y = atan(xDiff / yDiff);
	if(yDiff < 0) { angle_y += M_PI; }
	if(angle_y < 0) { angle_y += 2 * M_PI; }
	return angle_y;
}

void moveForwards(int maxspd, int minspd, int dist) 
{
	//speeds up, moves the robot forwards a certain distance, and slows down.

	int left1, left2, right1, right2;
	get_motor_encoders(&left1, &right1);
	
	while(1)
	{
		get_motor_encoders(&left2, &right2);
		int dtravelled = ((left2 - left1) + (right2 - right1)) / 2;

		if(dtravelled < (maxspd - minspd) * 2)
		{
			//accelerate phase
			set_motors(minspd + dtravelled / 2, minspd + dtravelled / 2);
		} 
		else if(dtravelled < dist - (maxspd - minspd) * 2) 
		{
			//max speed phase
			set_motors(maxspd, maxspd);
		} 
		else if(dtravelled < dist)
		{
			//decelerate phase
			set_motors(minspd + (dist - dtravelled) / 2, minspd + (dist - dtravelled) / 2);
		}
		else
		{
			break;
		}
		positioncalc();

		int ultrasound = get_us_dist();
		if(ultrasound < FRONT_WALL_MIN_DIST) { break; }
	}
	set_motors(0, 0);
}

void moveBackwards(int maxspd, int minspd, int dist) 
{
	//speeds up, moves the robot backwards a certain distance, and slows down.

	int left1, left2, right1, right2;
	get_motor_encoders(&left1, &right1);
	
	while(1)
	{
		get_motor_encoders(&left2, &right2);
		int dtravelled = ((left1 - left2) + (right1 - right2)) / 2;

		if(dtravelled < (maxspd - minspd) * 2)
		{
			//accelerate phase
			set_motors(-(minspd + dtravelled / 2), -(minspd + dtravelled / 2));
		} 
		else if(dtravelled < dist - (maxspd - minspd) * 2) 
		{
			//max speed phase
			set_motors(-maxspd, -maxspd);
		} 
		else if(dtravelled < dist)
		{
			//decelerate phase
			set_motors(-(minspd + (dist - dtravelled) / 2), -(minspd + (dist - dtravelled) / 2));
		}
		else
		{
			break;
		}
		positioncalc();
	}
	set_motors(0, 0);
}

void turnRight(int maxspd, int minspd, int degrees) 
{
	//causes robot to spin right to a certain angle.

	int left1, left2, right1, right2, drotated = 0;
	get_motor_encoders(&left1, &right1);
	int turnLength = RIGHT_ANGLE_TURN_ENC * degrees / 90;
	
	while(drotated < turnLength) 
	{
		get_motor_encoders(&left2, &right2);
		drotated = ((left2 - left1) + (right1 - right2)) / 2;

		if(drotated < (maxspd - minspd) * 2)
		{
			//accelerate phase
			set_motors(minspd + drotated / 2, -(minspd + drotated / 2));
		} 
		else if(drotated < turnLength - (maxspd - minspd) * 2) 
		{
			//max speed phase
			set_motors(maxspd, -maxspd);
		} 
		else
		{
			//decelerate phase
			set_motors(minspd + (turnLength - drotated) / 2, -(minspd + (turnLength - drotated) / 2));
		}
		positioncalc();
	}
	set_motors(0, 0);
}

void turnLeft(int maxspd, int minspd, int degrees) 
{
	//causes robot to spin left to a certain angle.

	int left1, left2, right1, right2, drotated = 0;
	get_motor_encoders(&left1, &right1);
	int turnLength = RIGHT_ANGLE_TURN_ENC * degrees / 90;
	
	while(drotated < turnLength) 
	{
		get_motor_encoders(&left2, &right2);
		drotated = ((left1 - left2) + (right2 - right1)) / 2;

		if(drotated < (maxspd - minspd) * 2)
		{
			//accelerate phase
			set_motors(-(minspd + drotated / 2), minspd + drotated / 2);
		} 
		else if(drotated < turnLength - (maxspd - minspd) * 2) 
		{
			//max speed phase
			set_motors(-maxspd, maxspd);
		} 
		else
		{
			//decelerate phase
			set_motors(-(minspd + (turnLength - drotated) / 2), minspd + (turnLength - drotated) / 2);
		}
		positioncalc();
	}
	set_motors(0, 0);
}

void print_square_centres() 
{
	//prints the centre of each square to the simulator.
	int x = 0;
	while(x <= SQUARE_DIST * 3)
	{
		int y = SQUARE_DIST - START_OFFSET;
		while(y <= SQUARE_DIST * 4 - START_OFFSET)
		{
			set_point(x, y);
			y += SQUARE_DIST;
		}
		x += SQUARE_DIST;
	}
}

void reverseStr(char str[])
{
	//reverses a string, because strrev() does not work with cygwin for some reason
    int length = strlen(str);
    int i, j;
    char c;

    for (i = 0, j = length - 1; i < j; i++, j--)
    {
        c = str[i];
        str[i] = str[j];
        str[j] = c;
    }
}

void reversePathDirection(char str[]) 
{
	//takes the path from the finish point back to the starting point, and transforms it to become a path for the other way around

	//adds an 'S' to the end of the string
	int length = strlen(str);
	str[length] = 'S';
	str[length+1] = '\0';

	reverseStr(str);

	int i = 0;
	while(str[i] != '\0')
	{
		//swap 'R's with 'L's, since left/right is reversed when tracking back
		if(str[i] == 'R') { str[i] = 'L'; }
		else if(str[i] == 'L') { str[i] = 'R'; }
		i++;
	}

	str[strlen(str) - 1] = '\0'; //remove the 'B' at the end
}

void shortenPath(char str[])
{
	//takes a path from start to finish and removes 'turn back/turn around' movements to make the path shorter

	int i = 0;
	while(str[i] != '\0')
	{
		if(str[i] == 'B')
		{
			if(str[i-1] == 'L' && str[i+1] == 'L')
			{
				//replace LBL with S
				str[i-1] = 'S';
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}

			else if(str[i-1] == 'L' && str[i+1] == 'S')
			{
				//replace LBS with R
				str[i-1] = 'R';
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}

			else if(str[i-1] == 'L' && str[i+1] == 'R')
			{
				//replace LBR with B
				str[i-1] = 'B'; 
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}

			else if(str[i-1] == 'S' && str[i+1] == 'L')
			{
				//replace SBL with R
				str[i-1] = 'R';
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}

			else if(str[i-1] == 'S' && str[i+1] == 'S')
			{
				//replace SBS with B
				str[i-1] = 'B'; 
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}

			else if(str[i-1] == 'S' && str[i+1] == 'R')
			{
				//replace SBR with L
				str[i-1] = 'L'; 
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}

			else if(str[i-1] == 'R' && str[i+1] == 'L')
			{
				//replace RBL with B
				str[i-1] = 'B'; 
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}

			else if(str[i-1] == 'R' && str[i+1] == 'S')
			{
				//replace RBS with L
				str[i-1] = 'L'; 
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}

			else if(str[i-1] == 'R' && str[i+1] == 'R')
			{
				//replace RBR with S
				str[i-1] = 'S';
				int j, length = strlen(str);
				for(j = i; j <= length-2; j++) { str[j] = str[j+2]; }
			}
			i--; //go to previous index so that if result of replacement is another B, it is also replaced.
		}
		else { i++; }
	}
}

void fixBearing(int idealBearing) 
{
	//spins the robot slightly leftwards/rightwards to its ideal bearing

	float targetBearing = idealBearing / RADIANS_TO_DEGREES;	
	if(bearing > 6) { bearing -= 2 * M_PI; } //set bearing to negative (temporarily) when moving left-of-upwards so that it can be compared to 0

	if(bearing < targetBearing) 
	{
		float bearingdiff = targetBearing - bearing;
		turnRight(PHASE1_SPEED, PHASE1_SLOW, bearingdiff * RADIANS_TO_DEGREES);
	}
	else if(bearing > targetBearing)
	{
		float bearingdiff = bearing - targetBearing;
		turnLeft(PHASE1_SPEED, PHASE1_SLOW, bearingdiff * RADIANS_TO_DEGREES);
	}
}

int getIdealBearing() 
{
	//gets the robot's current bearing in degrees rounded to the nearest 90.

	int degBearing = bearing * RADIANS_TO_DEGREES;

	if(degBearing < 45 || degBearing >= 315) { return 0; }
	if(degBearing >= 45 && degBearing < 135) { return 90; }
	if(degBearing >= 135 && degBearing < 225) { return 180; }
	if(degBearing >= 225 && degBearing < 315) { return 270; }

	//should be unreachable
	printf("Something wrong with the bearing/code");
	return 0;
}

char* getPath(int startx, int starty, int destx, int desty)
{
	//moves the robot from a square to another (in the grid where start = (0, 0)), following left wall, and records the path taken.

	int left, right, index = 0, currentx = startx, currenty = starty; //stores current position of robot in the grid in currentx and currenty 
	int idealBearing = getIdealBearing(); 
	char* path = malloc(sizeof(char) * PATH_MAX_LENGTH);

	while (currentx != destx || currenty != desty)
	{
		get_front_ir_dists(&left, &right);
		int front = get_us_dist();

		/*if(front < MAX_DIST_TO_WALL)
		{
			checkFrontWall(front);
		}*/

		//checks accurancy of the robot's calculated current position vs its real position
		log_trail();
		set_point(xpos, ypos);

		if(left > MAX_DIST_TO_WALL) 
		{
			//turn left
			turnLeft(PHASE1_SPEED, PHASE1_SLOW, 90);
			idealBearing -= 90;
			if(idealBearing < 0) { idealBearing += 360; }
			path[index] = 'L';
		}
		else if(front > MAX_DIST_TO_WALL)
		{
			//go straight
			path[index] = 'S';
		}
		else if(right > MAX_DIST_TO_WALL)
		{
			//turn right
			turnRight(PHASE1_SPEED, PHASE1_SLOW, 90);
			idealBearing += 90;
			if(idealBearing >= 360) { idealBearing -= 360; }
			path[index] = 'R';
		}
		else
		{
			//turn around
			turnRight(PHASE1_SPEED, PHASE1_SLOW, 180);
			idealBearing += 180;
			if(idealBearing >= 360) { idealBearing -= 360; }
			path[index] = 'B';
		}

		fixBearing(idealBearing);
		moveForwards(PHASE1_SPEED, PHASE1_SLOW, SQUARE_DIST * CM_TO_ENCODER);

		//uses robot's idealBearing to figure out which square in the grid the robot is currently in after moving 
		switch(idealBearing)
		{
			case 0: currenty++; break;
			case 90: currentx++; break;
			case 180: currenty--; break;
			case 270: currentx--; break;
			default: printf("Error: unexpected value for idealBearing\n"); break;
		}

		index++;
	}
	path[index] = '\0';
	return path;
}

void buildMap(double map[MAP_SIZE][2], char* path) 
{
	//builds a map for the robot to follow in Phase 2 using the path generated in Phase 1.

	int xcor = 0, ycor = 0, index = 0, i = 0;

	//map movement to first square
	for(i = 0; i <= (SQUARE_DIST - START_OFFSET); i++) 
	{
		map[index][0] = xcor;
		map[index][1] = ycor;
		ycor++;
		index++;
	}

	//map rest of path
	int direction = 0, pathindex = 1;
	while(path[pathindex] != '\0')
	{
		if(path[pathindex] == 'L') 
		{
			direction -= 90;
			if(direction < 0) { direction += 360; }
		}
		if(path[pathindex] == 'R') 
		{
			direction += 90;
			if(direction < 0) { direction += 360; }
		}
		
		for(i = 0; i < SQUARE_DIST; i++) 
		{
			if(direction == 0) { ycor++; }
			if(direction == 90) { xcor++; }
			if(direction == 180) { ycor--; }
			if(direction == 270) { xcor--; }
			map[index][0] = xcor;
			map[index][1] = ycor;
			index++;
		}

		pathindex++;
	}
}

void printMap(double map[MAP_SIZE][2]) 
{
	//prints the built map in the simulator.

	int i = 1;
	while(map[i][0] != 0 || map[i][1] != 0) //when another [0,0] is reached, that means the trail has reached its end
	{
		set_point(map[i][0], map[i][1]);
		i++;
	}
}

void followMap(double map[MAP_SIZE][2])
{
	int lastindex = 0;
	while(getDistance(xpos, ypos, 3 * SQUARE_DIST, 4 * SQUARE_DIST - START_OFFSET) > PHASE2_STOP)
	{
		//selects target point - the first point in the map that is just above a certain distance away from the robot
		while((getDistance(xpos, ypos, map[lastindex][0], map[lastindex][1]) < PHASE2_GOAL) && (map[lastindex + 1][0] != 0 || map[lastindex + 1][1] != 0))
		{
			lastindex++;
		}

		double angle_to_point = getAngle(xpos, ypos, map[lastindex][0], map[lastindex][1]); //angle between the robot position, the target point, and the vertical axis 
		if(angle_to_point > 5.5) { angle_to_point -= 2 * M_PI; }
		if(bearing > 5.5) { bearing -= 2 * M_PI; }
		double angle_to_turn = angle_to_point - bearing; //angle needed for the robot to turn so that it heads towards targeted point

		int leftspd = PHASE2_SPEED, rightspd = PHASE2_SPEED;		
		if(angle_to_turn > 0) //targeted point is to the right of robot's current heading
		{
			//turn right
			rightspd = PHASE2_SPEED - (PHASE2_SPEED * angle_to_turn * PHASE2_STEER);
			if(rightspd < 0) { rightspd = 0; }
		} 
		else if(angle_to_turn < 0) //targeted point is to the left of robot's current heading
		{
			//turn left
			leftspd = PHASE2_SPEED + (PHASE2_SPEED * angle_to_turn * PHASE2_STEER);
			if(leftspd < 0) { leftspd = 0; }
		}
		set_motors(leftspd, rightspd);

		positioncalc();
		log_trail();
	}
}

void slowDown() 
{
	//code to slow down once the robot nears finish
	int i;
	for(i = PHASE2_STOP; i > 0; i--)
	{
		double slowspeed = PHASE2_SPEED * ((double) i / (double) PHASE2_STOP);
		set_motors(slowspeed, slowspeed);
		log_trail();
	}
}

void setup()
{
	connect_to_robot();
	initialize_robot();

	set_ir_angle(LEFT, -45);
	set_ir_angle(RIGHT, 45);

	set_origin();
	set_point(0, 0);
	print_square_centres();

	get_motor_encoders(&leftprev, &rightprev);
	moveBackwards(PHASE1_SPEED, PHASE1_SLOW, START_OFFSET * CM_TO_ENCODER); //move robot to centre of first square.
}

char* phase_one() 
{
	//start to finish (left wall path)
	char* path1 = getPath(0, 0, 3, 4);
	printf("Path to Finish: %s\n", path1);

	shortenPath(path1);
	printf("Shortened Path to Finish: %s\n", path1);

	//finish to start (right wall path)
	char* path2 = getPath(3, 4, 0, 0);
	reversePathDirection(path2);
	printf("Inversed Return Path: %s\n", path2);

	shortenPath(path2);
	printf("Shortened Inversed Return Path: %s\n", path2);

	//get and return the shortest path
	char* shortestPath = malloc(sizeof(char) * PATH_MAX_LENGTH);
	if(strlen(path1) <= strlen(path2)) { strcpy(shortestPath, path1); }
	else { strcpy(shortestPath, path2); }
	printf("Shortest Path: %s\n", shortestPath);
	return shortestPath;
}

void return_to_origin() 
{
	set_point(0, 0);

	//returns the robot to the origin position.
	turnRight(PHASE1_SPEED, PHASE1_SLOW, (2 * M_PI - bearing) * RADIANS_TO_DEGREES); //get the robot to face origin

	double dist_to_origin = getDistance(xpos, ypos, 0, 0);
	moveForwards(PHASE1_SPEED, PHASE1_SLOW, dist_to_origin * CM_TO_ENCODER); //move to origin

	fixBearing(0); //resets robot's bearing to face upwards
}

void phase_two(char* path) 
{
	//using shortest path, create map from start to finish
	double map[MAP_SIZE][2];
	buildMap(map, path);
	printMap(map);

	//Follow map to finish
	followMap(map);
	slowDown();
}

int main() 
{
	setup();
	
	char* shortestPath = phase_one();
	return_to_origin();
	phase_two(shortestPath);

	set_motors(0,0);
	printf("Finished!\n");
	return 0;
}