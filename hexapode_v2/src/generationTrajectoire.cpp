/**
* @file     generationTrajectoire.cpp
* @brief    ROS node to listen to keyboard command and move the hexapod 
* 			accordingly
* @details  
* @author   caesarhao@gmail.com
* @copy		Ecole des Mines de Nantes
* @date     2014-02-06
*/

/** Basic view of the hexapod
 * 
 * 
 * FRONT VIEW       ^       0==0             0==0
 *    |=======|     |       |  |---x[___]x---|  |
 *    |       |     Z       |                   |
 *
 *
 *
 * TOP VIEW
 *    0        3
 *    \       /     ^
 *     \_____/      |
 *     |     |      Y
 * 1---|     |---4   X--->
 *     |_____|
 *     /     \      
 *    /       \
 *   2         5
 */

#include <iostream>
#include <math.h> 
// ROS related include 
#include <ros/ros.h>
#include <hexapode_v2/translation.h>
#include <hexapode_v2/etat_robot.h>
#include <hexapode_v2/sequence_moteur.h>
#include "zhrobot.hpp"
#include <geometry_msgs/Twist.h>



//------------------------
// should be in an header file, but not sure how to use it in ROS
// some hints may in there
// http://answers.ros.org/question/31636/including-header-files-in-ros-build-system/
// http://wiki.ros.org/catkin/conceptual_overview


// Zero value for motors' encoder 
#define SERVO_ZERO	(600)
// Pi value for motor's encoder, ie our upper bound
#define SERVO_PI	(2400)
// Set of admissible value for our configuration
#define SERVO_CARDE	(SERVO_PI-SERVO_ZERO) //1800
// Center value for motor's encoder, we want to work around it 
#define SERVO_CENTRE	(1500)
// Maximun angle for function checkangle which is not used yet
#define MAX_ANGLE	(PI/4) // 0.78 rad, 45 deg 
// Maximun admissible displacement for any translation  during a timestep
#define MAX_X		(40.0) // mm
#define MAX_Y		(40.0)
#define MAX_Z		(40.0)
// Maximum angle of rotation of the robot around z_0 axis
#define MAX_A		(0.30) // 0.30 rad, 17 deg
// altitude of the end effector, end of one leg, when up final position is up
#define STEP_UP		(40.0) // mm

using namespace std;


using namespace zhrobot;

// Possible state of the robot's legs
typedef enum _etat{
	E_ROBOT_PARALLEL, // all the leg on the ground
	E_ROBOT_LEFT_UP, // legs 1,3,5 up
	E_ROBOT_RIGHT_UP // legs 0, 2, 4 up
}rbt_etat;

// Possible type of move for the robot
typedef enum _move_state{
	E_MOVE_UPDOWN, // along the z_0 axis
	E_MOVE_VERTICAL, // along the y_0 axis
	E_MOVE_HORIZONTAL, // along the x_0 axis
	E_MOVE_TOURNE // rotation around z_0
} move_etat;	

// Methods of generationTrajectoire 
void ecouteTranslation(const hexapode_v2::translation trans);
void  envoieCmd();
void ecouteEtatRobot(const hexapode_v2::etat_robot etat_robot);



/**
* @class	GeneTraj
* @brief   Class  describing the whole robot and it's moves 
*/
class GeneTraj
{ 
	private:
		int robotType; // to differenciate our two setup (0 small one) (1 big one with laser scanner)
		Robot rbs[6]; //array of 6 Legs, an hexapod
		dVector srcCoor; // cartesaian coordiante of the end of a leg in it's base
		dVector dstCoor[6];// current cartesian coordinate of all legs' end , in 0 frame
		dVector dstCoorStand[6]; // final cartesian coordinate of all legs' end , in 0 frame
		dVector vangles[6]; // array of motors' angle vangles[0]=[theta0, theta1,theta2,q3] is for leg[0] 
		rbt_etat retat; // state of the robot's leg
		move_etat metat; //type of movement for the robot
		char cmd[256]; // string to send to the low level motor board
		int vMoteur[6][3]; //current servo command for the motor board
		int vMoteurOld[6][3]; //previous servo command for the motor board
		double C; // in y = a*x^2 + b*x + C used to compute legs path
		double betaX, betaY, betaZ, betaA; // accumulated command value from the client, one can deduce the displacement along one axis by y=Y_MAX*sin(betaY)
		
				
	public:
		//getter
		char *getCmd();
	
		//methods

		void initRobot1();
		void initRobot2();  
		move_etat calcMoveType(double dx,double dy,double dh, double da); 
		void retablirAngles(); 
		int transAngletoNum(int patteNum,int motorNum,double angle); 
		void calcNextPosition(int robotNum); 
		void createTraj(double dBetaX, double dBetaY, double dBetaZ, double dBetaA); 
		void createCmd(); 
		// not used yet
		bool checkAngles(dVector angles); 
		
		
		
		// TODO should be private with getter and setter
		int first_loop; //to enter first loop in creatTraj 
		void transition();
		bool transition_ok ;
		bool new_move;
		double interpol;
		int  increment;
		bool navigation_on ;
 };

//-------------------------

// getter
/**
* @brief   fonction pour initialiser le grand robot.
* @param   void
* @return  void
*/
char* GeneTraj::getCmd(){
	return cmd;
}
// methods
/**
* @brief   Set the parameters for the small hexapod
* @param   void
* @return  void
*/
void GeneTraj::initRobot1(){
/**
 *
 *
 *               (2)____(3)
 *                |  a3  |
 * [0]______[1]___|d2    |a4
 *      a1      a2       |
 *                      (4)
*/
	robotType = 0;
	std::cout<<"Initialisation le petit robot"<<std::endl;
	Link ls[]={
		/**
		 * theta  rotation around z_i
		 * d distance along z_i
		 * a  x_i-1
		 * alpha  rotation around x_i-1
		 */
		//   theta,    d,		 a, 	     alpha
		Link(0,		0, 		88, 	0), // link1
		Link(0,	  -29,		28,		PI/2), // link2
		Link(0,		0,		77,		0), // link3
		Link(-PI/2,	0,		103,	 0) // link4
	};  
	
/*
		0	3

	1			4
	
		2	5
*/
	for (int i = 0; i < 6; i++){ // set the parametrization for each leg
		rbs[i].setLinks(ls, 4);
		vangles[i].resize(0.0, 4);
	}
	
	// Set the position of all leg around the hexapod
	// before that the legs are all set at the same position 
	//by simply rotating around z_0 and translating along the nex x axis
	// one can easily set all the legs orientation
	
	rbs[0].setQ(0, 2.05);
	rbs[0].getLink(0).setA(88);
	
	rbs[1].setQ(0, PI);
	rbs[1].getLink(0).setA(63);
	
	rbs[2].setQ(0, -2.05);
	rbs[2].getLink(0).setA(88);
	
	rbs[3].setQ(0, 1.09);
	rbs[3].getLink(0).setA(88);
	
	rbs[4].setQ(0, 0);
	rbs[4].getLink(0).setA(63);
	
	rbs[5].setQ(0, -1.09);
	rbs[5].getLink(0).setA(88);

	// set the coordinate of the end of the leg in it's base ie [0,0,0,1]'	
	srcCoor.resize(4, 0.0);
	srcCoor[3] = 1;

	// define the cartesian coordinate for the end effector in 0 frame
	for (int i = 0; i < 6; i++){
		// rbs[i].getH();
		dstCoor[i].resize(4, 0.0);
		dstCoorStand[i].resize(4, 0.0);
		// current and final coordinate are the same -> no move
		dstCoorStand[i] = dstCoor[i] = rbs[i].getDestCoord(srcCoor); 
	}
	
	//default state and move 
	retat = E_ROBOT_PARALLEL;
	metat = E_MOVE_UPDOWN;
	//the client input are null at hte beginning
	betaX = betaY = betaZ = betaA = 0;
	// set the constant to the value C=z_4+step_up expressed in 0-frame
	// ie C=step_up in 0-frame the maxiamle altitude of the end of one leg in 0 frame
	C = dstCoorStand[0][2]+STEP_UP;
    // set all the entry in vMoteurOld to zero
	memset(vMoteurOld, 0, sizeof(vMoteurOld));
}
/**
* @brief   set the parameters for the big hexapod
* @param   void
* @return  void
*/
void GeneTraj::initRobot2(){
/**
 *
 *
 *               (2)___(3)
 *                |  a3 |
 * [0]______[1]___|d2   |a4
 *      a1      a2      |
 *                     (4)
*/
	robotType = 1;
	std::cout<<"Initialization Big Hexapod"<<std::endl;

	Link ls[]={
		/**
		 * theta  rotation around z_i
		 * d distance along z_i
		 * a  x_i-1
		 * alpha  rotation around x_i-1
		 */
		//   theta, d,		 a, 	alpha
		Link(0,		0, 		136, 	0), // link1
		Link(0,		0,		30,		PI/2), // link2
		Link(0,		0,		57,		0), // link3
		Link(-PI/2,	0,		132,	 0) // link4
	};
	/*
			0	3

		1			4

			2	5
	*/
	for (int i = 0; i < 6; i++){// set the parametrization for each leg
		rbs[i].setLinks(ls, 4);
		vangles[i].resize(0.0, 4);
	}
	
	// Set the position of all leg around the hexapod
	// before that the legs are all set at the same position 
	// by simply rotating around z_0 
	// one can easily set all the legs orientation
	// /!\ in fact the right and the left of the robot don't have the same parametrization 
	//      some translation offsets may be differents
		
	rbs[0].setQ(0, 2*PI/3);
	rbs[1].setQ(0, PI);
	rbs[2].setQ(0, -2*PI/3);
	rbs[3].setQ(0, PI/3);
	rbs[4].setQ(0, 0);
	rbs[5].setQ(0, -PI/3);

	// set the coordinate of the end of the leg in it's base ie [0,0,0,1]'
	srcCoor.resize(4, 0.0);
	srcCoor[3] = 1;
	
    // define the cartesian coordinate for the end effector in 0 frame
	for (int i = 0; i < 6; i++){
		//rbs[i].getH();
		dstCoor[i].resize(4, 0.0);
		dstCoorStand[i].resize(4, 0.0);
		// current and final coordinate are the same -> no move
		dstCoorStand[i] = dstCoor[i] = rbs[i].getDestCoord(srcCoor);
	}
	
	//default state and move
	retat = E_ROBOT_PARALLEL;
	metat = E_MOVE_UPDOWN;
	//the client input are null at hte beginning
	betaX = betaY = betaZ = betaA = 0;
	// set the constant to the value C=z_4+step_up expressed in 0-frame
	// ie C=step_up in 0-frame the maxiamle altitude of the end of one leg in 0 frame	
	C = dstCoorStand[0][2]+STEP_UP;
    // set all the entry in vMoteurOld to zero	
	memset(vMoteurOld, 0, sizeof(vMoteurOld));
	//init vangles et vMoteur
	int i,j;
	for (i = 0; i<6;i++){		
		vangles[i] = rbs[i].invertCoord(dstCoor[i]); 
		for( j=0;j<3;j++){
		vMoteur[i][j] = transAngletoNum(i, j, vangles[i][1+j]);	
		}
	}
	navigation_on = false;
	transition_ok = false;
	new_move = false;
	first_loop=0;
	std::cout<<"End of initialization"<<std::endl;
}
/**
* @brief   Function to set the type of move
* @param   dx, dy, dh, da
* @return  void
*/
move_etat GeneTraj::calcMoveType(double dx, double dy, double dh, double da){
	if (abs(dx)> 0.001){ //translation along x_0
		return E_MOVE_HORIZONTAL;
	}
	else if (abs(dy)> 0.001){//translation along y_0
		return E_MOVE_VERTICAL;
	}
	else if (abs(dh) > 0.001){//translation along z_0
		return E_MOVE_UPDOWN;
	}
	else{ //rotation around z_0
		return E_MOVE_TOURNE;
	}
}
/**
* @brief   Reset all angles to 0
* @param   void
* @return  void
*/
void GeneTraj::retablirAngles(){
	int i ;
	for (i = 0; i < 6; i++){
		
		rbs[i].setQ(1, 0); // theta1=0
		rbs[i].setQ(2, 0); // theta2=0
		rbs[i].setQ(3, -PI/2); // q3=0
		
		//current and final coordinate are the same -> no move
		dstCoor[i] = rbs[i].getDestCoord(srcCoor); 
	}
	
	betaX = betaY = betaZ = betaA = 0;
	memset(vMoteurOld, 0, sizeof(vMoteurOld));
}
/**
* @brief   Compute the command for the low level board 
* @param   int patteNum, int motorNum, double angle
* @return  int command
*/
int GeneTraj::transAngletoNum(int patteNum, int motorNum, double angle){
	if (patteNum < 3) // left handside
	{
		switch(motorNum){
			case 0:
				return (int)(angle*SERVO_CARDE/PI + SERVO_CENTRE);
			case 1:
				if (robotType == 0){ //small hexapod
					return (int)(angle*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{ // big hexapod
					return (int)(-angle*SERVO_CARDE/PI + SERVO_CENTRE);
				}
			default: // 2
				if (robotType == 0){ //small hexapod
					return (int)(-(PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{ // big hexapod
					return (int)((PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
		}
	}
	else{ // right handside
		switch(motorNum){
			case 0:
				return (int)(angle*SERVO_CARDE/PI + SERVO_CENTRE);
			case 1:
				if (robotType == 0){ //small hexapod
					return (int)(-angle*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{
					return (int)(angle*SERVO_CARDE/PI + SERVO_CENTRE);
				}
			default: // 2
				if ( robotType ==0 ){ //small hexapod
					return (int)((PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{ // big hexapod
					return (int)(-(PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
		}	
	}
}
/**
* @brief   Compute the path to the goal position
* @param   int
* @return  void
*/
void GeneTraj::calcNextPosition(int robotNum){
	
	double a, x, y,factor;
	
	switch(metat){
		
		case E_MOVE_UPDOWN:
			
			dstCoor[robotNum][2] = dstCoorStand[robotNum][2] - MAX_Z*sin(betaZ);
			break;
			
		case E_MOVE_VERTICAL:
			// factor is 'a' coefficient in z = a*y^2 + C
			factor = -(STEP_UP/(MAX_Y*MAX_Y));
			// compute y at the current position from previous command, because we have no feedback from motors
			y = MAX_Y*cos(betaY);
			///cout << sy<<" "<<factor*cz*cz+C<<endl;
			if(new_move){ //if change of move type, initialize the small step's parameters
				interpol= 0;
				increment =0;
				new_move=false;
			}
			if(increment<161){ //small step
				if(sin(betaY) >0){ //y +
					interpol = (floor(increment/6)+1)*MAX_Y/27; 
					increment ++;
					if ((robotNum%2) != 0){ // 1 3 5
						dstCoor[robotNum][1] = dstCoorStand[robotNum][1]+interpol;
						dstCoor[robotNum][2] = factor*y*y+C;
					}
					else{ // 0 2 4
						dstCoor[robotNum][1] = dstCoorStand[robotNum][1]-interpol;
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2] ;
					}
				}
				else{ //y -
					interpol = (floor(increment/6)+1)*MAX_Y/27; 
					increment ++;
					if ((robotNum%2) != 0){ // 1 3 5
						dstCoor[robotNum][1] = dstCoorStand[robotNum][1]+interpol;
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2] ;
					}
					else{ // 0 2 4
						dstCoor[robotNum][1] = dstCoorStand[robotNum][1]-interpol;
						dstCoor[robotNum][2] = factor*y*y+C;
					}
				}		
			}
			else{ //normal step
				if (sin(betaY) > 0){//y +
					if ((robotNum%2) != 0){ // 1 3 5
						dstCoor[robotNum][1] = dstCoorStand[robotNum][1]-y;
						dstCoor[robotNum][2] = factor*y*y+C;
					}
					else{ // 0 2 4
						dstCoor[robotNum][1] = dstCoorStand[robotNum][1]+y;
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2] ;
					}
				}
				else{ // sin(betaY) < 0, y -
					if ((robotNum%2) != 0){ // 1 3 5
						dstCoor[robotNum][1] = dstCoorStand[robotNum][1]-y ;
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
					}
					else{ // 0 2 4 
						dstCoor[robotNum][1] = dstCoorStand[robotNum][1] +y;
						dstCoor[robotNum][2] = factor*y*y+C;
					}
				}
		    } 
			break;
			
		case E_MOVE_HORIZONTAL: 
			// factor is 'a' coefficient in z = a*x^2 + C
			factor = -(STEP_UP/(MAX_X*MAX_X));
			// compute x at the current position, because we have no feedback from motors
			x = MAX_X*cos(betaX);
			if(new_move){ //if change of move type, initialize the small step's parameters
				interpol= 0;
				increment =0;
				new_move=false;
			}
			if(increment<161){//small step
				if(sin(betaX)>0){ // x +
					interpol = (floor(increment/6)+1)*MAX_X/27;
					increment ++;
					if ((robotNum%2) != 0){ // 1 3 5
						dstCoor[robotNum][0] = dstCoorStand[robotNum][0]+interpol;
						dstCoor[robotNum][2] = factor*x*x+C;
					}
					else{ // 0 2 4
						dstCoor[robotNum][0] = dstCoorStand[robotNum][0]-interpol;
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2] ;
					}
				}
				else{// x -
					interpol = (floor(increment/6)+1)*MAX_X/27; 
					increment ++;
					if ((robotNum%2) != 0){ // 1 3 5
						dstCoor[robotNum][0] = dstCoorStand[robotNum][0]+interpol;
						dstCoor[robotNum][2] =dstCoorStand[robotNum][2] ; 
					}
					else{ // 0 2 4
						dstCoor[robotNum][0] = dstCoorStand[robotNum][0]-interpol;
						dstCoor[robotNum][2] =factor*x*x+C; 
					}				
				}
				
			}
			else{ // normal step
				if (sin(betaX) > 0){ // x +
					if ((robotNum%2) != 0){ // 1 3 5
						dstCoor[robotNum][0] = dstCoorStand[robotNum][0] -x;
						dstCoor[robotNum][2] = factor*(x*x)+C;
					}
					else{ // 0 2 4
						dstCoor[robotNum][0] = dstCoorStand[robotNum][0] + x;
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
					}
				}
				else{ // sin(betaX) < 0, x-
					if ((robotNum%2) != 0){ // 1 3 5
						dstCoor[robotNum][0] = dstCoorStand[robotNum][0] -x;
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
					}
					else{ // 0 2 4
						dstCoor[robotNum][0] = dstCoorStand[robotNum][0] +x;
						dstCoor[robotNum][2] = factor*(x*x)+C;
					}
				}
			}
			break;
			
		case E_MOVE_TOURNE: 
			// factor is the coefficient so that z = factor*a^2 + C
			factor = -(STEP_UP/(MAX_A*MAX_A));
			a = MAX_A*cos(betaA);
			// len = sqrt(x^2+y^2).
			double len = sqrt(dstCoorStand[robotNum][0]*dstCoorStand[robotNum][0] + 
				dstCoorStand[robotNum][1]*dstCoorStand[robotNum][1]);
			// get the angle for the physical direction of the leg wrt to the base
			double angleNorm = rbs[robotNum].getLink(0).getParaTheta();
			double angle;
			if(new_move){//if change of move type, initialize the small step's parameters
				interpol= 0;
				increment =0;
				new_move=false;
			}
			if(increment<161  ){ //small step
				if(sin(betaA)>0){ //rotation anti-trigo
					interpol = (floor(increment/6)+1)*MAX_A/27; 
					increment ++;
					if ((robotNum%2) != 0){ // 1 3 5
						angle = angleNorm + interpol;
						dstCoor[robotNum][0] = len*cos(angle);
						dstCoor[robotNum][1] = len*sin(angle);
						dstCoor[robotNum][2] = factor*a*a+C;
					}
					else{ // 0 2 4
						angle = angleNorm - interpol;
						dstCoor[robotNum][0] = len*cos(angle);
						dstCoor[robotNum][1] = len*sin(angle);
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2];	
					}
				}
				else{//rotation trigo
					interpol = (floor(increment/6)+1)*MAX_A/27;
					increment ++;
					if ((robotNum%2) != 0){ // 1 3 5
						angle = angleNorm + interpol;
						dstCoor[robotNum][0] = len*cos(angle);
						dstCoor[robotNum][1] = len*sin(angle);
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2];	
					}
					else{ // 0 2 4
						angle = angleNorm -interpol;
						dstCoor[robotNum][0] = len*cos(angle);
						dstCoor[robotNum][1] = len*sin(angle);
						dstCoor[robotNum][2] = factor*a*a+C;
					}	
				}	
		    }
			else{// normal step
				if (sin(betaA) > 0){
					if ((robotNum%2) != 0){ // 1 3 5
						angle = angleNorm - a;
						dstCoor[robotNum][0] = len*cos(angle);
						dstCoor[robotNum][1] = len*sin(angle);
						dstCoor[robotNum][2] = factor*(a*a)+C;
					}
					else{ // 0 2 4
						angle = angleNorm + a;
						dstCoor[robotNum][0] = len*cos(angle);
						dstCoor[robotNum][1] = len*sin(angle);
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
					}
				}
				else{
					if ((robotNum%2) != 0){ // 1 3 5
						angle = angleNorm - a;
						dstCoor[robotNum][0] = len*cos(angle);
						dstCoor[robotNum][1] = len*sin(angle);
						dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
					}
					else{ // 0 2 4
						angle = angleNorm + a;
						dstCoor[robotNum][0] = len*cos(angle);
						dstCoor[robotNum][1] = len*sin(angle);
						dstCoor[robotNum][2] = factor*(a*a)+C;
					}
				}
			}	
			break;
	}
}

/**
* @brief   transition between 2 type of move
* @param   
* @return  void
*/
// TODO PB with 0 1 2
void  GeneTraj::transition(){
	int i, j;
	int delta =50;
	bool temp = true ;
    // default starting angle position
	int moteurCible[3] ;
	moteurCible[0]=1500; //theta1
	moteurCible[1]=1500; //theta2
	moteurCible[2]=1500; //theta3 
	
	for (i = 0; i<6;i++){ // legs	
		for (j = 0; j < 3; j++){ // motors
			if(  moteurCible[j] -vMoteur[i][j] > 100 ){
				vMoteur[i][j] += delta;
				temp = false ;
				///cout<<"patte "<<i<<" moteur "<<j+1<< " + petit : "<<vMoteur[i][j] <<"  "<<moteurCible[j] <<endl;
			}			
			else if(moteurCible[j] -vMoteur[i][j]< - 100){
				vMoteur[i][j] -= delta;
				temp = false ;
				///cout<<"patte "<<i<<" moteur "<<j+1<<" + grand : "<<vMoteur[i][j] <<"  "<< moteurCible[j] <<endl;
			}
		}
	}
	if(temp){ //al motors close to targeted position
		transition_ok = true;
		cout<<"transition done"<<endl;
		new_move =true;
	}
		
}



/**
* @brief   
* @param   double, double, double, double
* @return  void
*/
void GeneTraj::createTraj(double dBetaX, double dBetaY, double dBetaZ, double dBetaA){
	int i = 0, j = 0;
	move_etat met = calcMoveType(dBetaX,dBetaY,dBetaZ, dBetaA);
	
	// if the type of movement changes or it's the first loop til the switch on
	if (met != metat || first_loop==0){	
		cout << "transition" <<endl;
		transition_ok=false;
		transition();
		if(transition_ok){ //before starting the new move reset all angles and inputs
			first_loop=1;
			retablirAngles();
			retat = E_ROBOT_PARALLEL;
			metat=met;
			new_move=true;
		}
	}
	
	// if the type of movement does not change
	else{ 
		switch (retat){
			
			case E_ROBOT_PARALLEL: // the robot is in parallel state		
				switch(metat){
					case E_MOVE_UPDOWN: // move along z
						betaZ += dBetaZ;
						break;
					case E_MOVE_HORIZONTAL: // move along x
						betaX += dBetaX;
						break;
					case E_MOVE_VERTICAL: //move along y
						betaY += dBetaY;
						break;
					case E_MOVE_TOURNE: // rotation around z
						betaA += dBetaA;
						break;
				}
				///cout << betaX<<" "<<betaY<<" "<<betaZ<<" "<<betaA<<endl;
				break;
				
			/*  NEVER USED	*/
			case E_ROBOT_LEFT_UP: // 0 2 4 
				///cout << "left_up"<< endl;
				//added to move the robot because with proper syntax it was not suppose to move
				betaX += dBetaX;
				betaY += dBetaY;
				betaA += dBetaA;			
				break; //added to have proper switch syntax
				
			/*  NEVER USED	*/
			case E_ROBOT_RIGHT_UP: // 1 3 5 
			// never accessed, because the robot state is always left_up
				///cout << "right_up"<< endl;
				betaX += dBetaX;
				betaY += dBetaY;
				betaA += dBetaA;
				break;	
		}	
		
		// Compute the commande value for the motors
		for (i = 0; i<6;i++){
			calcNextPosition(i); // calculer la position prochaine pour chaque patte.
			vangles[i] = rbs[i].invertCoord(dstCoor[i]); // inverse kinematic
			for (j = 0; j < 3; j++){
				vMoteur[i][j] = transAngletoNum(i, j, vangles[i][1+j]);	 // 1+j, because theta0 is fixed from the geometry of the robot
			}
		}
	}
}
/**
* @brief   set the command message for the low level board, according to the datasheet of the board
* @param   void
* @return  void
*/
void GeneTraj::createCmd(){
	cmd[0] = '\0'; // "flush" the string if not empty
	char temp[20];
	for (int i = 0; i<6; i++){
		for (int j = 0; j < 3; j++){
			if (i<3){ // first 3 legs
				sprintf(temp, "# %d P%d ", 4*i+j, vMoteur[i][j]);
			}
			else{ // last 3 legs
				sprintf(temp, "# %d P%d ", 4*(1+i)+j, vMoteur[i][j]);			
			}
			// if change in command is big enough stor it in the command string
			if (abs(vMoteur[i][j] - vMoteurOld[i][j]) > 5){
				strcat(cmd, temp);
			}
			vMoteurOld[i][j] = vMoteur[i][j];
		}
	}
	// end the command string for, according to the communication protocol
	strcat(cmd, "\r\n");
	
	// this command will be send by the function envoieCmd
 }	


/**
* @brief   NOT USED YET
* @param   vector of 4 angles
* @return  bool
*/
bool GeneTraj::checkAngles(dVector angles){
	if ((angles[1] >= MAX_ANGLE) || (angles[1]<= -MAX_ANGLE)){ //theta1
		return false;
	}
	else if((angles[2] >= MAX_ANGLE) || (angles[2]<= -MAX_ANGLE)){ //theta2
		return false;
	}
	else if(((angles[3]+PI/2) >= MAX_ANGLE) || ((angles[3]+PI/2) <= -MAX_ANGLE)){ //q3
		return false;
	}
	else{
		return true;
	}
}


//======================================================================

GeneTraj gen;
ros::ServiceClient *pclient = NULL;


/**
* @brief   Function triggered by the topic translation, 
* create the new move for all the legs and send the command to the motors
* @param   double, double, double, double
* @return  void
*/
void ecouteTranslation(const hexapode_v2::translation trans)
{	

	if (trans.dx == 99 && trans.dy == 99 && trans.dh == 99 && trans.da == 99 && gen.navigation_on){
		// respone to key a; autonomous navigation mode
		gen.navigation_on = false;
		cout<<"autonomous navigation stopped "<<endl;
	}	
	else if (trans.dx == 99 && trans.dy == 99 && trans.dh == 99 && trans.da == 99 && !gen.navigation_on){
		// respone to key a; autonomous navigation mode
		gen.navigation_on = true;
		cout<<"autonomous navigation started "<<endl;
	}

	else if(!gen.navigation_on){
		double factor = 10; // magic factor to slow down the moves

		ROS_DEBUG("dx: [%f]", trans.dx);
		ROS_DEBUG("dy: [%f]", trans.dy);
		ROS_DEBUG("da: [%f]", trans.da);
		ROS_DEBUG("dh: [%f]", trans.dh);
		
		///cout << "increments : "<<trans.dx << trans.dy << trans.dh << trans.da << endl;
		
		// use the Methods from the class GeneTraj to generate the move the hexapod
		gen.createTraj(trans.dx/factor, trans.dy/factor, trans.dh/factor, trans.da/factor);
		// create the command message, translation from angle -> motor ref
		gen.createCmd();
		//send the command to the motors' board
		envoieCmd();
	}
	else{
	cout<<"/!\\ Warning : you are in autonomous mode, switch to manual with 'a' "<<endl;
	}
}
/**
* @brief   Function triggered by the topic cmd_vel, from navigation package 
* create the new move for all the legs and send the command to the motors
* @param   double, double, double, double
* @return  void
*/
void ecouteNavigation(const geometry_msgs::Twist twist)
{
	if (gen.navigation_on){
		///cout<<"autonomous navigation !"<<endl;
		double dx= 0,dy=0,da=0;
		double pas = 1/15; // to have the same magnitude as commands from the keyboard
		//transaltion from velocities to increments
		//really nasty 
		if( twist.linear.x > 0){ dx =pas;}
		if( twist.linear.x < 0){ dx =-pas;}
		if( twist.linear.y > 0){ dy =pas;}
		if( twist.linear.y < 0){ dy =-pas;}
		if( twist.angular.z > 0){ da =pas;}
		if( twist.angular.z < 0){ da =-pas;}
		///cout<<"x "<<dx<<" y "<<dy<<" a "<<da<<endl;

		// use the Methods from the class GeneTraj to generate the move the hexapod
		gen.createTraj(dx, dy, 0, da);
		// create the command message, translation from angle -> motor ref
		gen.createCmd();
		//send the command to the motors' board
		envoieCmd();
    }
}

/**
* @brief   send the message in Cmd to the low level motors' board
* @param   void
* @return  void
*/
void  envoieCmd()
{
	ROS_DEBUG("ENVOIE SEQUENCE");
	hexapode_v2::sequence_moteur seq_srv;
	// set the message 
	seq_srv.request.sequence = gen.getCmd();
	seq_srv.request.dim = seq_srv.request.sequence.size()+2;
	// call the service 
	pclient->call(seq_srv);
}
/**
* @brief   NOT USED YET
* @param   ROS message of type etat_robot 
* @return  void
*/
void ecouteEtatRobot(const hexapode_v2::etat_robot etat_robot)
{
}

//======================================================================

int main(int argc, char** argv)
{
	ros::init(argc, argv, "generationtrajectoire"); // initialise the node 
	if (argc > 1){ // if an argument is given when the node is run from the shell or a launch file
		if (0 == strcmp(argv[1], "0")){ // if the first argument is equal to 0
			gen.initRobot1(); // set small hexapode 
		}
		else{
			gen.initRobot2();// set big hexapode 
		}
	}
	else{ // by default set big hexapod
		gen.initRobot2();
	}
	
	
	ros::NodeHandle n;
	// create the client to trigger the sequence_moteur service
	ros::ServiceClient client = n.serviceClient<hexapode_v2::sequence_moteur>("sequence_");
	pclient = &client;
	
	// create 2 subscriber 
    ros::Subscriber trans_sub = n.subscribe("translation", 1000, ecouteTranslation);
    // not used yet
    ros::Subscriber etat_sub = n.subscribe("etat_robot", 1000, ecouteEtatRobot);
    
    ros::Subscriber nav_sub = n.subscribe("cmd_vel", 1000, ecouteNavigation);
    
	//ROS spin, the node wait there and trigger the subscruber at regular intervals
    ros::spin();
    
    return 0;
}








