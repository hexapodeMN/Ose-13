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
#include "generationTrajectoire.hpp"

using namespace std;

using namespace zhrobot;
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
	std::cout<<"Initialisation le grand robot"<<std::endl;

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
	//      some translation offsets are differents
		
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
}
/**
* @brief   fonction pour calculer le type de mouvement de robot, en fonction de la commande de client.
* @param   dx, dy, dh, da
* @return  void
*/
move_etat GeneTraj::calcMoveType(double dx, double dy, double dh, double da){
	if (abs(dx)> 0.001){
		return E_MOVE_HORIZONTAL;
	}
	else if (abs(dy)> 0.001){
		return E_MOVE_VERTICAL;
	}
	else if (abs(dh) > 0.001){
		return E_MOVE_UPDOWN;
	}
	else{
		return E_MOVE_TOURNE;
	}
}
/**
* @brief   fonction pour mettre a zero les angles des pattes.
* @param   void
* @return  void
*/
void GeneTraj::retablirAngles(){
	int i = 0;
	for (i = 0; i < 6; i++){
		rbs[i].setQ(1, 0);
		rbs[i].setQ(2, 0);
		rbs[i].setQ(3, -PI/2);
		dstCoor[i] = rbs[i].getDestCoord(srcCoor);
	}
	betaX = betaY = betaZ = betaA = 0;
	memset(vMoteurOld, 0, sizeof(vMoteurOld));
}
/**
* @brief   fonction pour initialiser le grand robot.
* @param   void
* @return  void
*/
int GeneTraj::transAngletoNum(int patteNum, int motorNum, double angle){
	if (patteNum < 3) // gauche
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
			default: // 2.
				if (robotType == 0){ //small hexapod
					return (int)(-(PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{ // big hexapod
					return (int)((PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
		}
	}
	else{ // droite.
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
* @brief   fonction pour calculer la position prochaine.
* @param   int
* @return  void
*/
void GeneTraj::calcNextPosition(int robotNum){
	double a, x, y, factor;
	switch(metat){
		case E_MOVE_UPDOWN:
			// justement changer la hauteur des pattes.
			dstCoor[robotNum][2] = dstCoorStand[robotNum][2] - MAX_Z*sin(betaZ);
			break;
			
		case E_MOVE_VERTICAL:
			// factor est 'a' dans y = a*x^2 + b * x + c.
			factor = -(STEP_UP/(MAX_Y*MAX_Y));
			// obtenir y de betaY.
			y = MAX_Y*sin(betaY);
			if (cos(betaY) > 0){// 1 3 5 dans space, 0 2 4 sur le sol.
				if ((robotNum%2) != 0){ // 1 3 5, dans space.
					dstCoor[robotNum][1] = dstCoorStand[robotNum][1] + y;
					dstCoor[robotNum][2] = factor*(y*y)+C;
				}
				else{ // sur le sol.
					dstCoor[robotNum][1] = dstCoorStand[robotNum][1] - y;
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
			}
			else{
				if ((robotNum%2) != 0){ // 1 3 5, sur le sol.
					dstCoor[robotNum][1] = dstCoorStand[robotNum][1] + y;
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
				else{ // dans space.
					dstCoor[robotNum][1] = dstCoorStand[robotNum][1] - y;
					dstCoor[robotNum][2] = factor*(y*y)+C;
				}
			}
			break;
		case E_MOVE_HORIZONTAL: // c'est la meme que verticale.
			factor = -(STEP_UP/(MAX_X*MAX_X));
			x = MAX_X*sin(betaX);
			if (cos(betaX) > 0){
				if ((robotNum%2) != 0){ // 1 3 5
					dstCoor[robotNum][0] = dstCoorStand[robotNum][0] + x;
					dstCoor[robotNum][2] = factor*(x*x)+C;
				}
				else{
					dstCoor[robotNum][0] = dstCoorStand[robotNum][0] - x;
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
			}
			else{
				if ((robotNum%2) != 0){ // 1 3 5
					dstCoor[robotNum][0] = dstCoorStand[robotNum][0] + x;
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
				else{
					dstCoor[robotNum][0] = dstCoorStand[robotNum][0] - x;
					dstCoor[robotNum][2] = factor*(x*x)+C;
				}
			}
			break;
		case E_MOVE_TOURNE:
			factor = -(STEP_UP/(MAX_A*MAX_A));
			a = MAX_A*sin(betaA);
			// obtenir la longueur de patte. len = sqrt(x^2+y^2).
			double len = sqrt(dstCoorStand[robotNum][0]*dstCoorStand[robotNum][0] + 
				dstCoorStand[robotNum][1]*dstCoorStand[robotNum][1]);
			// obtenir l'angle normale de patte.
			double angleNorm = rbs[robotNum].getLink(0).getParaTheta();
			double angle;
			if (cos(betaA) > 0){// 1 3 5 dans space, 0 2 4 sur le sol.
				if ((robotNum%2) != 0){ // 1 3 5, dans space.
					angle = angleNorm + a;
					dstCoor[robotNum][0] = len*cos(angle);
					dstCoor[robotNum][1] = len*sin(angle);
					dstCoor[robotNum][2] = factor*(a*a)+C;
					//cout << "y: " << y << " a: " << a << " z: " << dstCoor[robotNum][2] <<endl;
				}
				else{ // sur le sol.
					angle = angleNorm - a;
					dstCoor[robotNum][0] = len*cos(angle);
					dstCoor[robotNum][1] = len*sin(angle);
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
			}
			else{
				if ((robotNum%2) != 0){ // 1 3 5, sur le sol.
					angle = angleNorm + a;
					dstCoor[robotNum][0] = len*cos(angle);
					dstCoor[robotNum][1] = len*sin(angle);
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
				else{ // dans space.
					angle = angleNorm - a;
					dstCoor[robotNum][0] = len*cos(angle);
					dstCoor[robotNum][1] = len*sin(angle);
					dstCoor[robotNum][2] = factor*(a*a)+C;
				}
			}
			break;
	}
}
/**
* @brief   fonction pour creer la trajectoire.
* @param   double, double, double, double
* @return  void
*/
void GeneTraj::createTraj(double dBetaX, double dBetaY, double dBetaZ, double dBetaA){
	int i = 0, j = 0;
	move_etat met = calcMoveType(dBetaX,dBetaY,dBetaZ, dBetaA);
	/*  try to avoid bouncy transitions
	if (met != metat){// si le type de mouvement est different, le changer.
		retablirAngles();
		metat = met;
		retat = E_ROBOT_PARALLEL;
	}
	*/
	switch (retat){
		case E_ROBOT_PARALLEL: // 
			switch(metat){
				case E_MOVE_UPDOWN:
					betaZ += dBetaZ;
					break;
				case E_MOVE_HORIZONTAL:
				case E_MOVE_VERTICAL:
					betaX += dBetaX;
					betaY += dBetaY;
					retat = E_ROBOT_LEFT_UP;
					break;
				case E_MOVE_TOURNE:
					betaA += dBetaA;
					retat = E_ROBOT_LEFT_UP;
					break;
			}
			break;
		case E_ROBOT_LEFT_UP: // 0,,2,4 work
		//  added for symmetry
			betaX += dBetaX;
			betaY += dBetaY;
			betaA += dBetaA;
			break;
		// ----	
		case E_ROBOT_RIGHT_UP: // 1,3,5 work
			betaX += dBetaX;
			betaY += dBetaY;
			betaA += dBetaA;
			break;
	}
	for (i = 0; i<6;i++){
		calcNextPosition(i); // calculer la position prochaine pour chaque patte.
		// discar shell output
		//printValarray(dstCoor[i]);
		vangles[i] = rbs[i].invertCoord(dstCoor[i]);
		//printValarray(vangles[i]);
		for (j = 0; j < 3; j++){
			vMoteur[i][j] = transAngletoNum(i, j, vangles[i][1+j]);	
		}
	}
	// cout << "---------------" << endl;
}
/**
* @brief   fonction pour creer la commande pour la carte de robot, en fonction de la datasheet.
* @param   void
* @return  void
*/
void GeneTraj::createCmd(){
	cmd[0] = '\0';
	char temp[20];
	for (int i = 0; i<6; i++){
		for (int j = 0; j < 3; j++){
			if (i<3){
				sprintf(temp, "# %d P%d ", 4*i+j, vMoteur[i][j]);
			}
			else{
				sprintf(temp, "# %d P%d ", 4*(1+i)+j, vMoteur[i][j]);			
			}
			if (abs(vMoteur[i][j] - vMoteurOld[i][j]) > 5){
				strcat(cmd, temp);
			}
			vMoteurOld[i][j] = vMoteur[i][j];
		}
	}
	strcat(cmd, "\r\n");
}

/**
* @brief   NOT USED YET
* @param   vector of 4 angles
* @return  bool
*/
bool GeneTraj::checkAngles(dVector angles){
	if ((angles[1] >= MAX_ANGLE) || (angles[1]<= -MAX_ANGLE)){
		return false;
	}
	else if((angles[2] >= MAX_ANGLE) || (angles[2]<= -MAX_ANGLE)){
		return false;
	}
	else if(((angles[3]+PI/2) >= MAX_ANGLE) || ((angles[3]+PI/2) <= -MAX_ANGLE)){
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
	double factor = 15; // magic factor to slow down the moves

	ROS_DEBUG("dx: [%f]", trans.dx);
	ROS_DEBUG("dy: [%f]", trans.dy);
	ROS_DEBUG("da: [%f]", trans.da);
	ROS_DEBUG("dh: [%f]", trans.dh);
	
	cout << "increments : "<<trans.dx << trans.dy << trans.dh << trans.da << endl;
	
	// use the Methods from the class GeneTraj to generate the move the hexapod
	gen.createTraj(trans.dx/factor, trans.dy/factor, trans.dh/factor, trans.da/factor);
	// create the command message, translation from angle -> motor ref
	gen.createCmd();
	//send the command to the motors' board
	envoieCmd();

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
	// create the client to trigger the moteur_sequence service
	ros::ServiceClient client = n.serviceClient<hexapode_v2::sequence_moteur>("sequence_");
	pclient = &client;
	
	// create 2 subscurber 
    ros::Subscriber trans_sub = n.subscribe("translation", 1000, ecouteTranslation);
    // not used yet
    ros::Subscriber etat_sub = n.subscribe("etat_robot", 1000, ecouteEtatRobot);
    
	//ROS spin, the node wait there and trigger the subscruber at regular intervals
    ros::spin();
    
    return 0;
}








