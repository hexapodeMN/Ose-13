/**
* @file     zhrobot.cpp
* @brief    les classes et methodes de zhrobot.
* @details  Cette librairie est pour mettre en modele des articulations et calculer cinétique et cinétique inverse.
* @author   caesarhao@gmail.com
* @copy		Ecole des Mines de Nantes
* @date     2013-03-15
*/
#include <iostream>
#include "zhrobot.hpp"


namespace zhrobot
{
	/** 
	* @brief   print the value of a vector in the shell
	* @param   dVector& vector 
	* @return  void
	*/
	void printValarray(const dVector& va)
	{ //ecrit dans la console les valeurs d'un vecteur
	    for (int i=0; i<va.size(); i++) {
	        std::cout << va[i] << ' ';
		}
		std::cout <<std::endl;
	}
	
// Link structure

	/**
	 * @brief constructor for Link structure
 	 */ 
	Link::Link(double _theta, double _d, double _a, double _alpha, JointType _jt){
		// resize A to 4x4 matrix
		A.resize(4, 4); //  is the transformation matrix  T from i-1 to i configuration
		theta = _theta; // rotation around z_i
		d = _d; //distance along z_i
		a = _a; //distance along x_i-1
		alpha = _alpha; // rotation around x_i-1
		//  /!\  this is not the classical convention (taking Denavit-Hartenberg as classic approach )
		jt = _jt; // is the type of articulation, either prismatic or revolute
		// compute the transformation matrix
		calcA();
	}
	/**
	 * @brief deep copy method
	 */ 
	Link::Link(const Link& link)
	{//deep copy of a Link object
		new (this)Link(link.theta, link.d, link.a, link.alpha, link.jt);
	}
	
	//  setters
	
	void Link::setQ(double q){
	// set the parameter according to the joint type
		if (E_JOINT_R == jt){ // revolute joint
			theta = q;
		}
		else{ // prismatic joint, although it wont occure on our hexapod
			d = q;
		}
		// compute the transformation matrix
		calcA();
	}
	void Link::setTheta(double theta){
		this->theta = theta;	
	}
	void Link::setD(double d){
		this->d = d;	
	}
	void Link::setA(double a){
		this->a = a;	
	}
	void Link::setAlpha(double alpha){
		this->alpha = alpha;	
	}
	
	// getters
	
	dMatrix& Link::getA(){
		return A;
	}
	double &Link::getParaTheta(){
		return theta;
	}
	double &Link::getParaD(){
		return d;
	}
	double &Link::getParaA(){
		return a;
	}
	double &Link::getParaAlpha(){
		return alpha;
	}
	
	// Methods for Link objects
	
	/** 
	* @brief   compute the transformation matrix of the joint according to the Link parameters
	* @param   none uses the attribute from Link
	* @return  void
	*/
	void Link::calcA(){
		// A = Rz*Tz*Tx*Rx
		// according to Denavit-Hartenberg parameters
		// for all matrix M [row][column]
		dMatrix Rz(4,4,0.0), Tz(4,4,0.0), Tx(4,4,0.0), Rx(4,4,0.0); // create 4x4 null matrix
		Rz.unit(); // the rotation matrix around z axis
		Rz[0][0] = Rz[1][1]= cos(theta);
		Rz[0][1] = -sin(theta);
		Rz[1][0] = sin(theta);
		Tz.unit(); // the translation along z axis
		Tz[2][3] = d;
		Tx.unit(); // the translation along x axis
		Tx[0][3] = a;
		Rx.unit(); // the roation arounf x axis
		Rx[1][1] = Rx[2][2] = cos(alpha);
		Rx[1][2] = -sin(alpha);
		Rx[2][1] = sin(alpha);
		// compute the transformation matrix
		A = Rz * Tz * Tx * Rx;
	}

// Robot structure
//   /!\ Robot refers in fact to a leg and not the whole robot

	/**
	 * @brief empty constructor for Robot structure
 	 */ 
	Robot::Robot(){	}
	
	/**
	 * @brief constructor for Robot structure
	 * @param _Links array of Link and num number of link
 	 */ 
	Robot::Robot(Link _links[], int num){
		// H is the transformation matrix from end-effector (ie end of the leg) to the base frame of the robot
		// /!\ assumption _links.size() == num
		H.resize(4,4);
		// Copy of the array of Link, not sure if deep
		links = new Link[num];
		for (int i = 0; i<num; i++){
			links[i] = _links[i];
		}
		// store num n the robot attribute linknum whish is the number of Link in the leg/Robot
		linknum = num;
		// compute the transformation matrix for the leg
		calcH();
	}
	
	/**
	 * @brief deep copy of a Robot
 	 */ 
	Robot::Robot(const Robot& robot){
		new (this)Robot(robot.links, robot.linknum);
	}
	
	/**
	 * @brief destructor for Robot object
 	 */	
	Robot::~Robot(){
		if (NULL != links){
			delete[] links;
			links = NULL;
		}
	}
	
	// setters
	
	/**
	 * @brief set q parameter for link number index
	 * @param index of the link in the Robot/leg
	 * @param q value of the q parameter for the link
 	 */
	void Robot::setQ(int index, double q){
		links[index].setQ(q);
		// compute the transformation matrix for the Robot/leg
		calcH();
	}
	/**
	 * @brief set q parameter for all link in the Robot/leg
	 * @param qs vector of q parameters
 	 */
	void Robot::setQs(dVector qs){
		for (int i = 0; i<linknum; i++){
			links[i].setQ(qs[i]);
		}
		// compute the transformation matrix for the Robot/leg
		calcH();
	}	
	
	//getters
	
	/**
	 * @brief get H the transforamtion matrix for the Robot/leg
	 * @return H transforamtion matrix for the Robot/leg
 	 */
	dMatrix& Robot::getH(){
		return H;
	}
	/**
	 * @brief get A the transforamtion matrix for num-th link
	 * @return A the transforamtion matrix for num-th link
 	 */
	dMatrix & Robot::getA(int num){
		return links[num].getA();
	}
	/**
	 * @brief get index-th Link of the Robot/leg
	 * @return a Link
 	 */
	Link& Robot::getLink(int index){
		return links[index];
	}
	
	
	// Methodes for Robot Object
	/**
	 * @brief fills in the Link if a Robot/leg as been created without Links
	 * 
	 * comment it may be simpler tu suppress the empty constructor
 	 */
	void Robot::setLinks(Link _links[], int num){
		H.resize(4,4);
		links = new Link[num];
		for (int i = 0; i<num; i++){
			links[i] = _links[i];
		}
		linknum = num;
		// compute the transformation matrix for the Robot/leg
		calcH();	
	}
	
	/**
	 * @brief get the initial cartesian position of the end effector
	 * @param src a Vector of the end effector position in the end effector frame, src always equals [0 0 0 1]'
	 * @return a vector [x y z 1]' in the 0-frame
 	 */
	dVector Robot::getDestCoord(dVector &src){
		// H is the transformation matrix for the Robot/leg
		dVector d = H*src;
		return d;
	}
	/**
	 * @brief compute the transformation matrix for the whole Robot/leg
	 */
	void Robot::calcH(){
		// H is the transforamtion matrix for the Robot/leg
		H.unit();
		for (int i = 0; i < linknum; i++)
			//compute the transformation matrix of the Robot/leg by multiplying the transforamtion matrices from all Links
			// the order is good, so that H transform an end effector coordinate in a 0-frame coordinate
			H *= links[i].getA(); 
		}
	}
	/**
	 * @brief compute the inver kinematic for a given end effector position
	 * @return joint position for the cartesan given position of the end effector
	 * 
	 * NOT REVIEWED
	 * 
 	 */
	dVector Robot::invertCoord(dVector &dst){
		double x = dst[0]; //x
		double y = dst[1]; //y
		double z = dst[2]; //z
		// using the notation from the scheme in hexapode_v2 generation_trajectoire.cpp
		
		
		// distance de axe z4 et z2.
		double z42 = z - links[1].getParaD();
		// theta entre x0 et x1.
		double theta1 = links[0].getParaTheta();
		// a entre z0 et z1.
		double a1 = links[0].getParaA();
		double a2 = links[1].getParaA();
		double a3 = links[2].getParaA();
		double a4 = links[3].getParaA();
		// distance de y entre 4 et 1.
		double y41 = y - a1*sin(theta1);
		// distance de x entre 4 et 1.
		double x41 = x - a1*cos(theta1);
		// calculer theta entre x1 et x2.
		double theta2 = atan2(y41, x41) - atan2(sin(theta1), cos(theta1));
		theta2 = atan(tan(theta2));
		double m = sqrt(pow(x41, 2)+pow(y41, 2)) - a2;
		double w = (a4*a4 + z42*z42 +m*m -a3*a3)/(2*a4*sqrt(m*m+z42*z42));
		double betatheta = asin(w);
		if (betatheta < 0){
			betatheta += PI;
		}
		double beta = atan2(m, z42);
		double theta4a3 = betatheta - beta;
		double theta3 = asin((z42 - a4 * sin(theta4a3))/a3);
		double theta4 = theta4a3 - theta3;
		double thetas[] = {theta1, theta2, theta3, theta4};
		dVector rst(thetas, 4);
		return rst;
	}
	
	// A METHODE FOR THE DIRECT KINEMATIC MAY BE USEFUL
};

