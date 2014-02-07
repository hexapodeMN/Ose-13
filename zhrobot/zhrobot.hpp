/**
* @file     zhrobot.hpp
* @brief    Classes en methods from zhrobot 
* @details  Describe joint and legs from a robot, propose an inverse 
* 			kinematic model
* @author   caesarhao@gmail.com
* @copy		Ecole des Mines de Nantes
* @date     2014-02-05
*/

#ifndef _ZH_ROBOT_H_
#define _ZH_ROBOT_H_

//dependecies
#include "matclpro/cmatrix"


// defintion of the namespace
namespace zhrobot{


	// defintion of the Pi constant instead of using math.h
	#define PI	(3.1415926)
	
	// definition of custom types
	typedef techsoft::matrix<double>       dMatrix;
	typedef std::valarray<double>          dVector;
	
	/** 
	* @brief   print the value of a vector in the shell
	* @param   dVector& vector 
	* @return  void
	*/
	void printValarray(const dVector& va);
	
	/**
	* @enum	 JointType
	* @brief   a joint could be either prismatic or revolute
	*/
	typedef enum _JointType{
		E_JOINT_R,
		E_JOINT_P
	} JointType;

	/**
	* @class	Link
	* @brief   describe a Link in the robot's leg
	* 
	* It describe the type of joint plus geometric parameters to 
	* describe the transformation from one joint to the next in 
	* the robot's leg. 
	* The transformation should use Denavit-Hartenberg parameters 
	* but there has been a notation missuse
	*/
	class Link{
		private:
			double theta; // rotation around z_i
			double d; //distance along z_i
			double a; //distance along x_i-1
			double alpha; //rotation around x_i-1
			JointType jt; // joint type either prismatic or revolutive
			dMatrix A; // transformation matrix of the joint 
			           //according to the Link parameters 
			/** 
			* @brief   compute the transformation matrix of the joint 
			* 		according to the Link parameters (private)
			* @param   none uses the attribute from Link
			* @return  void
			*/
			void calcA();
			
		public:
			/**
			 * @brief constructor for Link structure
	 	 	*/ 
			Link(double _theta=0, double _d = 0, double _a = 0, double _alpha = 0, JointType _jt = E_JOINT_R);
			/**
		 	* @brief deep copy method
		 	*/ 
			Link(const Link& link);
			/**
		 	* @brief set the parameter q accordign to the joint type
			*/ 
			void setQ(double q);
			void setTheta(double theta); 
			void setD(double d); 
			void setA(double a); 
			void setAlpha(double alpha);
			double &getParaTheta(); 
			double &getParaD(); 
			double &getParaA(); 
			double &getParaAlpha(); 
			dMatrix &getA(); 
	};
	
	
	
	/**
	* @class	Robot
	* @brief   Groups Link to form a leg of the hexapod
	*/
	class Robot{
		private:
			int linknum; //number of Link
			class Link *links; // list of Links
			dMatrix H; //transformation matrix from end-effector 
					//(ie end of the leg) to the base frame of the robot
			
			/**
			 * @brief compute the transformation matrix for 
			 * 			the whole Robot/leg (private)
			 */
			void calcH();
			
		public:
			/**
			 * @brief empty constructor for Robot structure
	 		 */ 
			Robot();
			/**
			 * @brief constructor for Robot structure
			 * @param _Links array of Link and num number of link
	 		 */ 
			Robot(Link _links[], int num);
			/**
			 * @brief deep copy of a Robot
	 		 */ 
			Robot(const Robot& robot);
			/**
			 * @brief destructor for Robot object
	 		 */
			~Robot();
			
			/**
		 	* @brief set q parameter for link number index
			 * @param index of the link in the Robot/leg
			 * @param q value of the q parameter for the link
	 		 */
			void setQ(int index, double q); 
			/**
			 * @brief set q parameter for all link in the Robot/leg
			 * @param qs vector of q parameters
	 		 */
			void setQs(dVector qs); 
			/**
			 * @brief get H the transforamtion matrix for the Robot/leg
			 * @return H transforamtion matrix for the Robot/leg
	 		 */
	 	 	dMatrix &getH();
	 	 	/**
			 * @brief get A the transforamtion matrix for num-th link
			 * @return A the transforamtion matrix for num-th link
	 		 */
			dMatrix &getA(int num); 
			/**
		 	* @brief get index-th Link of the Robot/leg
		 	* @return a Link
	 		 */
			Link &getLink(int index); 
			 
			 
			/**
			 * @brief fills in the Link if a Robot/leg as been created 
			 * without Links
			 * 
			 * comment it may be simpler tu suppress the empty constructor
	 		 */
			void setLinks(Link _links[], int num); 
			/**
			 * @brief get the initial cartesian position of the end effector
			 * @param src a Vector of the end effector position in 
			 * 		the end effector frame, src always equals [0 0 0 1]'
			 * @return a vector [x y z 1]' in the 0-frame
	 		 */
			dVector getDestCoord(dVector &src);
			/**
			 * @brief compute the inver kinematic for a given end effector position
			 * @return joint position for the cartesan given position of the end effector
			 * 
			 * NOT REVIEWED
			 * 
	 		 */
			dVector invertCoord(dVector &dst);
	};

};// end namespace zhrobot
#endif

