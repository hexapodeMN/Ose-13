/**
* @file     zhrobot.hpp
* @brief    les classes et methodes de zhrobot.
* @details  Cette librairie est pour mettre en modele des articulations et calculer cinétique et cinétique inverse.
* @author   caesarhao@gmail.com
* @copy		Ecole des Mines de Nantes
* @date     2013-03-15
*/

#ifndef _ZH_ROBOT_H_
#define _ZH_ROBOT_H_

/dependecies
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
* It describe the type of joint plus geometric parameters to describe the transformation 
* from one joint to the next in the robot's leg. 
* The transformation should use Denavit-Hartenberg parameters but there has been a notation missuse
*/
class Link{
	private:
		double theta; // rotation around z_i
		double d; //distance along z_i
		double a; //distance along x_i-1
		double alpha; //rotation around x_i-1
		JointType jt; // joint type either prismatic or revolutive
		dMatrix A; ///< le matrice de cette articulation.
		/** 
		* @brief   compute the transformation matrix of the joint according to the Link parameters (private)
		* @param   none uses the attribute from Link
		* @return  void
		*/
		void calcA();
		
	public:
		Link(double _theta=0, double _d = 0, double _a = 0, double _alpha = 0, JointType _jt = E_JOINT_R);
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
* @brief   classe pour mettre en modele d'un robot avec quelques articulations(Link).
*/
class Robot{
	private:
		int linknum; ///< nombre d'articulations.
		class Link *links; ///< liste de Links.
		dMatrix H; ///< la matrice calculee des Links.
		/**
		* @brief   fonction pour calculer la matrice H.
		* @param   void
		* @return  void
		*/
		void calcH();
	public:
		Robot();
		Robot(Link _links[], int num);
		Robot(const Robot& robot);
		~Robot();
		void setLinks(Link _links[], int num); ///< modifier les links.
		void setQ(int index, double q); ///< modifier la paramettre de differents links.
		void setQs(dVector qs); ///< modifier les paramettres des links une fois.
		Link &getLink(int index); ///< obtenir le link index(0-(n-1)).
		dMatrix &getH(); ///< obtenir la matrice H.
		dMatrix &getA(int num=0); ///< obtenir la matrix A de link num.
		/**
		* @brief   fonction pour calculer la cinetique, obtenir la coordonnee dans systeme de coordonnee zero de la coordonnee dans T(0, 0, 0, 1).
		* @param   dVector(0, 0, 0, 1)
		* @return  dVector(x0, y0, z0, 1)
		*/
		dVector getDestCoord(dVector &src);
		/**
		* @brief   fonction pour calculer la cinetique inverse, obtenir les paramettres des Links de la coordonnee dans systeme de coordonnee zero.
		* @param   dVector(x0, y0, z0, 1)
		* @return  dVector(q1, q2, q3, q4)
		*/
		dVector invertCoord(dVector &dst);
};
};// end namespace zhrobot
#endif

