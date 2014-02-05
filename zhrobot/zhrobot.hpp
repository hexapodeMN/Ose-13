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

#include "matclpro/cmatrix"
// la space de nom pour cette librairie.
namespace zhrobot{
// la valeur de PI.
#define PI	(3.1415926)

typedef techsoft::matrix<double>       dMatrix;
typedef std::valarray<double>          dVector;
/**
* @brief   fonction pour imprimer les arrays.
* @param   dVector
* @return  void
*/
void printValarray(const dVector& va);
/**
* @enum	JointType
* @brief   differentes types de articulations.
*/
typedef enum _JointType{
	E_JOINT_R,
	E_JOINT_P
} JointType;
/**
* @class	Link
* @brief   classe pour mettre en modele d'une articulation.
*/
class Link{
	private:
		double theta; ///< angle entre x-1 et x.
		double d; ///< distance entre x-1 et x.
		double a; ///< distance entre z et z+1.
		double alpha; ///< angle entre z et z+1.
		JointType jt; ///< le type d'articulation.
		dMatrix A; ///< le matrice de cette articulation.
		/**
		* @brief   fonction pour calculer la matrice des parametres.
		* @param   void
		* @return  void
		*/
		void calcA();
	public:
		Link(double _theta=0, double _d = 0, double _a = 0, double _alpha = 0, JointType _jt = E_JOINT_R);
		Link(const Link& link);
		/**
		* @brief   fonction pour modifier la paramettre de l'articulation, c'est theta pour E_JOINT_R, et d pour E_JOINT_P.
		* @param   double
		* @return  void
		*/
		void setQ(double q);
		void setTheta(double theta); ///< modifier theta.
		void setD(double d); ///< modifier d.
		void setA(double a); ///< modifier a.
		void setAlpha(double alpha); ///< modifier alpha.
		double &getParaTheta();  ///< obtenir theta.
		double &getParaD(); ///< obtenir d.
		double &getParaA(); ///< obtenir a.
		double &getParaAlpha(); ///< obtenir alpha.
		dMatrix &getA(); ///< obtenir matrice A.
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

