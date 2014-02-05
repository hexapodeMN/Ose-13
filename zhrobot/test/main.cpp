#include <iostream>

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
#include "zhrobot.hpp"
using namespace std;
using namespace zhrobot;
int main(int argc, char *argv[]) {

	Link ls[]={
		//   theta, d,		 a, 	alpha
		Link(0,		0, 		136, 	0),
		Link(0,		0,		30,		PI/2),
		Link(0,		0,		57,		0),
		Link(-PI/2,	0,		132,	 0)
	};
	
	Robot rb(ls, 4);
	
	Robot rbs[] = {rb, rb, rb, rb, rb, rb};
	
	for (int i = 0; i<6; i++){
		double theta1 = i*PI/3;
		theta1 = (PI < theta1)?(theta1-2*PI):theta1;
		rbs[i].setQ(0, theta1);
		cout<<rbs[i].getH()<<endl;
	}

	dVector srcCoor(0.0, 4);
	srcCoor[3] = 1;
	dVector dstCoor[6];
	for (int i = 0; i< 6; i++){
		dstCoor[i].resize(4);
		dstCoor[i] = rbs[i].getDestCoord(srcCoor);
		printValarray(dstCoor[i]);
	}
	cout << "----------------------"<<endl;

	for (int i = 0; i< 6; i++){
		// z max 3.24
		// y max 4.9
		// x max 
		dstCoor[i][0] += -4;
		printValarray(rbs[i].invertCoord(dstCoor[i]));
	}
	//dst0[1] -= 4;
	//dst2[1] -= 4;
	//dst4[1] -= 4;
	//printValarray(rbs[0].invertCoord(dst0));
	//printValarray(rbs[2].invertCoord(dst2));
	//printValarray(rbs[4].invertCoord(dst4));
	//printValarray(dstC1);
	
	return 0;
}

