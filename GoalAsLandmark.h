#ifndef GOAL_LANDMARK_H__
#define GOAL_LANDMARK_H__
#include <iostream>
#include "Landmark.h"
using namespace std;

//class Landmark;
class ParticleFilter;

class GoalAsLandmark : public Landmark {
public:
	GoalAsLandmark(int id, double x, double y,double r,ifstream *rand_ifs);
	~GoalAsLandmark(){}

	void print(void);

	double getXPos(void){return m_position_x;};
	double getYPos(void){return m_position_y;};
	bool inGoal(State *s);
	PolerPos getRelativePos(State *s);

//	State sampling(PolerPos *measure);
private:
	double m_position_x;
	double m_position_y;
	double m_radius;

/*
	unsigned int getIntRand();
	double getDoubleRand();
	double getGaussRand();
	double gaussFunction(double sigma,double diff);
*/
};
#endif
