#ifndef POINT_LANDMARK_H__
#define POINT_LANDMARK_H__
#include <iostream>
#include "Landmark.h"
/*
#include "World.h"
#include "Robot.h"
#include "ParticleFilter.h"
#include "Coordinates.h"
*/
using namespace std;

//class Landmark;
class ParticleFilter;

class PointLandmark : public Landmark {
public:
	PointLandmark(int id, double x, double y,ifstream *rand_ifs);
	~PointLandmark(){}

	void print(void);
	//double observed(State *s,ParticleFilter *p);

	//double likelyhood(State *s,Observation *obs);
	double getXPos(void){return m_position_x;};
	double getYPos(void){return m_position_y;};
	PolerPos getMeasure(State *s);
	PolerPos getRelativePos(State *s);

	State sampling(PolerPos *measure);
private:
	double m_position_x;
	double m_position_y;

/*
	unsigned int getIntRand();
	double getDoubleRand();
	double getGaussRand();

	double gaussFunction(double sigma,double diff);
*/
};
#endif
