#ifndef LANDMARK_H__
#define LANDMARK_H__
#include <iostream>
#include <fstream>
#include "Coordinates.h"
#include "ParticleFilter.h"
using namespace std;

enum LandmarkType{point,line,goal};

class ParticleFilter;

class Landmark {
public:
	Landmark(int id,ifstream *rand_ifs);
	virtual ~Landmark(){}

	bool hasId(int id);
	int getType(void){return m_type;};
	virtual void print(void) = 0;

	//virtual double observed(State *s,ParticleFilter *p) = 0;
protected:
	int m_id;
	int m_type;
	ifstream* m_rand_ifs;
	const double normalizeAngle(double ang);

	unsigned int getIntRand();
	double getDoubleRand();
	double getGaussRand();
	double gaussFunction(double sigma,double diff);
};
#endif
