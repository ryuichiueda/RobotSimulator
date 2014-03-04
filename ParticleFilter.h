#ifndef PARTICLE_FILTER_H__
#define PARTICLE_FILTER_H__
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "Robot.h"
#include "Landmark.h"
using namespace std;

class Robot;
class World;
class Landmark;
class PointLandmark;
class GoalAsLandmark;
struct Particle{State state;double weight;};

class ParticleFilter {
public:

	ParticleFilter(int num,ifstream *ifs);
	~ParticleFilter();

	void print(void);

	void resetParticles(double cx,double cy,double ctheta,
				double stdev_r,double stdev_theta);
	void resetParticles(World *w);

	vector<Particle> *getParticles(void){return &m_particles;}
	int getParticleNum(void){return m_num;}

	void motion(Robot *robot,int action,bool resampling);
	void systematicResampling(Robot *robot,int action);
	void residualResampling(Robot *robot,int action);
	void moveParticles(Robot *robot,int action);

	void normalize(void);

	void observation(State *s,Landmark *landmark);
	
	Particle *getParticleAt(int pos){return &m_particles[pos];}
private:
	typedef vector<Particle>::iterator pit;

	vector<Particle> m_particles;
	ifstream *m_rand_ifs;

	void printParticle(Particle &p);

	int m_num;

	void pointLandmarkProc(State *s,PointLandmark *landmark);
	void goalAsLandmarkProc(State *s,GoalAsLandmark *landmark);
	const double normalizeAngle(double ang);

	unsigned int getIntRand();
	double getDoubleRand();
	double getGauss();
	double getGaussRand();

	double gaussFunction(double sigma,double diff);
	void sensorReset(PolerPos *measure,PointLandmark *landmark);
};
#endif
