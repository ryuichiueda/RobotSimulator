#ifndef ROBOT_H__
#define ROBOT_H__
#include "World.h"
#include "Landmark.h"
#include "ParticleFilter.h"
#include <vector>
#include <fstream>
#include "Coordinates.h"
using namespace std;

class Policy;
class Landmark;
class ParticleFilter;
class StateKnownPolicy;

class Robot {
public:
	enum Action {fw,cw,ccw};

	Robot(double x,double y, double theta,ifstream *rand_file);
	~Robot();

	void printState(void);
	void printState(State &s);
	void printGoal(void);

	ParticleFilter* getParticleFilter(void);

	vector<int>* getActions(void){return &m_actions;};

	void step(bool resampling);

	double getValue(void);
	double getValue(State *s);
	double getEstimatedValue(int a,State *prev);
	double getEstimatedValue(int a);

	Movement actionToMeanMovement(int a);
	Movement actionToMovement(int a);

	State posteriorState(Movement *m,State *prev);

	const char *actionToStr(int a); 

	bool isInGoal(void);
	void registerLandmarks(vector<Landmark*> *p);

	void observeLandmark(int id);

	int getStep(void);
private:
	typedef vector<Policy*>::iterator pit;

	vector<Landmark*> *mp_known_landmarks;
	State m_state;
	double m_body_radius;
	vector<ParticleFilter> m_particle_filters;
	vector<Policy*> mp_policies;
	vector<int> m_actions;

	double m_one_step_fw;
	double m_one_step_rot;

	ifstream *m_rand_ifs;

	double m_goal_x;
	double m_goal_y;

	unsigned int getIntRand();
	double getDoubleRand();
	double getGauss();

	PolerPos getGoalPosition(void);
	PolerPos getGoalPosition(State *s);

	State posteriorCenterState(int a);
	State posteriorState(int a);

	const double normalizeAngle(double ang);

	int m_step_counter;
};
#endif
