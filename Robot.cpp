#include "Robot.h"
//#include "StateKnownPolicy.h"
#include "ModQMDP.h"
#include "PointLandmark.h"
using namespace std;

Robot::Robot(double x,double y, double theta,ifstream *rand_file){
	m_state.x = x;
	m_state.y = y;
	m_state.theta = theta;

	m_actions.push_back(fw);
	m_actions.push_back(cw);
	m_actions.push_back(ccw);

	m_one_step_fw = 10.0;
	m_one_step_rot = 5.0*pi/180;

	m_body_radius = body_r;

	m_rand_ifs = rand_file;

	m_particle_filters.push_back(ParticleFilter(10000,rand_file));
	//mp_policies.push_back(new StateKnownPolicy());
	mp_policies.push_back(new ModQMDP());

	m_goal_x = goal_x;
	m_goal_y = goal_y;

	m_step_counter = 0;
}

Robot::~Robot(){
	for(pit i=mp_policies.begin();i!=mp_policies.end();i++){
		delete *i;
	}	
}

void Robot::printState(void){
	cout << "ROBOT " << m_state.x << " " << m_state.y << " "
		<< m_state.theta*180.0/pi << " "
		<< m_body_radius << endl;
}

void Robot::printState(State &s){
	cout << "ROBOT " << s.x << " " << s.y << " "
		<< s.theta*180.0/pi << endl;
}

ParticleFilter* Robot::getParticleFilter(void){
	return &m_particle_filters[0];
}

void Robot::printGoal(void){
	cout << "GOAL " << m_goal_x << " " << m_goal_y << endl;
}

Movement Robot::actionToMovement(int a){
	Movement m = actionToMeanMovement(a);
	double noise_rate = getGauss() * 0.1 + 1.0;
	m.distance = m.distance * noise_rate;
	m.angle = m.angle * noise_rate;

	return m;
}

unsigned int Robot::getIntRand(){
	char buf[4];
	m_rand_ifs->read(buf,4);
	return (buf[0]<<24) + (buf[1]<<16) + (buf[2]<<8) + buf[3];
}

double Robot::getDoubleRand(){
	return (double)getIntRand() / UINT_MAX;
}

double Robot::getGauss(){
	double tmp = 0.0;
	for(int i=0;i<12;i++)
		tmp += getDoubleRand();
	
	return tmp - 6.0;
}

PolerPos Robot::getGoalPosition(void){
	return getGoalPosition(&m_state);
}

PolerPos Robot::getGoalPosition(State *s){
	PolerPos p;

	//distance
	double vx = m_goal_x - s->x;
	double vy = m_goal_y - s->y;
	p.distance = sqrt(vx*vx + vy*vy);

	//direction
	p.direction = normalizeAngle( atan2(vy,vx) - s->theta );

	return p;
}

double Robot::getValue(void){
	PolerPos p = getGoalPosition();
	if(p.distance < m_body_radius)
		return 0.0;

	double d = (p.distance - m_body_radius)/m_one_step_fw; 
	double a = fabs(p.direction) / m_one_step_rot;
	return d + a;
}

double Robot::getValue(State *s){
	PolerPos p = getGoalPosition(s);
	if(p.distance < m_body_radius)
		return 0.0;

	double d = (p.distance - m_body_radius)/m_one_step_fw; 
	double a = fabs(p.direction) / m_one_step_rot;
	return d + a;
}

double Robot::getEstimatedValue(int a,State *prev){
	//actionで変位させる
	Movement m = actionToMeanMovement(a);
	State s = posteriorState(&m,prev);
	
	//極座標を求める
	PolerPos p = getGoalPosition(&s);
	if(p.distance < m_body_radius)
		return 0.0;

	double vd = (p.distance - m_body_radius)/m_one_step_fw; 
	double va = fabs(p.direction) / m_one_step_rot;
	return vd + va;
}

double Robot::getEstimatedValue(int a){
	//actionで変位させる
	State s = posteriorCenterState(a);
	
	//極座標を求める
	PolerPos p = getGoalPosition(&s);
	if(p.distance < m_body_radius)
		return 0.0;

	double vd = (p.distance - m_body_radius)/m_one_step_fw; 
	double va = fabs(p.direction) / m_one_step_rot;
	return vd + va;
}

const double Robot::normalizeAngle(double ang){
	while(ang > pi || ang < -pi){
		if(ang > pi)	ang -= pi*2;
		else		ang += pi*2;
	}
	return ang;
}

int Robot::getStep(){
	return m_step_counter;
}

void Robot::step(bool resampling){
	int a = mp_policies[0]->decision(this);
	getParticleFilter()->motion(this,a,resampling);
	
	m_state = posteriorState(a);

	m_step_counter++;
}

State Robot::posteriorState(Movement *m,State *prev){
	State s;

	s.x = prev->x + m->distance * cos(prev->theta);
	s.y = prev->y + m->distance * sin(prev->theta);
	s.theta = prev->theta + m->angle;

	return s;
}

State Robot::posteriorCenterState(int a){
	Movement m = actionToMeanMovement(a);
	return posteriorState(&m,&m_state);
}

State Robot::posteriorState(int a){
	Movement m = actionToMovement(a);
	return posteriorState(&m,&m_state);
}

Movement Robot::actionToMeanMovement(int a){
	Movement m;
	m.distance = 0.0;
	m.angle = 0.0;

	if(a == fw){
		m.distance = m_one_step_fw;
	}else if(a == cw){
		m.angle = -m_one_step_rot;
	}else if(a == ccw){
		m.angle = m_one_step_rot;
	}

	return m;
}

const char* Robot::actionToStr(int a){
	const char* sfw = "fw";
	const char* scw = "cw";
	const char* sccw = "ccw";
	const char* els = "stay";
	if(a == fw){return sfw;}
	else if(a == cw){return scw;}
	else if(a == ccw){return sccw;}

	return els;
}

bool Robot::isInGoal(void){
	return getValue() == 0.0;
}


void Robot::registerLandmarks(vector<Landmark*> *p){
	mp_known_landmarks = p;
}

void Robot::observeLandmark(int id){
	typedef vector<Landmark*>::iterator it;
	for(it i=mp_known_landmarks->begin();i!=mp_known_landmarks->end();i++){
		if((*i)->hasId(id)){
			getParticleFilter()->observation(&m_state,*i);
		}
	}
}

