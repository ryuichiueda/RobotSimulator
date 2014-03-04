#include "World.h"
#include "PointLandmark.h"
#include "GoalAsLandmark.h"
using namespace std;

World::World (ifstream *rand_ifs){
	m_pi = 3.141592;
	mp_landmarks.push_back(new PointLandmark(0,0.0,0.0,rand_ifs));
	mp_landmarks.push_back(new GoalAsLandmark(1,goal_x,goal_y,body_r,rand_ifs));

	m_world_x_min = -2000.0;
	m_world_y_min = -2000.0;
	m_world_x_max = 2000.0;
	m_world_y_max = 2000.0;
}

World::~World (){
	for(vector<Landmark*>::iterator i=mp_landmarks.begin();
		i!=mp_landmarks.end();i++){
		delete *i;
	}
}

vector<Landmark*>* World::getLandmarks(void){
	return &mp_landmarks;
}


void World::printLandmarks(void){
	for(vector<Landmark*>::iterator i=mp_landmarks.begin();
		i!=mp_landmarks.end();i++){
		(*i)->print();
	}
}
