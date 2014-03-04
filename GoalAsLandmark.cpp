#include "GoalAsLandmark.h"

GoalAsLandmark::GoalAsLandmark(int id,double x, double y,double r,ifstream* rand_ifs)
 : Landmark(id,rand_ifs){
	m_position_x = x;
	m_position_y = y;
	m_radius = r;

	m_type = goal;
}

void GoalAsLandmark::print(void){
	cout << "GOAL " << m_position_x << " " << m_position_y 
             << " " << m_radius << endl;
}

PolerPos GoalAsLandmark::getRelativePos(State *s){
	PolerPos pos;

	double vx = m_position_x - s->x;
	double vy = m_position_y - s->y;

	pos.distance = sqrt(vx*vx + vy*vy);
	pos.direction = normalizeAngle(atan2(vy,vx) - s->theta);

	return pos;
}

bool GoalAsLandmark::inGoal(State *s){
	PolerPos p = getRelativePos(s);

	return p.distance <= body_r;
}
