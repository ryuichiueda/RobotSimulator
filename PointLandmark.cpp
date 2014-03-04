#include "PointLandmark.h"

PointLandmark::PointLandmark(int id,double x, double y,ifstream* rand_ifs)
 : Landmark(id,rand_ifs){
	m_position_x = x;
	m_position_y = y;

	m_type = point;
}

void PointLandmark::print(void){
	cout << "LANDMARK " << m_position_x << " " << m_position_y << endl;
}

PolerPos PointLandmark::getRelativePos(State *s){
	PolerPos pos;

	double vx = m_position_x - s->x;
	double vy = m_position_y - s->y;

	pos.distance = sqrt(vx*vx + vy*vy);
	pos.direction = normalizeAngle(atan2(vy,vx) - s->theta);

	return pos;
}

PolerPos PointLandmark::getMeasure(State *s){
	PolerPos pos = getRelativePos(s);

	pos.distance *= (1.0 + 0.1*getGaussRand());
	pos.direction += (10.0/180.0)*pi*getGaussRand();

	return pos;
}

State PointLandmark::sampling(PolerPos *measure){
	State s;

	PolerPos m;
	m.distance = measure->distance * (1.0 + 0.1*getGaussRand());
	m.direction = measure->direction + (10.0/180.0)*pi*getGaussRand();
	

	double rnd = getDoubleRand()*2*pi;	
	s.x = m_position_x + m.distance * cos(rnd);
	s.y = m_position_y + m.distance * sin(rnd);

	double vx = m_position_x - s.x;
	double vy = m_position_y - s.y;

	s.theta = normalizeAngle(atan2(vy,vx) - m.direction);

	return s;
}
