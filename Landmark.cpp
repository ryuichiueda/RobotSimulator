#include "Landmark.h"

Landmark::Landmark(int id,ifstream *rand_ifs){
	m_id = id;
	m_rand_ifs = rand_ifs;
}

bool Landmark::hasId(int id){
	return id == m_id;
}

const double Landmark::normalizeAngle(double ang){
	while(ang > pi || ang < -pi){
		if(ang > pi)	ang -= pi*2;
		else		ang += pi*2;
	}
	return ang;
}

unsigned int Landmark::getIntRand(){
	char buf[4];
	m_rand_ifs->read(buf,4);
	return (buf[0]<<24) + (buf[1]<<16) + (buf[2]<<8) + buf[3];
}

double Landmark::getDoubleRand(){
	return (double)getIntRand() / UINT_MAX;
}

double Landmark::getGaussRand(){
	double tmp = 0.0;
	for(int i=0;i<12;i++)
		tmp += getDoubleRand();
	
	return tmp - 6.0;
} 

double Landmark::gaussFunction(double sigma,double diff){
	double h = - (diff * diff) / (2 * sigma * sigma);
	return exp(h) / (sigma * (sqrt(2*pi)));
}
