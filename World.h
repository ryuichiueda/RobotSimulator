#ifndef WORLD_H__
#define WORLD_H__
#include "Coordinates.h"
#include "Landmark.h"
#include "World.h"
#include <vector>
#include <iostream>
#include <fstream>
using namespace std;

class World;
class Landmark;

class World {
public:
	World(ifstream *rand_ifs);
	~World();

	vector<Landmark*>* getLandmarks(void);
	void printLandmarks(void);

	double getXmin(void){return m_world_x_min;};
	double getXmax(void){return m_world_x_max;};
	double getYmin(void){return m_world_y_min;};
	double getYmax(void){return m_world_y_max;};
private:
	vector<Landmark*> mp_landmarks;
	double m_pi;

	double m_world_x_min;
	double m_world_y_min;
	double m_world_x_max;
	double m_world_y_max;
};
#endif
