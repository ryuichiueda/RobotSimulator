#ifndef ACTION_H__
#define ACTION_H__
#include <vector>
#include <fstream>
#include "Robot.h"
using namespace std;

class Robot;

class Policy {
public:
	Policy();
	virtual ~Policy();
	virtual int decision(Robot *r) = 0;
private:
	//typedef vector<Action>::iterator ait;
};
#endif
