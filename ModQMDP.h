#ifndef MODQMDP_POLICY_H__
#define MODQMDP_POLICY_H__
#include "Policy.h"
#include "Robot.h"
using namespace std;

class Robot;

class ModQMDP : public Policy {
public:
	ModQMDP();
	virtual ~ModQMDP();
private:
	virtual int decision(Robot *r);
};
#endif
