#ifndef STATE_KNOWN_POLICY_H__
#define STATE_KNOWN_POLICY_H__
#include "Policy.h"
#include "Robot.h"
using namespace std;

class Robot;

class StateKnownPolicy : public Policy {
public:
	StateKnownPolicy();
	virtual ~StateKnownPolicy();

private:
	virtual int decision(Robot *r);
};
#endif
