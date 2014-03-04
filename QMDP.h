#ifndef QMDP_POLICY_H__
#define QMDP_POLICY_H__
#include "Policy.h"
#include "Robot.h"
using namespace std;

class Robot;

class QMDP : public Policy {
public:
	QMDP();
	virtual ~QMDP();
private:
	virtual int decision(Robot *r);
};
#endif
