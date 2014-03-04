#include "StateKnownPolicy.h"

StateKnownPolicy::StateKnownPolicy(): Policy(){
}

StateKnownPolicy::~StateKnownPolicy(){
}

int StateKnownPolicy::decision(Robot *r){
	vector<int> *p = r->getActions();
	double min_value = 1.0e100;
	int min_action = -1;
	for(vector<int>::iterator i=p->begin();i!=p->end();i++){
		double v = r->getEstimatedValue(*i);
		if( v < min_value){
			min_value = v;
			min_action = *i;
		}
	}

	return min_action;
}
