#include "ModQMDP.h"

ModQMDP::ModQMDP(): Policy(){
}

ModQMDP::~ModQMDP(){
}

int ModQMDP::decision(Robot *r){
	vector<int> *as = r->getActions();
	ParticleFilter *pf = r->getParticleFilter();

	double min_value = 1.0e100;
	int min_action = -1;

	//comparison of actions
	for(vector<int>::iterator a=as->begin();a!=as->end();a++){
		double sum_value = 0.0;
		for(int j=0; j< pf->getParticleNum(); j++){
			Particle *p = pf->getParticleAt(j);
			double v = r->getValue(&p->state);

			if(v != 0.0)
				sum_value += r->getEstimatedValue((*a),&p->state)*p->weight/v;
		}

		if(sum_value < min_value){
			min_value = sum_value;
			min_action = *a;
		}
	}

	return min_action;
}
