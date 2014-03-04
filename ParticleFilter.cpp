#include "ParticleFilter.h"
#include "PointLandmark.h"
#include "GoalAsLandmark.h"

ParticleFilter::ParticleFilter(int num,ifstream *ifs)
{
	m_num = num;
	for(int i=0;i<num;i++){
		Particle p;
		p.state.x = 0.0;
		p.state.y = 0.0;
		p.state.theta = 0.0;
		p.weight = 1.0/num;
		m_particles.push_back(p);
	}
	m_rand_ifs = ifs;
}

ParticleFilter::~ParticleFilter(){
}

void ParticleFilter::print(void){
	for(int i=0;i<m_num;i++)
		printParticle(m_particles[i]);	
}

void ParticleFilter::printParticle(Particle &p){
	cout << "PARTICLE " << p.state.x << " " << p.state.y << " "
		<< p.state.theta*180/pi << " " << p.weight << endl;
}

void ParticleFilter::resetParticles(World *w){
	for(int i=0;i<m_num;i++){
		m_particles[i].state.x = (w->getXmax() - w->getXmin())*getDoubleRand() + w->getXmin();
		m_particles[i].state.y = (w->getYmax() - w->getYmin())*getDoubleRand() + w->getYmin();
		m_particles[i].state.theta = 2*pi*getDoubleRand() - pi;
		m_particles[i].weight = 1.0/m_num;
	}
}

void ParticleFilter::resetParticles(double cx,double cy,double ctheta,
				double range_r,double range_theta){

	for(int i=0;i<m_num;i++){
		double rand_ang = getDoubleRand()*2*pi;
		double r = range_r * getDoubleRand();
		m_particles[i].state.x = cx + r * sin(rand_ang);
		m_particles[i].state.y = cy + r * cos(rand_ang);
		m_particles[i].state.theta = ctheta + (getDoubleRand() - 0.5)*range_theta;
		m_particles[i].weight = 1.0/m_num;
	}
}

unsigned int ParticleFilter::getIntRand(){
	char buf[4];
	m_rand_ifs->read(buf,4);
	return (buf[0]<<24) + (buf[1]<<16) + (buf[2]<<8) + buf[3];
}

double ParticleFilter::getDoubleRand(){
	return (double)getIntRand() / UINT_MAX;
}


void ParticleFilter::motion(Robot *robot,int action,bool resampling){
	if(resampling)
		residualResampling(robot,action);
	else
		moveParticles(robot,action);
}

void ParticleFilter::moveParticles(Robot *robot,int action){
	for(int i=0;i<m_num;i++){
		Movement m = robot->actionToMovement(action);
		m_particles[i].state = robot->posteriorState(&m,&m_particles[i].state);
	}
}

void ParticleFilter::residualResampling(Robot *robot,int action){
	random_shuffle(m_particles.begin(),m_particles.end());

	vector<Particle> prev;

	double sum_weight = 0.0;
	for(int i = 0;i < m_num;i++){
		sum_weight += m_particles[i].weight;
		prev.push_back(m_particles[i]);
	}

	double coef = m_num / sum_weight;
	int* choice = new int[m_num];
	for(int i = 0;i < m_num;i++){
		choice[i] = (int)floor(prev[i].weight * coef);
	}

	double new_w = 1.0/m_num;
	int k = 0;
	for(int i = 0;i < m_num;i++){
		for(int j=0;j<choice[i];j++){
			if(k >= m_num) break;

			Movement m = robot->actionToMovement(action);
			m_particles[k].state = robot->posteriorState(&m,&prev[i].state);
			m_particles[k].weight = new_w;
			k++;
		}
	}
	delete [] choice;

	int residual = m_num - k;
	if(residual == 0)
		return;

	double step = ((double)m_num) / residual;
	double accum = 0.0;
	while( k < m_num ){
		int i = (int)accum;
		if(i >= m_num)
			i = m_num - 1;

		Movement m = robot->actionToMovement(action);
		m_particles[k].state = robot->posteriorState(&m,&prev[i].state);
		m_particles[k].weight = new_w;

		accum += step;
		k++;
	}
}

void ParticleFilter::systematicResampling(Robot *robot,int action){

	random_shuffle(m_particles.begin(),m_particles.end());
	vector<Particle> prev;

	double sum_weight = 0.0;
	for(int i = 0;i < m_num;i++){
		//weight is changed to the accumurated value
		m_particles[i].weight += sum_weight;
		sum_weight = m_particles[i].weight;
		prev.push_back(m_particles[i]);
	}

	double step = sum_weight / m_num;
	int* choice = new int[m_num];
	double accum = getDoubleRand() / m_num;
	int j = 0;
	for(int i=0;i<m_num;i++){
		if(prev[j].weight >= accum)
			j++;

		if(j == m_num)
			j--;

		accum += step;
		choice[i] = j;
	}

	double new_w = 1.0/m_num;
	for(int i=0;i<m_num;i++){
		int j = choice[i];
		Movement m = robot->actionToMovement(action);
		m_particles[i].state = robot->posteriorState(&m,&prev[j].state);
		m_particles[i].weight = new_w;
	}

	delete [] choice;
}

void ParticleFilter::pointLandmarkProc(State *s,PointLandmark *landmark){
	PolerPos actual_pos = landmark->getRelativePos(s);
	if(actual_pos.distance <= body_r)
		return;
	// calcuate and give a measurement
	PolerPos pos = landmark->getMeasure(s);

	// change of weights
	double after_weight = 0.0;
	for(int i=0;i<m_num;i++){
		PolerPos ppos = landmark->getRelativePos(&m_particles[i].state);

		m_particles[i].weight *= gaussFunction(pos.distance*0.1,fabs(pos.distance - ppos.distance));
		m_particles[i].weight *= gaussFunction(10.0/180*pi,fabs(normalizeAngle(pos.direction - ppos.direction)));
		after_weight += m_particles[i].weight;

		//cerr << m_particles[i].weight << endl;
	}

	//cerr << after_weight << endl;
	//resetting
	if(after_weight < 1.0e-5){
		sensorReset(&pos,landmark);	
		cerr << "reset" << endl;
	}

	for(int i=0;i<m_num;i++){
		m_particles[i].weight /= after_weight;
	}
}

void ParticleFilter::normalize(void){
	double sum_weight = 0.0;
	for(int i = 0;i < m_num;i++)
		sum_weight += m_particles[i].weight;

	for(int i = 0;i < m_num;i++)
		m_particles[i].weight /= sum_weight;
}

void ParticleFilter::goalAsLandmarkProc(State *s,GoalAsLandmark *landmark){

	double after_weight = 0.0;
	for(int i=0;i<m_num;i++){
		// change of weights if the particle is in the goal
		if(landmark->inGoal(&m_particles[i].state))
			m_particles[i].weight *= 1.0e-5;

		after_weight += m_particles[i].weight;
	}

/*
	//cerr << after_weight << endl;
	//resetting
	if(after_weight < 1.0e-5){
//		sensorReset(&pos,landmark);	
		cerr << "reset" << endl;
	}

	for(int i=0;i<m_num;i++){
		m_particles[i].weight /= after_weight;
	}
*/
}

void ParticleFilter::sensorReset(PolerPos *measure,PointLandmark *landmark){
	for(int i=0;i<m_num;i++){
		m_particles[i].state = landmark->sampling(measure);
		m_particles[i].weight = 1.0/m_num;
	}
}

void ParticleFilter::observation(State *s,Landmark *landmark){
	if(landmark->getType() == point)
		 pointLandmarkProc(s,(PointLandmark *)landmark);
	else if(landmark->getType() == goal)
		 goalAsLandmarkProc(s,(GoalAsLandmark *)landmark);
}

const double ParticleFilter::normalizeAngle(double ang){
	while(ang > pi || ang < -pi){
		if(ang > pi)	ang -= pi*2;
		else		ang += pi*2;
	}
	return ang;
}

double ParticleFilter::getGaussRand(){
	double tmp = 0.0;
	for(int i=0;i<12;i++)
		tmp += getDoubleRand();
	
	return tmp - 6.0;
} 

double ParticleFilter::gaussFunction(double sigma,double diff){
	double h = - (diff * diff) / (2 * sigma * sigma);
	return exp(h) / (sigma * (sqrt(2*pi)));
}
