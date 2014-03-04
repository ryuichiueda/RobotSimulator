#include "World.h"
#include "Robot.h"
#include "Coordinates.h"
#include <fstream>
using namespace std;

void print(Robot &robot,World &wld,char suffix = ' '){
	cout << "STEP " << robot.getStep() << ' ' << suffix << endl;
	robot.printState();
	robot.printGoal();
	wld.printLandmarks();
	robot.getParticleFilter()->print();
}

int main(int argc, char const* argv[])
{
	ifstream ifs("/dev/random");
	World wld(&ifs);

	Robot robot(1000.0,0.0,- pi/2,&ifs);
	robot.getParticleFilter()->resetParticles(&wld);
/*
	robot.getParticleFilter()->resetParticles(1000.0,0.0,-pi/2,
						200.0,20.0/180.0 * pi);
*/

	//ロボットへランドマーク情報を渡す
	robot.registerLandmarks(wld.getLandmarks());

	while(!robot.isInGoal() && robot.getStep() <= 500){

		print(robot,wld);

		bool resampling = false;
		robot.observeLandmark(1);
		if(robot.getStep()%5 == 0){
			robot.observeLandmark(0);
			print(robot,wld,'p');
			resampling = true;
		}

		robot.step(resampling);
	}

	ifs.close();
	exit(0);
}

