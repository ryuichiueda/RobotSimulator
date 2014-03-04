#ifndef COORDINATES_H__
#define COORDINATES_H__

struct State {double x;double y;double theta;};
struct Movement { double distance; double angle; };
struct PolerPos { double distance; double direction; };

const double pi = 3.141592;

const double goal_x = 0.0;
const double goal_y = 200.0;

const double body_r = 50.0;

#endif
