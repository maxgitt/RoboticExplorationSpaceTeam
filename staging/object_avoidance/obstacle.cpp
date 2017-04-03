// Obstacle object. Will be published. Contains x location, y location, approximate width, and approximate height

#include <"obstacle.h">

Obstacle::Obstacle(){
	x = -1;
	y = -1;
	width = -1;
	height = -1;
}

Obstacle::Obstacle(int x_in, int y_in, int width_in, int length_in){
	x = x_in;
	y = y_in;
	width = width_in;
	length = height_in;
}
