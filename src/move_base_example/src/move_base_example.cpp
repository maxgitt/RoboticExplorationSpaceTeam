
#include "move_base_example.h"


int main(int argc, char **argv){
	MoveBase ex(argc, argv, 0, 0, 0);
	ex.move_to(4,4);
	return 0;
}
