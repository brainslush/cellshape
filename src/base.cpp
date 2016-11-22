#include "base.h"

std::vector<ofVec2d>& base::get_positions() {
	return positions;
}
std::vector<double>& base::get_parameters() {
	return parameters;
}
void base::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
	/*do nothing*/
}
