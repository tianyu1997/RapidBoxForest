#pragma once

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API

#include <SBF/scene_types.h>

#include <string>
#include <vector>

namespace rbf {

struct SubtractiveObstacleGroup {
	std::string name;
	std::vector<Obstacle> carving_obstacles;
	std::vector<Obstacle> validation_obstacles;
};

struct SubtractiveBuildOptions {
	bool run_connector = true;
	bool use_validation_obstacles_for_final_scene = true;
};

} // namespace rbf

#endif
