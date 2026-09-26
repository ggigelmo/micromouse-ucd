#include <iostream>
#include "src/MouseAgent.h"
#include "SimulatorPlatform.h"
int main() {
    std::cerr << "Running flood-fill solver..." << std::endl;
    SimulatorPlatform platform;
    MouseAgent mouseAgent(platform);
    return mouseAgent.run() == MotionResult::Ok ? 0 : 1;
}
