#include <iostream>

#include "src/MouseAgent.h"

int main() {
    std::cerr << "Running flood-fill solver..." << std::endl;
    MouseAgent mouseAgent;
    mouseAgent.exploreToGoal();
    return 0;
}
