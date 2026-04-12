#include "Experiments.h"
#include <random>

// global random generator for reproducibility. 
std::mt19937 gen(42);

int main() {
    // EXP_LEARNING({0.0, 0.0}, {1.0, 1.0}, 1000);
    
    for (int i = 16; i <= 35; i++)
        EXP_ADD1(i, 2, 3);

    EXP_RAND(25, 5, 2, 3);
    EXP_RAND(25, 10, 2, 3);
    EXP_RAND(25, 15, 2, 3);
    EXP_RAND(25, 20, 2, 3);
    EXP_RAND(25, 25, 2, 3);


    EXP_RAND(35, 10, 2, 3);
    EXP_RAND(35, 20, 2, 3);
    EXP_RAND(35, 30, 2, 3);
    EXP_RAND(35, 40, 2, 3);
    EXP_RAND(35, 50, 2, 3);
    
    EXP_RAND(45, 15, 2, 3);
    EXP_RAND(45, 30, 2, 3);
    EXP_RAND(45, 45, 2, 3);
    EXP_RAND(45, 60, 2, 3);
    EXP_RAND(45, 75, 2, 3);

    EXP_ROOM(30, 2, 3);
}
