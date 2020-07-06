#include <iostream>
#include "planning/planning.h"

int main(int argc, char *argv[]){

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging("Planning");

    planning::Planning plan;
    if(plan.Start()){
        std::cout << "Planning Start..." << std::endl;
    }
    else {
        std::cout << "Planning Start Failed..." << std::endl;
    }

    return 0;
}
