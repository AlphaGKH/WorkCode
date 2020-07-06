#include <iostream>

#include "display/display.h"

int main(int argc, char *argv[]){

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging("Display");

    display::Display displayer;
    if(displayer.Start()){
        std::cout << "Display Start..." << std::endl;
    }
    else {
        std::cout << "Display Start Failed..." << std::endl;
    }


    return 0;
}
