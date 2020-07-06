/**
  * @projectName :  LeapAuto
  * @brief       :
  * @author      :  guokonghui
  * @date        :  2019-11-25
*/
#include <iostream>

#include "perception/perception.h"

using namespace perception;

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging("Perception");

    perception::Perception perc;
    if(perc.Start()){
        std::cout << "Perception Start..." << std::endl;
    }
    else {
        std::cout << "Perception Start Failed..." << std::endl;
    }

    return 0;
}
