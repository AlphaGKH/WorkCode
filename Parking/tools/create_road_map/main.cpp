#include <iostream>

#include "generate_road_map/generate_road_map.h"

using namespace std;

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging("CreateRoadMap");

    tools::GenerateRoadMap map;
    map.Init();

    map.Start();

    cout << "Creating Road Map..." << endl;
    return 0;
}
