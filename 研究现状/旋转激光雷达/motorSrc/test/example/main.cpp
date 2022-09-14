#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>
#include <regex>
#include "motor/RMDS.h"

int main(int argc, char** argv) {
    auto dev = Motor::RMDS::findDev(1);
    if(dev.empty())
    {
        std::cerr << "cannot find motor device" <<std::endl;
        exit(-1);
    }

    Motor::RMDS m(dev, 1);

    std::vector<double> params;
    std::vector<std::string> args;
    for(int i = 1; i < argc; i++)
    {
        args.push_back(std::string(argv[i]));
    }

    if(args[0]=="-t")
    {
        if(argc == 3)
            m.rotateTo(std::stod(args[1]));
        else
            m.rotateTo(std::stod(args[1]), std::stod(args[2]));
    }
    else if(args[0]=="-m")
    {
        if(argc == 3)
            m.rotateMore(std::stod(args[1]));
        else
            m.rotateMore(std::stod(args[1]), std::stod(args[2]));
    }else if(args[0]=="-r")
    {
        if(argc == 3)
            m.rotate(std::stod(args[1]));
        else
            m.rotate();
    }else if(args[0]=="-p")
    {
        m.pause();
    }else if (args[0] == "-g")
    {
        // auto start = std::chrono::system_clock::now();
        // for (size_t i = 0; i < 1000; i++)
        // {
        //    m.getCurrentPose();
        // }
        // auto end = std::chrono::system_clock::now();
        // auto time = std::chrono::duration<double>(end - start).count();
        // std::cout << "total cost: " << time << std::endl;
        // std::cout << "echo freq: " << 1000 / time << "hz" << std::endl;
        std::cout << m.getCurrentPose() << std::endl;
    }

        return 0;
}