#pragma once
#include <chrono>
#include <memory>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>


class ConfigParams
{
    public:

        ConfigParams(std::string filename){setFile(filename);};
        ~ConfigParams(){};
        void setFile(std::string filename);
        void loadParams();
        void addParam(std::string name, std::string value);
        int getPlannerType();
        int getNumPoints();
        int getNumAngles();
        float getK();
        int getSampleType();
        float getConnectDist();
        float getTPRM_T();
        float getTPRM_D();
        float getExpandSize();
        float getStartEndThrsh();

    private:
        std::string _file;
        int _planner_type = 0;
        int _num_points = 1000;
        int _num_angles = 3;
        float _K = 3.0;
        int _sample_type = 0;
        float _connect_dist = 1.2;
        float _tprm_t = 0.5;
        float _tprm_d = 0.5;
        float _expand_size = 0.5;
        float _start_end_thrsh = 1.0;
};