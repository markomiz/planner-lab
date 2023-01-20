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

    private:
        std::string _file;
        int _planner_type;
        int _num_points;
        int _num_angles;
        float _K;
        
        int _sample_type;
        float _connect_dist;
        float _tprm_t;
        float _tprm_d;
};