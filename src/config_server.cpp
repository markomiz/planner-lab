
#include <chrono>
#include <memory>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include "config_server.h"

using namespace std;

void ConfigParams::setFile(string filename)
{
    _file = filename;
    loadParams();
};
void ConfigParams::loadParams()
{
    
    ifstream file(_file);
    string str; 
    while (getline(file, str))
    {
        
        vector<string> results;
        boost::split(results, str, [](char c){return c == ' ';});
        if (results.size() < 2) continue;
        string name = results[0];
        string value = results[1];
        addParam(name, value);
    }
    file.close();
};
void ConfigParams::addParam(string name, string value)
{

    if (name ==  "planner_type:"){
        if (value == "DPRMstar")
        {
            _planner_type = "DPRMstar";
        }
        else if (value == "GeometricPRMstar")
        {
            _planner_type = "GeometricPRMstar";
        }
        else
        {
            _planner_type = "ExactCell";
        }
        cout << "got planner! " << _planner_type << " \n";
    }
    else if (name ==  "num_points:"){
        _num_points = stoi(value);
        cout << "got num points! " << _num_points << " \n";
    }
    else if (name ==  "num_angles:"){
        _num_angles = stoi(value);
        cout << "got num angles! " << _num_angles<< " \n";
    }
    else if (name ==  "K:"){
        _K = stof(value);
    }
    else if (name ==  "sample_type:"){
        _sample_type = (value == "Uniform") ? 0 : 1;
    }
    else if (name ==  "connect_distance:"){
        _connect_dist = stof(value);
        cout << "got connect_dist! " << _connect_dist << " \n";
    }
    else if (name ==  "tprm_t:"){
        _tprm_t = stof(value);
    }
    else if (name ==  "tprm_t:"){
        _tprm_d = stof(value);
    }
    else if (name ==  "expand_distance:"){
        _expand_size = stof(value);
    }
        else if (name ==  "start_end_thrsh:"){
        _start_end_thrsh = stof(value);
    }   
};
std::string ConfigParams::getPlannerType()
{
    return _planner_type;
};
int ConfigParams::getNumPoints()
{
    return _num_points;
};
int ConfigParams::getNumAngles()
{
    return _num_angles;
};
float ConfigParams::getK()
{
    return _K;
};
int ConfigParams::getSampleType()
{
    return _sample_type;
};
float ConfigParams::getConnectDist()
{
    return _connect_dist;
};
float ConfigParams::getTPRM_T()
{
    return _tprm_t;
};
float ConfigParams::getTPRM_D()
{
    return _tprm_d;
};
float ConfigParams::getExpandSize()
{
    return _expand_size;
}
float ConfigParams::getStartEndThrsh()
{
    return _start_end_thrsh;
}