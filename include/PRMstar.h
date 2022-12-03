#pragma once
#include "Planner.h"

class PRMstar : public Planner
{
    public:
        PRMstar() : Planner(){}; 
        ~PRMstar();

        void Solve(int n);


};
