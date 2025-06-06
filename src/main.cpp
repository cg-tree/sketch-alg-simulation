
#include <cmath>
#include <vector>
#include <cstdio>
#include <random>
#include <string>
#include <cstring>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <map>

using namespace std;

#include "point.h"
#include "line.h"

#include "global.h"
#include "utils.h"

#include "ellipse.h"
#include "plume.h"
#include "drone.h"
#include "cubicspline.h"
#include "criticalpath.h"

#include "kitchen_sync.h"

int main()
{
    int i = 0;
    // 0.005 originally
    // for testing purposes making this larger and looping only once
    // for (epsilon = 0.005; i < 20 ; epsilon += 0.001)
    for (epsilon = 0.01; i < 1 ; epsilon += 0.001){
        INF = 4 * epsilon;
        test_infrastructure ();
        eps.push_back (epsilon);
        ++i;
    }
    
    cout <<"areas ";
    for (int i = 0;i < areas.size(); i ++)
        cout << areas[i] << " ";
    
    cout << endl;
    
    
    cout <<"lengths ";
    for (int i = 0;i < lengths.size(); i ++)
        cout << lengths[i] << " ";
    
    cout << endl;
    
    
    cout <<"angles ";
    for (int i = 0;i < angles.size(); i ++)
        cout << angles[i] << " ";
    
    cout << endl;
    
    cout <<"epsilons ";
    for (int i = 0;i < eps.size(); i ++)
        cout << eps[i] << " ";
    
    cout << endl;
    
    return 0;
}


/*
 
 EPSILON = 0.05
 Point start_a (0.479583,0.0707107);
 Point start_b (0.5,0);
 angle turned by A is  97.4447
 distance traversed by A is  10.305
 
 EPSILON = 0.01
 Point start_a (0.499199,0.0141421);
 Point start_b (0.5,0);
 
 angle turned by A is  184.588
 distance traversed by A is  10.1943
 
 EPSILON = 0.005
 Point start_a (0.4998,0.00707107);
 Point start_b (0.5,0);
 angle turned by A is  63.9424
 distance traversed by A is  2.91295
 
 EPSILON = 0.001
 Point start_a (0.499992,0.00141421);
 Point start_b (0.5,0);
 
 angle turned by A is  185.67
 distance traversed by A is  2.929

 */
