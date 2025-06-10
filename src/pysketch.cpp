#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;


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

#include "logging.h"
#include "kitchen_sync.h"

void set_global_defaults()
{
    gaussianCenter.clear();
    gaussianVar.clear();

    epsilon = 0.005;
    INF = 4*epsilon;

    //initial direction
    alpha = 1.47; // slightly off pi/2 to try to cause early crossing
    //number of gaussians
    num = 4;
    //min number of crossings before checking termination condition
    CROSSBOUND = 100;

    majorAxis = 0.25;
    minorAxis = 0.25;
    //distance between drones-ish
    DIST = sqrt (49);
    THRESHOLD = exp (-majorAxis*majorAxis);
   
    drone_start_B = Point (1 + DIST*epsilon*0.5,-2.38);
    drone_start_A = Point (1 - DIST*epsilon*0.5,-2.38);

    drone_start_BB = Point (1 + DIST*epsilon*0.5,-2.38);
    drone_start_AB = Point (1 - DIST*epsilon*0.5,-2.38);
    
    // data/gaussian_contours_data_7.txt  
    gaussianCenter.push_back(Point(1,1));
    gaussianCenter.push_back(Point(-2,0));
    gaussianCenter.push_back(Point(1,0));
    gaussianCenter.push_back(Point(4,0));
    
    gaussianVar.push_back(Point(0.6,0.6));
    gaussianVar.push_back(Point(0.8,1.0));
    gaussianVar.push_back(Point(0.9,1.8));
    gaussianVar.push_back(Point(0.5,0.5));

    cout<<"Initializing test... "<<endl;
    cout<<"Epsilon "<<epsilon<<endl;
    cout<<"Initial direction "<<alpha<<endl;
    cout<<"Number of Gaussians "<<num<<endl;
    cout<<"Least difference between starting and end point "<<INF<<endl;
    cout<<"Minimum crossings before checking termination "<<CROSSBOUND<<endl;
    cout<<"Minimum distance factor between drones and minimum distance "<<DIST<<" " <<DIST * epsilon<<endl;
    cout<<"Concentration THRESHOLD "<<THRESHOLD<<endl;
    cout<<"Drone A starting point "<<drone_start_A.x<<" "<<drone_start_B.y<<endl;
    cout<<"Drone B starting point "<<drone_start_B.x<<" "<<drone_start_A.y<<endl;
    
    cout<<"Centers of gaussians "<<endl;
    
    for (int i =0 ; i < num; i ++)
        cout<<gaussianCenter[i].x<<" "<<gaussianCenter[i].y<<endl;
        
}

int main0()
{
    int i = 0;
    // 0.005 originally
    // for testing purposes making this larger and looping only once
    // for (epsilon = 0.005; i < 20 ; epsilon += 0.001)
    for (epsilon = 0.01; i < 1 ; epsilon += 0.001){
        INF = 4 * epsilon;
        
        //test_infrastructure ();//hack
        
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


PYBIND11_MODULE(pysketch, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    
    m.def("main0", &main0, "A function that runs sketch");
    m.def("set_global_defaults", &set_global_defaults, "A function that runs the setup phase of the simulation");
    m.def("print_test_infrastructure_info", &print_test_infrastructure_info, "A function that prints simulation parameter info");
    m.def("sketch", &sketch_algorithm, "A function that runs sketch");

    //define this so that we can look at thee gaussians
    py::class_<Point>(m,"Point")
      .def(py::init<double,double>())
      .def("x",&Point::getX,"point.x")
      .def("y",&Point::getY,"point.y");


    //getters
    m.def("get_crossbound",[](){return CROSSBOUND;},"A function that gets CROSSBOUND");
    m.def("get_num",[](){return num;},"A function that gets the number of gaussians");
    m.def("get_threshhold",[](){return THRESHOLD;},"A function that gets THRESHOLD");
    m.def("get_epsilon",[](){return epsilon;},"A function that gets epsilon");
    m.def("get_alpha",[](){return alpha;},"A function that gets alpha");
    m.def("get_dist",[](){return DIST;},"A function that gets dist");
    m.def("get_majoraxis",[](){return majorAxis;},"A function that gets majorAxis");
    m.def("get_minoraxis",[](){return minorAxis;},"A function that gets minorAxis");


    //gaussian control
    m.def("clear_gaussians",[](){
        gaussianCenter.clear();
        gaussianVar.clear();
        num = 0;
    },"A function that clears the vector of gaussians");

    m.def("get_gaussian_centers",[](){return gaussianCenter;},"A function that returns the vector of gaussian center points");
    
    m.def("get_gaussian_vars",[](){return gaussianVar;},"A function that returns the vector of gaussian variances(or is it STD?)");

    m.def("push_gaussian",[](double x, double y, double varx, double vary){

        gaussianCenter.push_back(Point(x,y));
        gaussianVar.push_back(Point(varx,vary));
        ++num;
        },"A function to add another gaussian");

    //setters

    m.def("set_crossbound",[](int c){CROSSBOUND=c;},"A function that sets CROSSBOUND");
    m.def("set_num",[](int c){num=c;},"A function that sets the number of gaussians");
    m.def("set_threshhold",[](double d){THRESHOLD=d;},"A function that sets THRESHOLD");
    m.def("set_epsilon",[](double c){epsilon=c;},"A function that sets epsilon");
    m.def("set_alpha",[](double c){alpha=c;},"A function that sets alpha");
    m.def("set_dist",[](double c){DIST=c;},"A function that sets dist");
    m.def("set_majoraxis",[](double d){majorAxis=d;},"A function that sets majorAxis");
    m.def("set_minoraxis",[](double d){minorAxis=d;},"A function that sets minorAxis");

    m.def("set_drone_start_pos",[](double x, double y){
        drone_start_B = Point (x + DIST*epsilon*0.5,y);
        drone_start_A = Point (x - DIST*epsilon*0.5,y);

        drone_start_BB = Point (x + DIST*epsilon*0.5,y);
        drone_start_AB = Point (x - DIST*epsilon*0.5,y);
        
        },"A function that sets both drones initial position");

    m.def("fout",[](const char* fname){

        fclose(out);
        FILE* out = fopen(fname,"w");
        },"set output file location");
}
