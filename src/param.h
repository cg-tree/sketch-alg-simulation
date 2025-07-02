#ifndef PARAM_H
#define PARAM_H

typedef struct Param_s{
default_random_engine generator;

int maxiterations=100;
int num = 2;
int CROSSBOUND = 100;
double majorAxis = 0.75;
double minorAxis = 0.75;
double DIST = 36;
double THRESHOLD = exp (-majorAxis*majorAxis);
double alpha;
double epsilon, INF;
double varX, varY;

vector<double> areas, lengths, angles, eps;
FILE *out = fopen ("../sketch_plot.txt", "w");


Point drone_start_B, drone_start_A, drone_start_AB, drone_start_BB;
vector<Point> gaussianCenter;
vector<Point> gaussianVar;


int currentGaussian1;
} Param;

#endif //PARAM_H
