#ifndef GLOBAL_H
#define GLOBAL_H


default_random_engine generator;

typedef pair<Point, int> CrossData;
typedef pair<double, double> Pair;

const double PI = 3.14159;
#ifndef LEGACY
Param param;

#endif // LEGACY
#ifdef LEGACY
normal_distribution<double> distribution (0, 0.1);

int maxiterations=10;
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
#endif // LEGACY
#endif //GLOBAL_H
