

#include "point.h"

default_random_engine generator;
normal_distribution<double> distribution (0, 0.1);


int maxiterations=10;
int num = 2;
int CROSSBOUND = 100;
double majorAxis = 0.75;
double minorAxis = 0.75;
double PI = 3.14159;
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

typedef pair<Point, int> CrossData;
typedef pair<double, double> Pair;

int currentGaussian1;

/*
double gaussian (Point input, int gaussianId)
{
    double X = input.x - gaussianCenter[gaussianId].x;
    double Y = input.y - gaussianCenter[gaussianId].y;
    double varX = gaussianVar[gaussianId].x;
    double varY = gaussianVar[gaussianId].y;
    return exp ( - (X * X / (2*varX)) - (Y * Y/(2*varY)));
}

vector<double> getGaussian (vector<Point> points)
{
    vector<double> fval;
    
    for (int i = 0; i < points.size(); i ++)
    {
        double sum = 0;
        for (int j = 0; j < gaussianCenter.size(); j++)
            sum = sum + gaussian (points[i], j);
        fval.push_back (sum);
    }
    
    return fval;
}

double getGaussian (Point point)
{
    double fval;
    double sum = 0;
    for (int j = 0; j < gaussianCenter.size(); j++){
        sum = sum + gaussian (point, j);
    } 
    fval = sum;
    return fval;
}


vector<double> get_gradient (vector<vector<double> > A, vector<double> x, vector<double> b)
{
    vector<double> gradient;
    int m = A.size();
    int n = x.size();
    
    for (int i = 0 ; i < n; i++)
    {
        vector<double> row;
        for (int j  = 0; j < m; j ++)
            row.push_back (A[j][i]);
        vector<double> temp;
        temp = get (prod(A,x),b);
        double val = 2 * dot_prod (row, temp);
        gradient.push_back (val);
    }
    
    return gradient ;
}
*/
