
#ifndef OMNISCIENT_UTILS_H
#define OMNISCIENT_UTILS_H
using namespace std;

void omniscient_concentration_partialxx();
void omniscient_concentration_partialyy();
void omniscient_concentration_partialxy();
void omniscient_concentration_partialyx();
void omniscient_concentration_partialx();
void omniscient_concentration_partialy();

/*
 * return [partialx(x,y), partialy(x,y)]
void omniscient_concentration_gradient(
    vector<double> gauss_centers, vector<double> gauss_vars, double x, double y, double* partialx, double* partialy);

void omniscient_concentration_hessian(
    vector<double> gauss_centers, vector<double> gauss_vars, double x, double y, double* partialx, double* partialy);

dd * */
void omniscient_critical_point_at_level();
#endif //OMNISCIENT_UTILS_H
