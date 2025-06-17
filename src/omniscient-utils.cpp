

#include <cassert>
#include <vector>
#include "utils.h"
#include "omniscient-utils.h"

void omniscient_concentration_partialxx(){}
void omniscient_concentration_partialyy(){}
void omniscient_concentration_partialxy(){
  //partialx * (partialy/concentration)
}
void omniscient_concentration_partialyx(){
  //partialy * (partialx/concentration)
}

void omniscient_concentration_partialx(){
  double concentration = getGaussian();
  double sum =0;
  for(){
    sum += ( gauss_centers[i].x - x )/ gauss_vars[i].x;
  }
  *partialx = concentration * sum;
}

void omniscient_concentration_partialy(){
  double concentration = getGaussian();
  double sum =0;
  for(){
    sum += ( gauss_centers[i].y - y )/ gauss_vars[i].y;
  }
  *partialy = concentration * sum;
}

/*
 * return [partialx(x,y), partialy(x,y)]
 * */
void omniscient_concentration_gradient(
    vector<double> gauss_centers,
    vector<double> gauss_vars,
    double x, double y,
    double* partialx, double* partialy){

  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, partialx);
  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, partialy);

}

void omniscient_concentration_hessian(
    vector<double> gauss_centers,
    vector<double> gauss_vars,
    double x, double y,
    double* partialxx, double* partialyy,
    double* partialxy, double* partialyx ){
  
  omniscient_concentration_partialxx(
      gauss_centers, gauss_vars, x, y, partialxx);
  omniscient_concentration_partialyy(
      gauss_centers, gauss_vars, x, y, partialyy);
  omniscient_concentration_partialxy(
      gauss_centers, gauss_vars, x, y, partialxy);
  omniscient_concentration_partialyx(
      gauss_centers, gauss_vars, x, y, partialyx);
}

void omniscient_curvature_at_point(
    vector<double> gauss_centers,
    vector<double> gauss_vars,
    double x, double y, double* curvature){
  double partialx;
  double partialy;
  double partialxx;
  double partialxy;
  double partialyx;
  double partialyy;

  omniscient_concentration_gradient(
      gauss_centers, gauss_vars, x, y,&partialx, &partialy);


  omniscient_concentration_hessian(
      gauss_centers, gauss_vars, x, y,
      &partialxx, &partialyy
      &partialxy, &partialxy);

  //get_orthogonal vector()

}

void omniscient_critical_point_at_level(){}

#ifdef TEST
void test_omniscient_concentration_partialxx(){}
void test_omniscient_concentration_partialyy(){}
void test_omniscient_concentration_partialxy(){}
void test_omniscient_concentration_partialyx(){}
void test_omniscient_concentration_partialx(){}
void test_omniscient_concentration_partialy(){}
void test_omniscient_concentration_gradient(
    vector<double> gauss_centers, vector<double> gauss_vars, double x, double y, double* partialx, double* partialy){}
void test_omniscient_concentration_hessian(
    vector<double> gauss_centers, vector<double> gauss_vars, double x, double y, double* partialx, double* partialy){}
void test_omniscient_critical_point_at_level(){}

int main(){
  test_omniscient_concentration_partialxx(){}
  test_omniscient_concentration_partialyy(){}
  test_omniscient_concentration_partialxy(){}
  test_omniscient_concentration_partialyx(){}
  test_omniscient_concentration_partialx(){}
  test_omniscient_concentration_partialy(){}
  test_omniscient_concentration_gradient(
    vector<double> gauss_centers, vector<double> gauss_vars, double x, double y, double* partialx, double* partialy){}
  test_omniscient_concentration_hessian(
    vector<double> gauss_centers, vector<double> gauss_vars, double x, double y, double* partialx, double* partialy){}
  test_omniscient_critical_point_at_level(){}
  

  return 0;
}

#endif //TEST
