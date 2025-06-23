#include <cassert>
#include <cstdio>
#include <vector>
using namespace std;
#include "point.h"
//#include "omniscient_utils.h"

/*
 * vectors a and b are orthogonal if a.b=0
 * we have 2 degrees of freedom so we choose a
 * new coordinate st magnitude of the denominator
 * in the problem is maximized which minimizes error propagation
 * */
static void get_orthogonal_vector(double x, double y, double* nx, double* ny){
  int magnitude = (x != 0) + (y != 0);
  if (magnitude == 0){
    *nx = 0;
    *ny = 0;
    return;
  }
  int max = abs(y) > abs(x);
  if(max){
    *nx = x;
    *ny = (-1 * x * *nx) / y;
  }
  else{
    *ny = y;
    *nx = (-1 * y * *ny) / x;
  }
}

double omniscient_concentration(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y){

  int len = gauss_centers.size();
  assert( len == gauss_vars.size() );
  
  double concentration = 0;
  for( int i=0; i<len; ++i ){
    assert( (gauss_vars[i].x != 0) && (gauss_vars[i].y != 0) );
    double dx = x - gauss_centers[i].x;
    double dy = y - gauss_centers[i].y;
    double vx = 2 * gauss_vars[i].x;
    double vy = 2 * gauss_vars[i].y;

    concentration += exp( -( ((dx*dx)/vx) + ((dy*dy)/vy) ) );
  }
  return concentration;
}


void omniscient_concentration_partialx(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y,
    double* partialx){

  int len = gauss_centers.size();
  assert( len == gauss_vars.size() );
  
  double concentration = omniscient_concentration(
      gauss_centers, gauss_vars, x, y);
  
  double sum =0;
  for( int i=0; i<len; ++i ){
    assert( (gauss_vars[i].x != 0) && (gauss_vars[i].y != 0) );
    sum += ( gauss_centers[i].x - x )/ gauss_vars[i].x;
  }
  *partialx = concentration * sum;
}

void omniscient_concentration_partialy(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y,
    double* partialy){

  int len = gauss_centers.size();
  assert( len == gauss_vars.size() );

  double concentration = omniscient_concentration(
      gauss_centers, gauss_vars, x, y);
  
  double sum =0;
  for( int i=0; i<len; ++i ){
    assert( (gauss_vars[i].x != 0) && (gauss_vars[i].y != 0) );
    sum += ( gauss_centers[i].y - y )/ gauss_vars[i].y;
  }
  *partialy = concentration * sum;
}

/*
 * return [partialx(x,y), partialy(x,y)]
 * */
void omniscient_concentration_gradient(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y,
    double* partialx, double* partialy){

  int len = gauss_centers.size();
  assert( len == gauss_vars.size() );

  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, partialx);
  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, partialy);

}

void omniscient_concentration_partialxx(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y,
    double* partialxx){

  int len = gauss_centers.size();
  assert( len == gauss_vars.size() );
  
  double concentration = omniscient_concentration(
      gauss_centers, gauss_vars, x, y);
  
  assert( concentration > 0 );
  double varsum =0;
  for( int i=0; i<len; ++i ){
    assert( gauss_vars[i].x != 0 );
    varsum += 1/gauss_vars[i].x;
  }
  double partialx; 
  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, &partialx);
  *partialxx = (partialx * (partialx/concentration)) - (concentration*varsum);
}

void omniscient_concentration_partialyy(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y,
    double* partialyy){

  int len = gauss_centers.size();
  assert( len == gauss_vars.size() );
  
  double concentration = omniscient_concentration(
      gauss_centers, gauss_vars, x, y);
  
  assert( concentration > 0 );
  double varsum =0;
  for( int i=0; i<len; ++i ){
    assert( gauss_vars[i].y != 0 );
    varsum += 1/gauss_vars[i].y;
  }
  double partialy; 
  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, &partialy);
  *partialyy = (partialy * (partialy/concentration)) - (concentration*varsum);
}



void omniscient_concentration_partialxy(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y,
    double* partialxy){

  //partialx * (partialy/concentration)
  double concentration = omniscient_concentration(
      gauss_centers, gauss_vars, x, y);
  
  assert( concentration > 0 );
  
  double partialx;
  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, &partialx);
  double partialy;
  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, &partialy);

  *partialxy = partialx * ( partialy / concentration );
  if (*partialxy == -0) *partialxy = 0;
}

void omniscient_concentration_partialyx(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y,
    double* partialyx){
  //partialy * (partialx/concentration)
  double concentration = omniscient_concentration(
      gauss_centers, gauss_vars, x, y);

  assert( concentration > 0 );

  double partialx;
  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, &partialx);
  double partialy;
  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, &partialy);

  *partialyx = partialy * ( partialx / concentration );
  if (*partialyx == -0) *partialyx = 0;
}

void omniscient_concentration_hessian(
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
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
    vector<Point> gauss_centers,
    vector<Point> gauss_vars,
    double x, double y, double* curvature){
  double partialx;
  double partialy;
  
  omniscient_concentration_gradient(
      gauss_centers, gauss_vars, x, y, &partialx, &partialy);


  double tanx, tany;
  get_orthogonal_vector(partialx, partialy, &tanx, &tany);
  int magnitude = (tanx != 0) + (tany != 0);
  if( magnitude == 0 ){
    *curvature = 0;
    return;
  }
  double magnitudet = sqrt( (tanx * tanx) + (tany * tany) ); 
  tanx /= magnitudet;
  tany /= magnitudet;

  double norm_concentration_grad = sqrt( (partialx * partialx) + (partialy * partialy) );

  assert( norm_concentration_grad > 0 );

  double partialxx;
  double partialxy;
  double partialyx;
  double partialyy;

  omniscient_concentration_hessian(
      gauss_centers, gauss_vars, x, y,
      &partialxx, &partialyy,
      &partialxy, &partialyx);
  
  double htx = (tanx * partialxx) + (tany * partialxy);
  double hty = (tanx * partialyx) + (tany * partialyy);

  double thtx = tanx * htx;
  double thty = tany * hty;

  double tht = thtx + thty;

  *curvature = tht / norm_concentration_grad;

}

void omniscient_critical_point_at_level(){}

#ifdef TEST
void test_omniscient_critical_point_at_level(){}

/****************************************************************/
void test_omniscient_concentration_partialx(){

  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,1) );
  double partialx;

  //expect negative value
  double x = 1;
  double y = 0;

  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, &partialx);
  
  printf("partialx %f\n",partialx);
  assert(partialx <0);

  //expect positive value
  x = -1;

  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, &partialx);

  printf("partialx %f\n",partialx);
  assert(partialx >0);

  //expect zero
  x = 0;
  y = 1;

  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, &partialx);

  printf("partialx %f\n",partialx);
  assert(partialx ==0);

  //expect zero
  y = -1;

  omniscient_concentration_partialx(
      gauss_centers, gauss_vars, x, y, &partialx);

  printf("partialx %f\n",partialx);
  assert(partialx ==0);
}
/****************************************************************/
void test_omniscient_concentration_partialy(){

  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,1) );
  double partialy;

  //expect negative
  double x = 0;
  double y = 1;
  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, &partialy);
  
  printf("partialy %f\n",partialy);
  assert(partialy <0);
  
  //expect positive value
  y = -1;

  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, &partialy);

  printf("partialy %f\n",partialy);
  assert(partialy >0);

  //expect zero
  x = 1;
  y = 0;

  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, &partialy);

  printf("partialy %f\n",partialy);
  assert(partialy ==0);

  //expect zero
  x = -1;

  omniscient_concentration_partialy(
      gauss_centers, gauss_vars, x, y, &partialy);

  printf("partialy %f\n",partialy);
  assert(partialy ==0);
}

/****************************************************************/
void test_omniscient_concentration_gradient(){
  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  double x = 0;
  double y = 0;
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,1) );
  double partialx;
  double partialy;
  
  omniscient_concentration_gradient(
      gauss_centers, gauss_vars, x, y, &partialx, &partialy);
  assert( partialx == 0 );
  assert( partialy == 0 );

  printf("Test concentration gradient PASSED\n");
}
/****************************************************************/
void test_omniscient_concentration_partialxx(){
  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,1) );
  
  double x = 0;

  double partialxx;
  for(double y = -1; y <= 1; y += 1){
    omniscient_concentration_partialxx(
        gauss_centers, gauss_vars, x, y, &partialxx);

    printf("partialxx %f\n",partialxx);
    assert( partialxx < 0 );
  }

  
}
/****************************************************************/
void test_omniscient_concentration_partialyy(){
  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,1) );
  
  double y = 0;
  
  double partialyy;
  for(double x = -1; x <= 1; x += 1){
    omniscient_concentration_partialyy(
      gauss_centers, gauss_vars, x, y, &partialyy);
    printf("partialyy %f\n",partialyy);
    assert( partialyy < 0 );
  }
}
/****************************************************************/
void test_omniscient_concentration_partialxy(){
  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,1) );
  double y = 0;
  double partialxy;
  for(double x = -1; x <= 1; x += 1){
  omniscient_concentration_partialxy(
      gauss_centers, gauss_vars, x, y, &partialxy);
    printf("partialxy %f\n",partialxy);
    assert( partialxy == 0 );
  }
  
  double x = 0;
  for(double y = -1; y <= 1; y += 1){
  omniscient_concentration_partialxy(
      gauss_centers, gauss_vars, x, y, &partialxy);
    printf("partialxy %f\n",partialxy);
    assert( partialxy == 0 );
  }
}
/****************************************************************/
void test_omniscient_concentration_partialyx(){
  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,1) );
  double x = 0;
  double partialyx;
  for(double y = -1; y <= 1; y += 1){
  omniscient_concentration_partialyx(
      gauss_centers, gauss_vars, x, y, &partialyx);
    printf("partialyx %f\n",partialyx);
    assert( partialyx == 0 );
  }

  double y = 0;
  for(double x = -1; x <= 1; x += 1){
  omniscient_concentration_partialyx(
      gauss_centers, gauss_vars, x, y, &partialyx);
    printf("partialyx %f\n",partialyx);
    assert( partialyx == 0 );
  }
}
/****************************************************************/
void test_omniscient_concentration_hessian(){

  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  double x = 0;
  double y = 0;
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,1) );
  double partialxx;
  double partialxy;
  double partialyx;
  double partialyy;
  omniscient_concentration_hessian(
      gauss_centers, gauss_vars, x, y,
      &partialxx, &partialyy,
      &partialxy, &partialxy);

  printf("partialxx %f\n",partialxx);
  printf("partialxy %f\n",partialxy);
  printf("partialyx %f\n",partialyx);
  printf("partialyy %f\n",partialyy);


}

/****************************************************************/
void test_omniscient_curvature_at_point(){

  vector<Point> gauss_centers;
  vector<Point> gauss_vars;
  
  gauss_centers.push_back( Point(0,0) );
  gauss_vars.push_back( Point(1,2) );
  
  double curvaturex;

  omniscient_curvature_at_point(
    gauss_centers, gauss_vars,
    1, 0,  &curvaturex);

  double curvaturey;

  omniscient_curvature_at_point(
    gauss_centers, gauss_vars,
    0, 1,  &curvaturey);

  printf("curvature 1,0 : %f\n",curvaturex);
  printf("curvature 0,1 : %f\n",curvaturey);
  for(double i = 1; i>0; i = i-0.1){
    double c;
    omniscient_curvature_at_point(
      gauss_centers, gauss_vars,
      i,- i,  &c);
  printf("curvature %f,%f : %f\n",i,i,c);

  }
  assert( curvaturex > curvaturey );


}
/****************************************************************/

int main(){
  test_omniscient_concentration_partialx();
  
  test_omniscient_concentration_partialy();
  
  test_omniscient_concentration_gradient();
  
  test_omniscient_concentration_partialxx();

  test_omniscient_concentration_partialyy();
  
  test_omniscient_concentration_partialxy(
    );
  
  test_omniscient_concentration_partialyx(
    );
  
  test_omniscient_concentration_hessian();

  test_omniscient_curvature_at_point();
  
  test_omniscient_critical_point_at_level(
    );

  return 0;
}

#endif //TEST
