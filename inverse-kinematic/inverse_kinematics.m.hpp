// Automatically translated using m2cpp 2.0 on 2023-01-03 00:33:04

#ifndef INVERSE_KINEMATICS_M_HPP
#define INVERSE_KINEMATICS_M_HPP

#include "inverse_kinematics.m.hpp"
#include <armadillo>

using namespace arma ;

arma::mat inverse_kinematics() ;
void f() ;

arma::mat inverse_kinematics(
  arma::Mat<double> P, 
  arma::Mat<double> B,
  arma::Mat<double> T,
  double yaw, double elev, double roll)
{
  double cy = std::cos(yaw), ce = std::cos(elev), cr = std::cos(roll);
  double sy = std::sin(yaw), se = std::sin(elev), sr = std::sin(roll);

  arma::Mat<double> R_z = {
      {cy, -sy, 0.0},
      {sy, cy, 0.0},
      {0.0, 0.0, 1.0}
  };

  arma::Mat<double> R_y = {
      {ce, 0.0, se},
      {0.0, 1.0, 0.0},
      {-se, 0.0, ce}
  };

  arma::Mat<double> R_x = {
      {1.0, 0.0, 0.0},
      {0.0, cr, -sr},
      {0.0, sr, cr}
  };

  arma::Mat<double> R_bp = R_z*R_y*R_x;
  arma::Mat<double> Q = arma::zeros<mat>(6, 3);
  arma::Mat<double> rods = arma::zeros<vec>(6) ;
  for (int i=0; i<6; i++)
  {
    arma::Mat<double> v = P.rows(i, i) ;
    v(0, 2) = 0 ;
    arma::Mat<double> q = (R_bp*arma::strans(v)).t() + T - B.rows(i, i);
    rods(i) = sqrt(dot(q, q));
  }
  return rods ;
}

void f()
{
  arma::Mat<double> B = {
    {0.99144, -0.13053, 0}, 
    {0.99144, 0.13053, 0},
    {-0.38268, 0.92388, 0},
    { -0.60876, 0.79335, 0},
    { -0.60876, -0.79335, 0},
    { -0.38268, -0.92388, 0}
  };
  arma::Mat<double> P = {
    {0.49572, -0.065263, 1},
    { 0.49572, 0.065263, 1},
    { -0.19134, 0.46194, 1},
    { -0.30438, 0.39668, 1},
    { -0.30438, -0.39668, 1},
    { -0.19134, -0.46194, 1}
  };
  arma::Mat<double> T = {0, 0, 1} ;
  std::cout<<"T: \n"<<T<<std::endl;
  // std::cout<<"B(2): \n"<<B.rows(2,2)<<std::endl;
  arma::Mat<double> rods = inverse_kinematics(P, B, T, 0.0, arma::datum::pi/4,0.0);
  std::cout<<"rods: \n"<<rods<<std::endl;
}
#endif