#include <string>
#include <iostream>
#include "Eigen/Dense"
#include "DataObject.h"


#ifndef _NEWMARK_H_
#define _NEWMARK_H_

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Newmark{
 public:
  Newmark(ProblemConfig*, TimeDis* );
  void solve();
  void setInit(VectorXd u, VectorXd du, VectorXd ddu);
  void newmarkOpt(double, double);
  MatrixXd getSolution();
  MatrixXd getSolution(int i);
  
 private:
  MatrixXd solution_u, solution_du, solution_ddu;
  ProblemConfig* pb_conf;
  TimeDis* time;
  void initialize();
  MatrixXd newmark_operator(MatrixXd& stiffness, MatrixXd& damping, MatrixXd& mass);
  void newmark_step(MatrixXd& newmark_ope, MatrixXd& damping, MatrixXd& mass, int step);
  double alpha, beta;
};

#endif
