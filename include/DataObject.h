#include <iostream>
#include <vector>
#include <random>
#include <string>
#include <cmath>
#include <functional>
#include "Eigen/Dense"
#include "Eigen/Cholesky"

#ifndef _DATAOBJECT_H_
#define _DATAOBJECT_H_
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::LLT;

typedef VectorXd (*user_function) (double t); 

class ProblemConfig{
 public:
  ProblemConfig(){};
  ProblemConfig(int n_dof);
  virtual MatrixXd computeStiffness();
  virtual MatrixXd computeDamping();
  virtual MatrixXd computeMass();
  virtual VectorXd computeExtLoad(double t);
  virtual void updateParams(vector<double> p)=0;
  int size();
  int nn();
 protected:
  int n;
};

/* 
 *  Class to define a n dofs mass-spring system
 */

class MassSpring: public ProblemConfig{
 public:
  MassSpring(int ndof);
  void setMass(double);
  void setStiffness(double);
  //void setUserLoad(user_function );
  void setUserLoad( std::function<VectorXd(double)>& );
  void updateParams(vector<double> p);
  MatrixXd computeStiffness();
  MatrixXd computeMass();
  VectorXd computeExtLoad(double t);
 protected:
  double stiff, mass;
  //user_function ext_load;
  std::function<VectorXd(double)> ext_load;
};

/*
 *  Class to define aero elastic problem
 */

class AeroElastic: public ProblemConfig{
 public :
  AeroElastic();
  void setStiffness(double kh, double ka);
  void setMass(double m);
  void setDamping(double c);
  void setGeometry(double l, double xf, double xc, double c, double b, double e);
  void setAeroDyn(double u, double rho);
  void updateParams(vector<double> p);
  MatrixXd computeStiffness();
  MatrixXd computeDamping();
  MatrixXd computeMass();
  void setInit(VectorXd q, VectorXd dq);
  
 protected:
  double kh, ka;
  double m;
  double ch;
  double c, b, e;
  double l, xf, xc;
  double u, rho;
  VectorXd q_init, dq_init;
};


class TimeDis{
 public:
  TimeDis();
  void setTimeInterval(double tinit, double tend);
  void setTimeStep(int n_step);
  double eval(int i);
  int getNstep();
  int size();
  double stepSize();
  vector<double> getVals();

 private:
  double t_0;
  double t_N;
  double step_size;
  int n_step;
  vector<double> time_vals;
  void compute();
};


/*
 *  Class for observations simulations
 */

class SensorObservations{
 public:
  SensorObservations(TimeDis*, MatrixXd&);
  void setObservationsTimeDis(TimeDis* time);
  void setMeasuredDofIds(vector<int>);
  void setNoiseLevel(double);  
  MatrixXd extract();
 private:
  double noise_level;
  bool isInteger(double);
  vector<int> dofs_id;
  vector<int> step_id;
  TimeDis* time_obs;
  TimeDis* time_simu;
  MatrixXd data;
  bool AreCorrupted;
  normal_distribution<double> noise_generator;
};



#endif
