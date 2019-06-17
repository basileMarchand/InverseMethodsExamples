#include "DataObject.h"

#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

/*
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(MatrixXd)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(VectorXd)
*/
/*
 *   Pure virtual class definition for the Model and Observer objects
 */

class Operator{
 public:
  Operator(TimeDis* time_dis){this->time=time_dis;}
  virtual VectorXd eval(int step)=0;
  virtual VectorXd eval(int step, VectorXd x)=0;
  virtual MatrixXd matrix(int step)=0;
  bool has_jacobian(){return this->JACOBIAN;};
  int size(){return this->n;};
 protected:
  TimeDis* time;
  int n;
  bool JACOBIAN=false;
};

/*
 *  Kalman filtering definitions 
 *    - Linear Kalman filter
 *    - Extended Kalman Filter
 *    - Unscented Kalman Filter
 */

class KalmanFilter{
 public:
  KalmanFilter(Operator*, Operator*, TimeDis*);
  void initialize();
  void setModelError(MatrixXd);
  void setObsError(MatrixXd);
  void setInitState(VectorXd, MatrixXd);
  void run(MatrixXd);
  virtual void doStep(int step, VectorXd obs)=0;
  MatrixXd getState();
  VectorXd getState(int step);
  vector<MatrixXd> getCovariance();

 protected:
  Operator* model;
  Operator* observer;
  TimeDis* time;
  MatrixXd model_error;
  MatrixXd obs_error;
  MatrixXd state;
  vector<MatrixXd> covariance;
};

// Linear Kalman Filter 
class LinearKalmanFilter: public KalmanFilter{
 public:
  LinearKalmanFilter(Operator*, Operator*, TimeDis*);
  void run(MatrixXd);
  void doStep(int step, VectorXd obs);
};


// Extended Kalman Filter
class ExtendedKalmanFilter: public KalmanFilter{
 public:
  ExtendedKalmanFilter(Operator*, Operator*, TimeDis*);
  //void run(MatrixXd);
  void doStep(int step, VectorXd obs);
  void setLinearizationParam(int order, double step);
  
 protected:
  MatrixXd linearization(Operator*, VectorXd, int);
  double diff_step;
  int diff_order;
  
};

// Unscented Kalman Filter 
class UnscentedKalmanFilter: public KalmanFilter{
 public:
  UnscentedKalmanFilter(Operator*, Operator*, TimeDis*);
  //void run(MatrixXd);
  void doStep(int step, VectorXd obs);
  void setUnscentedParam(double, double, double);
  
 protected:
  vector<VectorXd> computeSigmaPoint(VectorXd mean, MatrixXd cov);
  vector<double> computeSigmaWeightMean(int n);
  vector<double> computeSigmaWeightCov(int n);
  vector<VectorXd> propagate(Operator*, vector<VectorXd>&, int);
  VectorXd reconstructMean(vector<VectorXd>&, vector<double>&);
  MatrixXd reconstructCov(VectorXd, vector<VectorXd>&, VectorXd, vector<VectorXd>&,  vector<double>&); 
  double alpha, beta, kappa;
};


#endif 
