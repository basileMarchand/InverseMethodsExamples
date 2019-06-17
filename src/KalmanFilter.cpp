#include "KalmanFilter.h"


/*
 *  Base class for Kalman filtering
 */


KalmanFilter::KalmanFilter(Operator* model, Operator* observer, TimeDis* time){
  this->model = model;
  this->observer = observer;
  this->time = time;
}

void KalmanFilter::setModelError(MatrixXd mod_error){
  this->model_error = mod_error;
}

void KalmanFilter::setObsError(MatrixXd obs_error){
  this->obs_error = obs_error;
}

void KalmanFilter::setInitState(VectorXd init_mean, MatrixXd init_cov ){
  this->state.col(0) = init_mean;
  this->covariance[0] = init_cov;
}

void KalmanFilter::initialize(){
  this->state = MatrixXd::Zero(this->model->size(),this->time->size());
  this->covariance.resize(this->time->size());
}

MatrixXd KalmanFilter::getState(){
  return this->state;
}

vector<MatrixXd> KalmanFilter::getCovariance(){
  return this->covariance;
}

VectorXd KalmanFilter::getState(int step){
  return this->state.col(step);
}

void KalmanFilter::run(MatrixXd obs_data){
  VectorXd obs_k;
  
  for(int i=1;i<this->time->size();i++){
    obs_k = obs_data.col(i);
    this->doStep(i, obs_k);
  }
}

/*
 *   Linear Kalman Filter
 */

LinearKalmanFilter::LinearKalmanFilter(Operator* model, Operator* observer, TimeDis* time): KalmanFilter(model, observer, time){}

void LinearKalmanFilter::doStep(int step, VectorXd obs_data){
  VectorXd u_hat;
  MatrixXd C_hat, gain;
  MatrixXd m_ope = this->model->matrix(step);
  MatrixXd o_ope = this->observer->matrix(step);
  // prediction
  u_hat = m_ope * this->state.col(step-1) ;
  C_hat = m_ope * this->covariance[step-1] * m_ope.transpose() + this->model_error;
  // kalman gain
  gain = C_hat * o_ope.transpose() * ( o_ope * C_hat * o_ope.transpose() + this->obs_error ).inverse();
  // correction
  this->state.col(step) = u_hat + gain * ( obs_data - o_ope * u_hat);
  this->covariance[step] = C_hat - gain * o_ope * C_hat;
}


/*
 *    Extended Kalman Filter 
 */ 

ExtendedKalmanFilter::ExtendedKalmanFilter(Operator* model, Operator* observer, TimeDis* time): KalmanFilter(model, observer, time){}

void ExtendedKalmanFilter::setLinearizationParam(int order, double step){
  this->diff_order = order;
  this->diff_step = step;
}

void ExtendedKalmanFilter::doStep(int step, VectorXd obs_data){
  VectorXd u_hat;
  MatrixXd C_hat, gain;
  MatrixXd m_ope, o_ope;
  // Compute linearization of operators if needed
  if(this->model->has_jacobian()){
    m_ope = this->model->matrix(step-1);
  }
  else{
    m_ope = this->linearization(this->model, this->state.col(step-1), step-1);
  }

  if(this->observer->has_jacobian()){
    o_ope = this->observer->matrix(step);
  }
  else{
    o_ope = this->linearization(this->observer, this->state.col(step-1), step-1);
  }
  // prediction
  u_hat = this->model->eval( step-1, this->state.col(step-1) ) ;
  C_hat = m_ope * this->covariance[step-1] * m_ope.transpose() + this->model_error;
  // kalman gain
  gain = C_hat * o_ope.transpose() * ( o_ope * C_hat * o_ope.transpose() + this->obs_error ).inverse();
  // correction
  VectorXd tmp = this->observer->eval(step, u_hat);
  this->state.col(step) = u_hat + gain * ( obs_data - tmp );
  this->covariance[step] = C_hat - gain * o_ope * C_hat;
}

MatrixXd ExtendedKalmanFilter::linearization(Operator * ope, VectorXd x, int step){
  int m = ope->size();
  int n = x.size();
  MatrixXd jacobian = MatrixXd::Zero(m,n);
  if (this->diff_order == 1){
    VectorXd f = ope->eval(step, x);
    VectorXd dx, df;
    for(int j=0;j <n; j++){
      dx = VectorXd::Zero(n);
      dx(j) = x(j) * this->diff_step;
      if(x(j)<0.0000000000001){
	dx(j) = this->diff_step;
      }
      df = ope->eval(step, x + dx);
      jacobian.col(j) = ( df - f ) / (dx(j));
    }
  }
  if (this->diff_order == 2){
    VectorXd df1, df2; 
    VectorXd dx;
    for(int j=0;j <n; j++){
      dx = VectorXd::Zero(n);
      dx(j) = x(j) * this->diff_step;
      df1 = ope->eval(step, x + dx);
      df2 = ope->eval(step, x - dx);
      jacobian.col(j) = ( df1 - df2 ) / (2 * x(j) * this->diff_step);
    }
  }
  return jacobian;
}


/*
 * Unscented Kalman Filter 
 */ 

UnscentedKalmanFilter::UnscentedKalmanFilter(Operator* model, Operator* observer, TimeDis* time): KalmanFilter(model, observer, time){}


void UnscentedKalmanFilter::setUnscentedParam(double alpha, double beta, double kappa){
  this->alpha = alpha;
  this->beta = beta;
  this->kappa = kappa;
}

void UnscentedKalmanFilter::doStep(int step, VectorXd obs_data){
  // Compute sigma-points
  vector<VectorXd> sigm_x = this->computeSigmaPoint(this->state.col(step-1), this->covariance[step-1]);
  vector<double>   sigm_w_m = this->computeSigmaWeightMean(sigm_x[0].size());
  vector<double>   sigm_w_c = this->computeSigmaWeightCov(sigm_x[0].size());
  // Propagate sigma-points through ithe model operator
  vector<VectorXd> sigm_x_hat = this->propagate(this->model, sigm_x, step);
  // Reconstruct mean and variance-covariance matrix
  VectorXd x_hat = this->reconstructMean(sigm_x_hat, sigm_w_m);
  MatrixXd C_hat = this->reconstructCov(x_hat, sigm_x_hat, x_hat, sigm_x_hat, sigm_w_c);
  C_hat += this->model_error;
  // Propagate sigma point through the observer
  vector<VectorXd> sigm_y = this->propagate(this->observer, sigm_x_hat, step);
  VectorXd y_mean = this->reconstructMean(sigm_y, sigm_w_m);
  // mixed covariance matrix
  MatrixXd cov_yy = this->reconstructCov(y_mean, sigm_y, y_mean, sigm_y, sigm_w_c);
  cov_yy += this->obs_error;
  MatrixXd cov_xy = this->reconstructCov(x_hat, sigm_x_hat, y_mean, sigm_y, sigm_w_c);
  // Kalman gain 
  MatrixXd gain = cov_xy * cov_yy.inverse();
  // correction
  this->state.col(step) = x_hat + gain * ( obs_data - y_mean );
  this->covariance[step] = C_hat - gain * cov_yy * gain.transpose();
}


vector<VectorXd> UnscentedKalmanFilter::propagate(Operator* ope, vector<VectorXd>& sigm_x, int step){
  vector<VectorXd> output(sigm_x.size());
  VectorXd tmp;
  for(int i=0; i<sigm_x.size(); i++){
    tmp = ope->eval(step, sigm_x[i]);
    output[i] = tmp;
  }
  return output;
} 


vector<VectorXd> UnscentedKalmanFilter::computeSigmaPoint(VectorXd mean, MatrixXd cov){
  
  int n = mean.size();
  vector<VectorXd> sigm(2*n+1);
  sigm[0] = mean;
  double lambda = this->alpha*this->alpha * ( n + this->kappa ) - n;
  double coef = n + lambda;
  LLT<MatrixXd> chol(  coef * cov);
  MatrixXd chol_L = chol.matrixL();
  for(int i=0; i<n; i++){
    sigm[1+i] = mean +  chol_L.col(i)  ;
    sigm[1+n+i] = mean - chol_L.col(i) ;
  }
  return sigm;
}

vector<double> UnscentedKalmanFilter::computeSigmaWeightMean(int n){
  vector<double> sigm_w(2*n+1);
  double lambda = this->alpha*this->alpha * ( n + this->kappa ) - n;
  sigm_w[0] = lambda / (n + lambda);
  for(int i=0;i<n;i++){
    sigm_w[1+i] = 1. / (2.*(n + lambda));
    sigm_w[1+n+i] = 1. / (2*(n + lambda));
  }
  return sigm_w;
}

vector<double> UnscentedKalmanFilter::computeSigmaWeightCov(int n){
  vector<double> sigm_w(2*n+1);
  double lambda = this->alpha*this->alpha * ( n + this->kappa ) - n; 
  sigm_w[0] = lambda / (n + lambda) + 1 - this->alpha*this->alpha + this->beta;
  for(int i=0;i<n;i++){
    sigm_w[1+i] = 1. /(2.*(n + lambda)) ;
    sigm_w[1+n+i] = 1. / (2.*(n + lambda)) ;
  }
  return sigm_w;
}

VectorXd UnscentedKalmanFilter::reconstructMean(vector<VectorXd>& sigm_x, vector<double>& sigm_w){
  VectorXd mean = VectorXd::Zero(sigm_x[0].size());
  for(int i=0; i<sigm_x.size(); i++){
    mean += sigm_w[i] * sigm_x[i];
  }
  return mean;
}

MatrixXd UnscentedKalmanFilter::reconstructCov(VectorXd mean_x, vector<VectorXd>& sigm_x, VectorXd mean_y, vector<VectorXd>& sigm_y,  vector<double>& sigm_w){
  int m = sigm_x[0].size();
  int n = sigm_y[0].size();
  MatrixXd cov = MatrixXd::Zero(m,n);
  for(int i=0; i<sigm_x.size(); i++){
    cov += sigm_w[i] * ( sigm_x[i] - mean_x ) * ( sigm_y[i] - mean_y ).transpose();
  }
  return cov;
}




