#include "Newmark.h"


Newmark::Newmark(ProblemConfig* pb, TimeDis* time){
  this->pb_conf = pb;
  this->time = time;
  this->initialize();
}

void Newmark::initialize(){
  int n_dof =  this->pb_conf->nn();
  int n_step = this->time->getNstep();
  this->solution_u =  MatrixXd::Zero(n_dof,n_step);
  this->solution_du = MatrixXd::Zero(n_dof,n_step);
  this->solution_ddu = MatrixXd::Zero(n_dof,n_step);
}

void Newmark::setInit(VectorXd u, VectorXd du, VectorXd ddu){
  this->solution_u.col(0) = u;
  this->solution_du.col(0) = du;
  this->solution_ddu.col(0) = ddu;
}

void Newmark::newmarkOpt(double alpha, double beta){
  this->alpha = alpha;
  this->beta  = beta;
}

void Newmark::solve(){
  int n_dof = this->pb_conf->nn();
  MatrixXd stiffness(n_dof,n_dof), mass(n_dof,n_dof), damping(n_dof,n_dof);
  stiffness = this->pb_conf->computeStiffness();
  mass      = this->pb_conf->computeMass();
  damping   = this->pb_conf->computeDamping();
  MatrixXd newmark_ope = this->newmark_operator(stiffness, damping, mass);
  for(int i=0;i<this->time->getNstep()-1;i++){
    this->newmark_step(newmark_ope, damping, mass, i);
  }
}

MatrixXd Newmark::newmark_operator(MatrixXd& stiffness, MatrixXd& damping, MatrixXd& mass){
  MatrixXd ope = MatrixXd::Zero(this->pb_conf->nn(),this->pb_conf->nn());
  MatrixXd ope_inv(this->pb_conf->nn(),this->pb_conf->nn());
  double step_size = this->time->stepSize();
  double a0 = 1./( this->beta * step_size * step_size );
  double a1 = this->alpha / (this->beta * step_size);
  ope = stiffness + a0 * mass + a1 * damping;
  ope_inv = ope.inverse();
  return ope_inv;
}

void Newmark::newmark_step(MatrixXd& newmark_ope, MatrixXd& damping, MatrixXd& mass, int step){
  VectorXd f_ext;
  int n_dof = this->pb_conf->nn();
  // compute coefficients
  double dt = this->time->stepSize();
  double a0 = 1./( this->beta * dt * dt );
  double a1 = this->alpha / (this->beta * dt);
  double a2 = 1. / (this->beta * dt);
  double a3 = 1. / (2 * this->beta) - 1.;
  double a4 = this->alpha / this->beta - 1;
  double a5 = dt /2 * ( this->alpha / this->beta - 2) ; 
  double a6 = dt * ( 1 - this->alpha);
  double a7 = this->alpha * dt;  
  // Compute effective load 
  f_ext = this->pb_conf->computeExtLoad(this->time->eval(step)) ;
  f_ext += damping * (a1 * this->solution_u.col(step) + a4 * this->solution_du.col(step) + a5 * this->solution_ddu.col(step) );
  f_ext += mass * (a0 * this->solution_u.col(step) + a2 * this->solution_du.col(step) + a3 * this->solution_ddu.col(step) );
  // Compute displacement
  
  this->solution_u.col(step+1) = newmark_ope * f_ext;
  // Compute speed and acceleration
  this->solution_ddu.col(step+1) = a0 * ( this->solution_u.col(step+1) - this->solution_u.col(step) ) - a2 * this->solution_du.col(step) - a3 * this->solution_ddu.col(step) ;
  this->solution_du.col(step+1) = this->solution_du.col(step) + a6 * this->solution_ddu.col(step) + a7 * this->solution_ddu.col(step+1);
}


MatrixXd Newmark::getSolution(){
  return this->solution_u;
}

MatrixXd Newmark::getSolution(int i){
  if(i==0){
    return this->solution_u;
  }
  else if(i==1){
    return this->solution_du;
  }
  else if(i==2){
    return this->solution_ddu;
  }
}
