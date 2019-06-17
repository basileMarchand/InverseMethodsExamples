
#include "DataObject.h"

#define PI 3.14139




/***********************************************
 * Base class for model definition
 ***********************************************/
ProblemConfig::ProblemConfig(int n_dof){
  this->n = n_dof;
}

MatrixXd ProblemConfig::computeStiffness(){
  return MatrixXd::Zero(this->n,this->n);
}

MatrixXd ProblemConfig::computeDamping(){
  return MatrixXd::Zero(this->n,this->n);
}

MatrixXd ProblemConfig::computeMass(){
  return MatrixXd::Zero(this->n,this->n);
}

VectorXd ProblemConfig::computeExtLoad(double t){
  return VectorXd::Zero(this->n);
}

int ProblemConfig::nn(){
  return this->size();
}

int ProblemConfig::size(){
  return this->n;
}

/***********************************************
 * Class for n dofs Mass-spring
 ***********************************************/

MassSpring::MassSpring(int ndof) : ProblemConfig(ndof){}

void MassSpring::setMass(double mass){
  this->mass = mass;
}

void MassSpring::setStiffness(double stiff){
  this->stiff = stiff;
}

void MassSpring::setUserLoad(std::function<VectorXd(double)>& ext_load){
  this->ext_load = ext_load;
}

void MassSpring::updateParams(vector<double> p){
  this->stiff = p[0];
  this->mass = p[1];
}

MatrixXd MassSpring::computeMass(){
  MatrixXd mass_mat = MatrixXd::Zero(this->n,this->n);
  for(int i=0; i< this->n; i++){
    mass_mat(i,i) =  this->mass;
  }
  return mass_mat;
}

MatrixXd MassSpring::computeStiffness(){
  MatrixXd stiff_mat = MatrixXd::Zero(this->n, this->n);
  stiff_mat(0,0) = this->stiff;
  stiff_mat(0,1) = - this->stiff;
  stiff_mat(this->n-1,this->n-2) = -this->stiff;
  stiff_mat(this->n-1,this->n-1) = this->stiff;
  for(int i=1; i< this->n-1; i++){
    stiff_mat(i,i) = this->stiff;
    stiff_mat(i,i-1) = -this->stiff;
    stiff_mat(i,i+1) = -this->stiff;
  }
  return stiff_mat;
}

VectorXd MassSpring::computeExtLoad(double t){
  return this->ext_load(t);
}


/******************************************************
 * Class for AeroElastic model
 ******************************************************/


AeroElastic::AeroElastic() : ProblemConfig(2) {}

void AeroElastic::setStiffness(double kh, double ka){
  this->kh = kh;
  this->ka = ka;
}

void AeroElastic::setMass(double m){
  this->m = m;
}

void AeroElastic::setDamping(double c){
  this->ch = c;
}

void AeroElastic::setGeometry(double l, double xf, double xc, double c, double b, double e){
  this->l = l;
  this->xf = xf;
  this->xc = xc;
  this->c = c;
  this->b = b ;
  this->e = e ;
}

void AeroElastic::setAeroDyn(double u, double rho){
  this->u = u;
  this->rho = rho;
}

void AeroElastic::updateParams(vector<double> p){
  this->ch = p[0];
}


MatrixXd AeroElastic::computeMass(){
  MatrixXd mass_matrix = MatrixXd::Zero(2,2);
  double S = this->m * (0.5 * this->c - this->xf);
  double Ia = (1./3.) * this->m * ( this->c*this->c - 3 *this->c * this->xf + 3  * this->xf*this->xf);
  double coef = this->rho * PI * this->b * this->b;
  mass_matrix(0,0) = this->m;
  mass_matrix(0,1) = S + coef * (0.5 * this->c - this->xf);
  mass_matrix(1,0) = S + coef * (0.5 * this->c - this->xf);
  mass_matrix(1,1) = Ia + coef * (0.5 * this->c - this->xf)*(0.5 * this->c - this->xf) + this->b*this->b/8.;
  return mass_matrix;
}

MatrixXd AeroElastic::computeStiffness(){
  MatrixXd stiff_matrix = MatrixXd::Zero(2,2);
  double coef = this->rho * this->u * this->u * PI * this->c;
  stiff_matrix(0,0) = this->kh;
  stiff_matrix(0,1) = coef;
  stiff_matrix(1,0) = 0.;
  stiff_matrix(1,1) = this->ka - this->e * this->c * coef ; 
  return stiff_matrix;
}

MatrixXd AeroElastic::computeDamping(){
  MatrixXd damp_matrix = MatrixXd::Zero(2,2);
  double coef = this->rho * this->u * PI * this->c;
  damp_matrix(0,0) = this->ch + coef;
  damp_matrix(0,1) = coef * ((3./4.)*this->c - this->xf  + this->c/4. );
  damp_matrix(1,0) = coef * ( -this->e * this->c );
  damp_matrix(1,1) = coef * ((0.5 * this->c - this->xf) * (0.5 * this->c - this->xf) + ( 3./4. * this->c - this->xf) * (this->c /4.) ) ;
  return damp_matrix;
}

/*
 * Time Discretization definition
 *
 */ 

TimeDis::TimeDis(){}

void TimeDis::setTimeInterval(double tinit, double tend){
  this->t_0 = tinit;
  this->t_N = tend;
}

void TimeDis::setTimeStep(int n){
  this->n_step = n;
  this->compute();
}

void TimeDis::compute(){
  this->time_vals.resize(this->n_step);
  this->step_size = (this->t_N - this->t_0) / (this->n_step-1) ;
  for(int i=0; i<this->n_step ; i++){
    this->time_vals[i] = this->t_0 + i * this->step_size;
  }
}

double TimeDis::eval(int i){
  return this->time_vals[i];
}


int TimeDis::getNstep(){
  return this->time_vals.size();
}


int TimeDis::size(){
  return this->time_vals.size();
}

double TimeDis::stepSize(){
  return this->step_size;
}


vector<double> TimeDis::getVals(){
  return this->time_vals;
}

/*
 *  Observations simulations class
 */

SensorObservations::SensorObservations(TimeDis* time_simu, MatrixXd& data){
  this->time_simu = time_simu;
  this->data = data;
}

void SensorObservations::setNoiseLevel(double sigma){
  this->noise_level = sigma;
  this->noise_generator = normal_distribution<double>(0.0,sigma);
}

void SensorObservations::setMeasuredDofIds(vector<int> dofs){
  this->dofs_id = dofs;
}

bool SensorObservations::isInteger(double x){
  return x == floor(x);
}

void SensorObservations::setObservationsTimeDis(TimeDis* time){
  this->time_obs = time;
  // Find corresponding step in the original time dis
  double dt_simu = this->time_simu->stepSize();
  double dt_obs  = this->time_obs->stepSize();
  double ratio = dt_obs / dt_simu;
  try{
    if( !this->isInteger(ratio) ){
      cout << "Ratio = " << ratio << endl;
      throw string("ERROR - Measurements time doesn't match with simulation time");
    }
    else{
      int dn = floor(ratio);
      for(int i=0; i<this->time_obs->size(); i++){
	this->step_id.push_back(i*dn);
      }
    }
  }
  catch(string const& msg){
    cerr << msg << endl;
    exit(1);
  }
}

MatrixXd SensorObservations::extract(){
  MatrixXd obs_data = MatrixXd::Zero(this->dofs_id.size(), this->step_id.size());
  default_random_engine generator;
  double du = 0;
  for(int k=0; k<this->step_id.size(); k++){
    for(int i=0; i<this->dofs_id.size(); i++){
      du = this->noise_generator(generator);
      obs_data(i,k) = (1.+du) * this->data(this->dofs_id[i],this->step_id[k]);
    }
  }
  return obs_data;
}

