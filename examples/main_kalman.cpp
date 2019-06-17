#include <string>
#include <fstream>


#include "DataObject.h"
#include "Newmark.h"
#include "KalmanFilter.h"
#include "XmlTree.hpp"

using namespace std;

/*****************************************
 * Function to write result on text file
 *****************************************/
void exportData(string fname, vector<double> time, MatrixXd data){
  ofstream myfile;
  myfile.open(fname);
  int n_step = time.size();
  int n_dofs = data.rows();
  for(int i=0;i<n_step;i++){
    myfile << time[i] << " " ;
    for(int j=0;j<n_dofs;j++){
      myfile << data(j,i) << " ";
    }
    myfile << "\n";
  }
  myfile.close();
}

/***************************************
 * Model class for Kalman filtering
 ***************************************/

class ModelOperatorJoin: public Operator{
public:
  ModelOperatorJoin(TimeDis* time): Operator(time){};
  void setDirectModel(AeroElastic* mod){
    this->model = mod;
    this->n = 2 * this->model->size() + 1;
  }
  VectorXd eval(int step, VectorXd x){
    int ndof = this->model->size();
    //VectorXd param = x.tail(1);
    VectorXd state = x.head(2*ndof);
    vector<double> params = {x(4)};
    this->model->updateParams(params);
    MatrixXd K = this->model->computeStiffness();
    MatrixXd C = this->model->computeDamping();
    MatrixXd M = this->model->computeMass();
    MatrixXd Minv = M.inverse();
    MatrixXd T = MatrixXd::Zero(2*ndof, 2*ndof);
    T.block(0,ndof,ndof,ndof) = MatrixXd::Identity(ndof,ndof);
    T.block(ndof,0,ndof,ndof) = - Minv * K ;
    T.block(ndof,ndof,ndof,ndof) = - Minv * C ;
    VectorXd output = VectorXd::Zero(this->n);
    output.head(2*ndof) = state + this->time->stepSize() * T * state;
    output(4) = x(4);
    return output;
  }
  VectorXd eval(int step){};
  MatrixXd matrix(int step){};
protected:
  AeroElastic* model;
  double ka;
};

/***************************************
 * Observation class for Kalman Filtering
 ***************************************/

class ObsOperatorJoin: public Operator{
public: 
  ObsOperatorJoin(TimeDis* time, int nobs): Operator(time){this->n = nobs;}
  void initialize(){
    this->projector = MatrixXd::Zero(this->n, 5);
    MatrixXd Id =  MatrixXd::Identity(this->n, this->n);
    this->projector.block(0,0,this->n,this->n) = Id;
  }
  VectorXd eval(int step, VectorXd x){
    return this->projector * x;
  }
  VectorXd eval(int step){};
  MatrixXd matrix(int step){};
  
protected:
  MatrixXd projector;
};


/**************************************
 * Main Function 
 **************************************/
int main(){

  XmlBlock input;
  input.fromFile( "kalman.xml" );
  
  XmlBlock* model_param = input.getBlock("model");
  XmlBlock* time_param = input.getBlock("time");
  XmlBlock* solver_param = time_param->getBlock("newmark");
  XmlBlock* obs_param = time_param->getBlock("observations");
  XmlBlock* kalman_param = input.getBlock("kalman");
  XmlBlock* ekf_param = input.getBlock("ekf");
  XmlBlock* ukf_param = input.getBlock("ukf");

  // Initial Conditions
  VectorXd u_0(2);
  //u_0 << 0.1, -0.1;
  std::vector<double> u = time_param->getVectorDoubleElem("initial");
  u_0 << u[0] , u[1];
  VectorXd du_0 = VectorXd::Zero(2);
  VectorXd ddu_0 = VectorXd::Zero(2);
  /********************************************************
   *     Solve direct Problem to simulate observation     * 
   ********************************************************/
  //-> Define AeroElastic model
  AeroElastic* direct_model = new AeroElastic();
  direct_model->setStiffness(model_param->getDoubleElem("kh"),model_param->getDoubleElem("ka"));
  direct_model->setDamping(model_param->getDoubleElem("ch"));
  direct_model->setMass(model_param->getDoubleElem("mass"));
  direct_model->setGeometry(model_param->getDoubleElem("l"),model_param->getDoubleElem("xf"),model_param->getDoubleElem("xc"),model_param->getDoubleElem("c"),model_param->getDoubleElem("b"),model_param->getDoubleElem("e"));
  direct_model->setAeroDyn(model_param->getDoubleElem("U"), model_param->getDoubleElem("rho"));

  // Define time discretization 
  TimeDis* time = new TimeDis();
  time->setTimeInterval(time_param->getDoubleElem("init"), time_param->getDoubleElem("end"));
  time->setTimeStep(time_param->getIntElem("step"));
  
  // Declare newmark solver 
  Newmark* solver = new Newmark(direct_model, time);
  solver->setInit(u_0, du_0, ddu_0);
  solver->newmarkOpt(solver_param->getDoubleElem("alpha"), solver_param->getDoubleElem("beta"));
  solver->solve();

  // Export data for plot 
  vector<double> time_vals = time->getVals();
  exportData("airfoil.dat", time_vals, solver->getSolution()); 

  /********************************************************
   *    Extract Obs data and add Gaussian white noise     *
   ********************************************************/
  
  vector<int> dofs_id = obs_param->getVectorIntElem("dofs");
  TimeDis* time_obs = new TimeDis();
  time_obs->setTimeInterval(time_param->getDoubleElem("init"), time_param->getDoubleElem("end"));
  time_obs->setTimeStep(obs_param->getIntElem("step"));
  
  // Observations configurations
  int n_obs = dofs_id.size();  // Only h is observed, set 2 for h and alpha measure

  MatrixXd data = solver->getSolution();

  SensorObservations* sensor = new SensorObservations(time, data);
  sensor->setObservationsTimeDis(time_obs);
  sensor->setMeasuredDofIds(dofs_id);
  sensor->setNoiseLevel(0.01);
  MatrixXd obs_data = sensor->extract();
  exportData("observations.dat", time_obs->getVals(), obs_data);
  /********************************************************
   *    Solve inverse problem using Kalman Filtering      *
   *               with join formulation
   ********************************************************/
  ModelOperatorJoin* model = new ModelOperatorJoin(time_obs);
  model->setDirectModel(direct_model);
  
  ObsOperatorJoin* obs = new ObsOperatorJoin(time_obs, n_obs);
  obs->initialize();

  // Kalman parameters
  VectorXd state_init(5);
  state_init << u_0, du_0, 0.;
  MatrixXd state_cov = MatrixXd::Identity(5,5);
  state_cov *= 0.01;
  //state_cov(4,4) = 1.;
  MatrixXd model_error = MatrixXd::Identity(5,5);
  model_error *= 0.002;
  //model_error(4,4) = 0.1;

  MatrixXd obs_error = MatrixXd::Identity(n_obs,n_obs);
  obs_error *= 0.001;

  // ******************************************
  // Extended Kalman Filter resolution
  // ******************************************
  cout << "*******************************************" << endl;
  cout << "* Solve inverse problem using Extended KF *" << endl; 
  cout << "*******************************************" << endl;
  ExtendedKalmanFilter* ekf = new ExtendedKalmanFilter(model, obs, time_obs);
  ekf->setLinearizationParam(ekf_param->getIntElem("order"),ekf_param->getDoubleElem("variation"));
  ekf->initialize();
  ekf->setInitState(state_init, state_cov);
  ekf->setModelError(model_error);
  ekf->setObsError(obs_error);
  ekf->run(obs_data);

  // Extract param evaluation and export in txt file
  MatrixXd res_ekf = ekf->getState();
  MatrixXd param = res_ekf.block(4,0,1,time_obs->size());
  MatrixXd state = res_ekf.block(0,0,2,time_obs->size());
  vector<MatrixXd> variances = ekf->getCovariance();
  MatrixXd var_param = MatrixXd::Zero(1,time_obs->size());
  for(int i=0; i<variances.size(); i++){
    var_param(0,i) = variances[i](4,4);
  }
  MatrixXd ekf_res(2,time_obs->size());
  ekf_res.row(0) = param;
  ekf_res.row(1) = var_param;
  exportData("param_ekf.dat", time_obs->getVals(), ekf_res);
  exportData("state_ekf.dat", time_obs->getVals(), state);


  // ******************************************
  // Unscented Kalman Filter resolution
  // ******************************************
  cout << "********************************************" << endl;
  cout << "* Solve inverse problem using Unscented KF *" << endl; 
  cout << "********************************************" << endl;

  // Unscented parametes
  double alpha_ut = 0.001 ;
  double beta_ut  = 2. ;
  double kappa_ut = 0. ;
  
  ModelOperatorJoin* model2 = new ModelOperatorJoin(time_obs);
  model2->setDirectModel(direct_model);
  
  ObsOperatorJoin* obs2 = new ObsOperatorJoin(time_obs, n_obs);
  obs2->initialize();

  UnscentedKalmanFilter* ukf = new UnscentedKalmanFilter(model2, obs2, time_obs);
  ukf->setUnscentedParam(ukf_param->getDoubleElem("alpha"), ukf_param->getDoubleElem("beta"), ukf_param->getDoubleElem("kappa"));
  ukf->initialize();
  ukf->setInitState(state_init, state_cov);
  ukf->setModelError(model_error);
  ukf->setObsError(obs_error);
  ukf->run(obs_data);

  // Extract param evaluation and export in txt file
  MatrixXd res_ukf = ukf->getState();
  MatrixXd param_ukf = res_ukf.block(4,0,1,time_obs->size());
  MatrixXd state_ukf = res_ukf.block(0,0,2,time_obs->size());
  variances = ukf->getCovariance();
  MatrixXd var_param_ukf = MatrixXd::Zero(1,time_obs->size());
  for(int i=0; i<variances.size(); i++){
    var_param_ukf(0,i) = variances[i](4,4);
  }
  MatrixXd ukf_res(2,time_obs->size());
  ukf_res.row(0) = param_ukf;
  ukf_res.row(1) = var_param_ukf;
  exportData("param_ukf.dat", time_obs->getVals(), ukf_res);
  exportData("state_ukf.dat", time_obs->getVals(), state_ukf);
}


