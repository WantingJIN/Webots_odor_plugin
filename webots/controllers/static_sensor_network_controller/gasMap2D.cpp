#include "gasMap2D.h"
#include <iostream>


GasMap2D::GasMap2D(double sigma, double gamma, double scaling_param, double wind_speed, double rco){
  avg_WRt=0;
  counter_WRt=0;

  configureKernelAlgorithm(sigma, gamma, scaling_param, wind_speed, rco);
  initialize_gas_map_features();
}

void GasMap2D::initialize_gas_map_features(){
  mx_min = map_x_min;
  mx_max = map_x_max;
  my_min = map_y_min;
  my_max = map_y_max;
  mx_resol = (mx_max - mx_min) / map_x_nbins;
  my_resol = (my_max - my_min) / map_y_nbins;
  mx_update_range = kp.rco / mx_resol;
  my_update_range = kp.rco / my_resol;
}

void GasMap2D::configureKernelAlgorithm(double sigma, double gamma, double scaling_param, double wind_speed, double rco){
  kp.sigma = sigma;
  kp.gamma = gamma;
  kp.scaling_param = scaling_param;
  kp.wind_speed = wind_speed;
  kp.rco = rco;

  for (int i = 0; i < map_x_nbins; i++){
    for (int j = 0; j < map_y_nbins; j++){
      gasWeightConMap[i][j] = 0;
      gasAverConMap[i][j] = 0;
      gasWeightMap[i][j] = 0;
      gasConfidenceMap[i][j] = 0;
    }
  }
  setGaussianDivisor();
}

void GasMap2D::runKernelAlgorithm(Position current_position, float current_odor){

  double x_dist = 0, y_dist = 0; //distance from sampled point from center of cells
  //normalize received odor sample
  int idx_x = round((current_position.x - mx_min) / mx_resol);
  int idx_y = round((current_position.y - my_min) / my_resol);
  int idx_x_start = (idx_x - mx_update_range > 0) ? (idx_x - mx_update_range) : 0;
  int idx_x_end = (idx_x + mx_update_range < map_x_nbins) ? (idx_x + mx_update_range) : map_x_nbins;
  int idx_y_start = (idx_y - my_update_range > 0) ? (idx_y - my_update_range) : 0;
  int idx_y_end = (idx_y + my_update_range < map_y_nbins) ? (idx_y + my_update_range) : map_y_nbins;
  //printf("x_pos is %f, y_pos is %f, odor_con is %f \n", current_position.x, current_position.y, current_odor);
  //printf("idx x is %d, idx y is %d \n", idx_x, idx_y);
  //printf("idx x start is %d, idx x end is %d, idx y start is %d, idx y end is %d \n", idx_x_start, idx_x_end, idx_y_start, idx_y_end);
  for(int i = idx_x_start; i < idx_x_end; i++){
    for(int j = idx_y_start; j < idx_y_end; j++){
      x_dist = i * mx_resol + mx_min - current_position.x;
      y_dist = j * my_resol + my_min - current_position.y;
      float wcells = compute_gaussian(x_dist, y_dist);
      //printf("distance from cell (%d, %d) to current position is (%f, %f)", i, j, x_dist, y_dist);
      //printf("weight is %f \n", wcells);
      gasWeightMap[i][j] += wcells;
      //printf("Weight of the measurement of cell (%d, %d) is %f \n", i, j, wcells);
      //printf(" Updated gasWeightMap(%d,%d) is %f ", i, j, gasWeightMap[i][j]);
      gasWeightConMap[i][j] += current_odor * wcells;
      //printf(" Updated gasWeightConMap(%d,%d) is %f ", i, j, gasWeightConMap[i][j]);
      gasConfidenceMap[i][j] = 1 - exp(-gasWeightMap[i][j] / pow(kp.scaling_param,2));
      //printf(" gasAverConMap(%d,%d) is %f \n", i, j, gasAverConMap[i][j]);
      gasAverConMap[i][j] = gasWeightConMap[i][j] / gasWeightMap[i][j];
      //printf(" Updated gasAverConMap(%d,%d) is %f \n", i, j, gasAverConMap[i][j]);
    }
  }
}


void GasMap2D::buildGasDistributionMap(vector<Position> sensor_positions, vector<SampleBuffer> sampleBuffer) {
  //printf("sensor_positions size is %d, measurements size is %d \n", sensor_positions.size(), measurements.size());
  for (int i = 0; i < sensor_positions.size(); i++){
    runKernelAlgorithm(sensor_positions[i], sampleBuffer[i].average_concentration);
  }
}
//////////////////////////////////////////////////// PRIVATE FUNCTIONS ///////////////////////////////////////

// void GasMap2D::setSigmaMatrix(){
//   //if (kp.wind_speed ==0){
//     _sigma_matrix << kp.sigma,0,
//                 0,kp.sigma;
//   //}else{
//     // FIXME: check if this is correct
//     // float bc= kp.sigma/sqrt(1+kp.gamma*kp.wind_speed/kp.sigma);
//     // _sigma_matrix << kp.sigma+kp.gamma*kp.wind_speed,0,0,
//     //              0,bc,0,
//     //              0,0,bc;
//   //}
// }

// void GasMap2D::setGaussianDivisor(){
//   _gaussian_divisor= 1/(pow(2*M_PI,3/2)*pow(_sigma_matrix.determinant(),0.5));
// }
void GasMap2D::setGaussianDivisor(){
   _gaussian_divisor= 1/(2*M_PI*pow(kp.sigma,2));
}

float GasMap2D::compute_gaussian(const double x_dist, const double y_dist){
    float a  = kp.sigma + kp.gamma*kp.wind_speed;
    float b  = kp.sigma/(1 + kp.gamma*kp.wind_speed/kp.sigma);
    //printf("a is %f, b is %f \n", a, b);
    //float num= (pow(x_dist,2)+ pow(y_dist,2))/ (2 * pow(kp.sigma,2));
    float num= (pow(x_dist,2)/ (2 * pow(a,2)) + pow(y_dist,2)/ (2 * pow(b,2)));
    //printf("num is %f \n", num);
    return exp(-num)/_gaussian_divisor;
}

void GasMap2D::retrieve_readings_from_gasmap(const vector<Position> history_position, vector<double> &concentrations_from_map){
  for(int i=0; i<history_position.size(); i++){
    int idx_x = round((history_position[i].x - mx_min) / mx_resol);
    int idx_y = round((history_position[i].y - my_min) / my_resol);
    concentrations_from_map.push_back(gasAverConMap[idx_x][idx_y]);
  }
}

void GasMap2D::logGasMap(char * filename){
  FILE *gas_map_file = fopen(filename, "w");
  for(int i = 0; i < map_x_nbins; i++){
    for(int j = 0; j < map_y_nbins; j++){
      fprintf(gas_map_file, "%f, ", gasAverConMap[i][j]);
    }
  }
  fprintf(gas_map_file, "\n");
  fclose(gas_map_file);
}

void GasMap2D::printGasMap(){
  printf("Gas distribution map: \n");
  for(int i = 0; i < map_x_nbins; i++){
    for(int j = 0; j < map_y_nbins; j++){
      printf("%f, ", gasAverConMap[i][j]);
    }
  }
  printf("\n");
}