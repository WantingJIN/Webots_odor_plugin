#ifndef KERNEL3D_H_
#define KERNEL3D_H_

#include "Position.h"
#include <vector>
#include <math.h>
#include <stdio.h>
#include "SampleBuffer.h"

using namespace std;

//gas map boundaries
#define map_x_min 2.75
#define map_x_max 12.7418
#define map_y_min 0.01
#define map_y_max 3.9916
#define map_x_nbins 64
#define map_y_nbins 64


struct KernelParameters {
  float sigma;
  float gamma;
  float scaling_param;
  float wind_speed;
  float rco;
  float wmin;
  float rmin;
  float rmax;
};

class GasMap2D{
  public:

    double gasWeightConMap[map_x_nbins][map_y_nbins];
    double gasAverConMap[map_x_nbins][map_y_nbins];
    double gasWeightMap[map_x_nbins][map_y_nbins];
    double gasConfidenceMap[map_x_nbins][map_y_nbins];

    double mx_min;
    double my_min;
    double mx_max;
    double my_max;
    double mx_resol;
    double my_resol;
    double mx_update_range;
    double my_update_range;

    
    GasMap2D(double sigma, double gamma, double scaling_param, double wind_speed, double rco);
    void configureKernelAlgorithm(double sigma, double gamma, double scaling_param, double wind_speed, double rco);
    void runKernelAlgorithm(Position current_position, float current_odor);
    KernelParameters kp;
    void initialize_gas_map_features();
    void retrieve_readings_from_gasmap(const vector<Position> history_position, vector<double> &concentrations_from_map);
    void buildGasDistributionMap(vector<Position> sensor_positions, vector<SampleBuffer> sampleBuffer);
    void logGasMap(char* filename);
    void printGasMap();

    
  private:
    //Eigen::Matrix2f _sigma_matrix;
    double _sigma_matrix[2][2];
    float _gaussian_divisor;
    float avg_WRt;
    float counter_WRt;

    void setBinBorders(int n_bins);
    void setSigmaMatrix();
    void setGaussianDivisor();
    float compute_gaussian(const double x_dist, const double y_dist);

};

#endif
