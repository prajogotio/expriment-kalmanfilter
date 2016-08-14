#include "kalman.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

struct Reading {
  double x, y, z;
};

void split(const std::string& s, char delim,
           std::vector<std::string>* tokens) {
  std::stringstream stream(s);
  std::string item;
  while (std::getline(stream, item, delim)) {
    tokens->push_back(item);
  }
}

const bool kEmitResult = true;

int main() {
  std::ifstream input("test_in.txt");
  std::string line;
  std::vector<Reading> readings;
  while (std::getline(input, line)) {
    std::vector<std::string> tokens;
    // Each line contains the acceleration reading along x, y and z axis.
    split(line, ' ', &tokens);
    readings.push_back({std::stod(tokens[0]),
                        std::stod(tokens[1]),
                        std::stod(tokens[2])});
  }
  kalman::Matrix init_acceleration(3, 1, {0.0, 0.0, 0.0});
  kalman::Matrix init_covariance(3, 3, {1000.0, 0.0, 0.0,
                                        0.0, 1000.0, 0.0,
                                        0.0, 0.0, 1000.0});
  kalman::KalmanFilter filter(init_acceleration, init_covariance);
  kalman::Matrix transition = kalman::Matrix::Identity(3);
  kalman::Matrix process_error = kalman::Matrix::Identity(3)
                                     .ScalarMultiply(0.125);
  kalman::Matrix reading_error(3, 3, {10.0,  0.0,  0.0,
                                       0.0, 10.0,  0.0,
                                       0.0,  0.0, 10.0});
  kalman::Matrix scaling = kalman::Matrix::Identity(3);
  for (const Reading& reading : readings) {
    filter.Predict(transition, process_error);
    filter.UpdateWithCurrentReading(
        kalman::Matrix(3, 1, {reading.x, reading.y, reading.z}),
        reading_error,
        scaling);
    const kalman::Matrix& current_mean = filter.GetCurrentMean();
    if (kEmitResult) {
      std::cout << current_mean.GetEntry(0, 0) << " " 
                << current_mean.GetEntry(1, 0) << " "
                << current_mean.GetEntry(2, 0) << std::endl;
    }
  }
  return 0;
}