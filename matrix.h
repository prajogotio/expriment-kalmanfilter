#ifndef KALMAN_MATRIX_H
#define KALMAN_MATRIX_H

#include <cassert>
#include <vector>

namespace kalman {

class Matrix {
 public:
  Matrix(int num_rows, int num_columns, const std::vector<double> entries)
      : num_rows_(num_rows), num_columns_(num_columns), entries_(entries) {
        assert(num_rows_ * num_columns_ == entries_.size());
      }
  Matrix() : num_rows_(0), num_columns_(0) {}
  Matrix Multiply(const Matrix& other) const;
  Matrix ScalarMultiply(double c) const;
  Matrix Add(const Matrix& other) const;
  Matrix Minus(const Matrix& other) const;
  Matrix Transpose() const;
  Matrix Invert() const;
  double GetEntry(int row, int col) const;
  bool Equals(const Matrix& other) const;
  std::string ToString() const;
  static Matrix Identity(int dimension);
  
 private:
  void SetSize(int num_rows, int num_columns);
  void SetEntry(int row, int col, double val);
  double& GetEntryRef(int row, int col);
  void SwapRow(int row_1, int row_2);

  // Adds 'c' * row_2' to 'row_1'.
  void AddRow(int row_1, int row_2, double c);

  // Multiplies 'row' with scalar multiplier 'c'.
  void MultiplyRow(int row, double c);

  std::vector<double> entries_;
  int num_rows_;
  int num_columns_;
};

}  // namespace kalman
#endif  // KALMAN_MATRIX_H