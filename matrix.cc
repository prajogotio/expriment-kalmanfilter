#include "matrix.h"

#include <cassert>
#include <iostream>
#include <cmath>
#include <sstream>

namespace kalman {

static constexpr double kPrecisionError = 1e-12;

Matrix Matrix::Multiply(const Matrix& other) const {
  assert(other.num_rows_ == num_columns_);
  Matrix result;
  result.SetSize(num_rows_, other.num_columns_);
  for (int i = 0; i < num_rows_; ++i) {
    for (int j = 0; j < other.num_columns_; ++j) {
      double sum = 0;
      for (int k = 0; k < num_columns_; ++k) {
        sum += GetEntry(i, k) * other.GetEntry(k, j);
      }
      result.SetEntry(i, j, sum);
    }
  }
  return result;
}

Matrix Matrix::ScalarMultiply(double c) const {
  Matrix result;
  result.SetSize(num_rows_, num_columns_);
  for (int i = 0; i < num_rows_; ++i) {
    for (int j = 0; j < num_columns_; ++j) {
      result.SetEntry(i, j, GetEntry(i, j) * c);
    }
  }
  return result;
}

Matrix Matrix::Add(const Matrix& other) const {
  assert(other.num_rows_ == num_rows_);
  assert(other.num_columns_ == num_columns_);
  Matrix result;
  result.SetSize(num_rows_, num_columns_);
  for (int i = 0; i < num_rows_; ++i) {
    for (int j = 0; j < num_columns_; ++j) {
      result.SetEntry(i, j, GetEntry(i, j) + other.GetEntry(i, j));
    }
  }
  return result;
}

Matrix Matrix::Minus(const Matrix& other) const {
  return Add(other.ScalarMultiply(-1));
}

Matrix Matrix::Transpose() const {
  Matrix result;
  result.SetSize(num_columns_, num_rows_);
  for (int i = 0; i < num_rows_; ++i) {
    for (int j = 0; j < num_columns_; ++j) {
      result.SetEntry(j, i, GetEntry(i, j));
    }
  }
  return result;
}

Matrix Matrix::Invert() const {
  assert(num_rows_ == num_columns_);
  Matrix augmented;
  augmented.SetSize(num_rows_, num_columns_ * 2);
  for (int i = 0; i < num_rows_; ++i) {
    for (int j = 0; j < num_columns_; ++j) {
      augmented.SetEntry(i, j, GetEntry(i, j));
    }
    augmented.SetEntry(i, num_rows_ + i, 1.0);
  }
  for (int cur_column = 0; cur_column < num_rows_; ++cur_column) {
    int pivot_entry = cur_column;
    for (int i = cur_column; i < num_rows_; ++i) {
      if (fabs(augmented.GetEntry(i, cur_column)) >
          fabs(augmented.GetEntry(pivot_entry, cur_column))) {
        pivot_entry = i;
      }
    }
    if (fabs(augmented.GetEntry(pivot_entry, cur_column)) < kPrecisionError) {
      // Matrix is non-invertible.
      return Matrix();
    }
    augmented.SwapRow(pivot_entry, cur_column);
    augmented.MultiplyRow(cur_column,
                          1.0/augmented.GetEntry(cur_column, cur_column));
    for (int i = 0; i < num_rows_; ++i) {
      if (i == cur_column) continue;
      augmented.AddRow(i, cur_column, -augmented.GetEntry(i, cur_column));
    }
  }
  Matrix inverse;
  inverse.SetSize(num_rows_, num_columns_);
  for (int i = 0; i < num_rows_; ++i) {
    for (int j = 0; j < num_columns_; ++j) {
      inverse.SetEntry(i, j, augmented.GetEntry(i, j + num_columns_));
    }
  }
  return inverse;
}

double Matrix::GetEntry(int row, int col) const {
  assert(row < num_rows_);
  assert(col < num_columns_);
  return entries_[row * num_columns_ + col];
}

bool Matrix::Equals(const Matrix& other) const {
  if (num_rows_ != other.num_rows_ || num_columns_ != other.num_columns_) {
    return false;
  }
  for (int i = 0; i < num_rows_; ++i) {
    for (int j = 0; j < num_columns_; ++j) {
      if (fabs(GetEntry(i, j) - other.GetEntry(i, j)) > kPrecisionError) {
        return false;
      }
    }
  }
  return true;
}

std::string Matrix::ToString() const {
  std::stringstream stream;
  stream.precision(15);
  for (int i = 0; i < num_rows_; ++i) {
    for (int j = 0; j < num_columns_; ++j) {
      stream << GetEntry(i, j) << " ";
    }
    stream << "\n";
  }
  return stream.str();
}

Matrix Matrix::Identity(int dimension) {
  assert(dimension > 0);
  Matrix identity;
  identity.SetSize(dimension, dimension);
  for (int i = 0; i < dimension; ++i) {
    identity.SetEntry(i, i, 1);
  }
  return identity;
}

void Matrix::SetSize(int num_rows, int num_columns) {
  entries_.resize(num_rows * num_columns, 0);
  num_rows_ = num_rows;
  num_columns_ = num_columns;
}

void Matrix::SetEntry(int row, int col, double val) {
  double& entry = GetEntryRef(row, col);
  entry = val;
}

double& Matrix::GetEntryRef(int row, int col) {
  assert(row < num_rows_);
  assert(col < num_columns_);
  return entries_[row * num_columns_ + col];
}

void Matrix::SwapRow(int row_1, int row_2) {
  assert(row_1 < num_rows_);
  assert(row_2 < num_rows_);
  if (row_1 == row_2) return;
  double temp;
  for (int i = 0; i < num_columns_; ++i) {
    temp = GetEntry(row_1, i);
    SetEntry(row_1, i, GetEntry(row_2, i));
    SetEntry(row_2, i, temp);
  }
}

void Matrix::AddRow(int row_1, int row_2, double c) {
  assert(row_1 < num_rows_);
  assert(row_2 < num_rows_);
  for (int i = 0; i < num_columns_; ++i) {
    SetEntry(row_1, i, GetEntry(row_1, i) + c * GetEntry(row_2, i));
  }
}

void Matrix::MultiplyRow(int row, double c) {
  assert(row < num_rows_);
  for (int i = 0; i < num_columns_; ++i) {
    SetEntry(row, i, GetEntry(row, i) * c);
  }
}

}  // namespace kalman
