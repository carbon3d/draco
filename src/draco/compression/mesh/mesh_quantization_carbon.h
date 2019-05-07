#pragma once

#include "draco/mesh/mesh.h"

class MeshQuantizationCarbon {
 public:
  MeshQuantizationCarbon() :
      quantization_bits_(-1),
      range_(0),
      min_values_(3, 0)
  {}
  bool IsSet() {return quantization_bits_ != -1;}
  int quantization_bits() const {return quantization_bits_;}
  float range() const {return range_;}
  float min_values_x() {return min_values_[0];}
  float min_values_y() {return min_values_[1];}
  float min_values_z() {return min_values_[2];}
  std::string FillFromMesh(draco::Mesh *mesh, float grid_delta);
 private:
  int quantization_bits_;
  float range_;
  std::vector<float> min_values_;
};

