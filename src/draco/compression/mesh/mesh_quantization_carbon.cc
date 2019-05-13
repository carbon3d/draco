#include "draco/compression/mesh/mesh_quantization_carbon.h"


std::string MeshQuantizationCarbon::FillFromMesh(draco::Mesh *mesh, float grid_delta) {
  constexpr int kMaxNumQuantizationBits = 30;
  constexpr int kMinNumQuantizationBits = 1;
  if (grid_delta < 0) return "Negative Grid Delta";  
  const draco::PointAttribute *const pos_att =
      mesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
  const int num_components = pos_att->num_components();
  if (num_components != 3) return "The position attribute does not have 3 values.";
  range_ = 0.f;
  float max_values[3];
  float att_val[3];
  pos_att->GetValue(draco::AttributeValueIndex(0), min_values_);
  pos_att->GetValue(draco::AttributeValueIndex(0), max_values);
  for (draco::AttributeValueIndex i(1); i < pos_att->size(); ++i) {
    pos_att->GetValue(i, att_val);
    for (int c = 0; c < 3; ++c) {
      if (min_values_[c] > att_val[c])
        min_values_[c] = att_val[c];
      if (max_values[c] < att_val[c])
        max_values[c] = att_val[c];
    }
  }
  for (int c = 0; c < num_components; ++c) {
    const float dif = max_values[c] - min_values_[c];
    if (dif > range_)
      range_ = dif;
  }
  // In case all values are the same, initialize the range to unit length. This
  // will ensure that all values are quantized properly to the same value.
  if (range_ == 0.f) range_ = 1.f;
  quantization_bits_ = ceilf(log((range_ / grid_delta) + 1) / log(2));
  if (quantization_bits_ > kMaxNumQuantizationBits) {
    quantization_bits_ = kMaxNumQuantizationBits;
  } else if (quantization_bits_ < kMinNumQuantizationBits) {
    quantization_bits_ = kMinNumQuantizationBits;
  } else {
    range_ = grid_delta * (powf(2, quantization_bits_) - 1);
  }
  return "";
}
