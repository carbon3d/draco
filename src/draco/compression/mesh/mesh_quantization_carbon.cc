#include "draco/compression/mesh/mesh_quantization_carbon.h"


std::string MeshQuantizationCarbon::FillFromMesh(draco::Mesh *mesh, float grid_delta) {
  constexpr int kMaxNumQuantizationBits = 30;
  if (grid_delta < 0) return "Negative Grid Delta";  
  const draco::PointAttribute *const pos_att =
      mesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
  const int num_components = pos_att->num_components();
  if (num_components != 3) return "The position attribute does not have 3 values.";
  range_ = 0.f;
  min_values_ = std::vector<float>(num_components, 0.f);
  const std::unique_ptr<float[]> max_values(new float[num_components]);
  const std::unique_ptr<float[]> att_val(new float[num_components]);
  pos_att->GetValue(draco::AttributeValueIndex(0), att_val.get());
  pos_att->GetValue(draco::AttributeValueIndex(0), min_values_.data());
  pos_att->GetValue(draco::AttributeValueIndex(0), max_values.get());
  for (draco::AttributeValueIndex i(1); i < static_cast<uint32_t>(pos_att->size());
       ++i) {
    pos_att->GetValue(i, att_val.get());
    for (int c = 0; c < num_components; ++c) {
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
  quantization_bits_ = ceilf(log(range_ / grid_delta) / log(2));
  if (quantization_bits_ > kMaxNumQuantizationBits) {
    quantization_bits_ = kMaxNumQuantizationBits;
  } else {
    range_ = grid_delta * powf(2, quantization_bits_);
  }
  return "";
}
