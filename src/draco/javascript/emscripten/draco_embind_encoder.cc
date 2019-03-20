// Copyright 2019 Carbon
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <math.h>

#include <iostream>

#include "draco/core/decoder_buffer.h"
#include "draco/io/mesh_io.h"
#include "draco/mesh/mesh.h"


class MeshQuantizationCarbon {
 public:
  MeshQuantizationCarbon() :
      quantization_bits_(-1),
      range_(0),
      min_values_(3, 0)
  {}
  bool IsSet() {return quantization_bits_ == -1;}
  int quantization_bits() const {return quantization_bits_;}
  float range() const {return range_;}
  float min_values_x() {return min_values_[0];}
  float min_values_y() {return min_values_[1];}
  float min_values_z() {return min_values_[2];}
  bool FillFromMesh(draco::Mesh *mesh, float grid_delta);
 private:
  int quantization_bits_;
  float range_;
  std::vector<float> min_values_;
};

bool MeshQuantizationCarbon::FillFromMesh(draco::Mesh *mesh, float grid_delta) {
  constexpr int kMaxNumQuantizationBits = 30;
  if (grid_delta < 0) return false;  
  const draco::PointAttribute *const pos_att =
      mesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
  const int num_components = pos_att->num_components();
  if (num_components != 3) return false;
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
  return true;
}

class DecoderBufferOwner {
 public:
  DecoderBufferOwner(size_t buffer_size) : buffer_(buffer_size, 0) {
    decoder_buffer_.Init(&(buffer_[0]), buffer_size);
  }
  draco::DecoderBuffer* GetDecoderBuffer() {return &decoder_buffer_;}
  emscripten::val GetBufferView() {
    return emscripten::val(emscripten::typed_memory_view(buffer_.size(), &(buffer_[0])));
  }
  
 private:
  draco::DecoderBuffer decoder_buffer_;
  std::vector<char> buffer_;
};

bool DecodeFileBufferToMesh(draco::DecoderBuffer* buffer,
                            std::string file_type,
                            draco::Mesh *out_mesh) {
  // Reads a mesh from a decoder buffer.
  const std::string extension(file_type);
  draco::Status status = draco::ReadMeshFromBuffer(buffer, draco::Options(),
                                                   extension, out_mesh);
  if (! status.ok()) {
    std::cerr << "Reading mesh from buffer yielded " << status.error_msg()
              << std::endl;
  }
  return status.ok();
}

EMSCRIPTEN_BINDINGS(dracoEncoder) {
  emscripten::class_<draco::Mesh>("Mesh")
      .constructor<>()
      .function("num_faces", &draco::Mesh::num_faces)
      ;
  emscripten::class_<draco::DecoderBuffer>("DecoderBuffer")
      .constructor<>()
      .function("Init", emscripten::select_overload<void(const char*, size_t)>(
          &draco::DecoderBuffer::Init), emscripten::allow_raw_pointers());
      ;
  emscripten::class_<DecoderBufferOwner>("DecoderBufferOwner")
      .constructor<size_t>()
      .function("GetDecoderBuffer", &DecoderBufferOwner::GetDecoderBuffer,
                emscripten::allow_raw_pointers())
      .function("GetBufferView", &DecoderBufferOwner::GetBufferView)
      ;
  emscripten::class_<MeshQuantizationCarbon>("MeshQuantizationCarbon")
      .constructor<>()
      .function("IsSet", &MeshQuantizationCarbon::IsSet)
      .function("quantization_bits", &MeshQuantizationCarbon::quantization_bits)
      .function("range", &MeshQuantizationCarbon::range)
      .function("min_values_x", &MeshQuantizationCarbon::min_values_x)
      .function("min_values_y", &MeshQuantizationCarbon::min_values_y)
      .function("min_values_z", &MeshQuantizationCarbon::min_values_z)
      .function("FillFromMesh", &MeshQuantizationCarbon::FillFromMesh,
                emscripten::allow_raw_pointers())
      ;
  emscripten::function("DecodeFileBufferToMesh", DecodeFileBufferToMesh,
                       emscripten::allow_raw_pointers());
}
