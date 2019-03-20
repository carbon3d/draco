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

#include "draco/compression/encode.h"
#include "draco/core/decoder_buffer.h"
#include "draco/io/mesh_io.h"
#include "draco/mesh/mesh.h"
#include "draco/point_cloud/point_cloud.h"

typedef draco::GeometryAttribute::Type draco_GeometryAttribute_Type;
typedef draco::MeshEncoderMethod draco_MeshEncoderMethod;

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

class DracoInt8Array {
 public:
  DracoInt8Array() {}
  int8_t GetValue(int index) const {return values_[index];}
  bool SetValues(const char *values, int count) {
    values_.assign(values, values + count);
    return true;
  }
  size_t size() { return values_.size(); }
  emscripten::val GetView() {
    return emscripten::val(emscripten::typed_memory_view(values_.size(), &(values_[0])));
  }
  
 private:
  std::vector<int8_t> values_;
};


class Encoder {
 public:
  Encoder();

  void SetEncodingMethod(long method);
  void SetAttributeQuantization(draco_GeometryAttribute_Type type,
                                long quantization_bits);
  void SetAttributeExplicitQuantization(draco_GeometryAttribute_Type type,
                                        long quantization_bits,
                                        long num_components,
                                        float origin_x, float origin_y, float origin_z,
                                        float range);
  void SetSpeedOptions(long encoding_speed, long decoding_speed);
  void SetTrackEncodedProperties(bool flag);

  int EncodeMeshToDracoBuffer(draco::Mesh *mesh, DracoInt8Array *buffer);
  int EncodePointCloudToDracoBuffer(draco::PointCloud *pc,
                                    bool deduplicate_values,
                                    DracoInt8Array *buffer);
  int GetNumberOfEncodedPoints();
  int GetNumberOfEncodedFaces();

 private:
  draco::Encoder encoder_;
};

Encoder::Encoder() {}

void Encoder::SetEncodingMethod(long method) {
  encoder_.SetEncodingMethod(method);
}

void Encoder::SetAttributeQuantization(draco_GeometryAttribute_Type type,
                                       long quantization_bits) {
  encoder_.SetAttributeQuantization(type, quantization_bits);
}

void Encoder::SetAttributeExplicitQuantization(
    draco_GeometryAttribute_Type type, long quantization_bits,
    long num_components,  float origin_x, float origin_y, float origin_z,
    float range) {
  float origin[3] = {origin_x, origin_y, origin_z};
  encoder_.SetAttributeExplicitQuantization(type, quantization_bits,
                                            num_components, origin, range);
}

void Encoder::SetSpeedOptions(long encoding_speed, long decoding_speed) {
  encoder_.SetSpeedOptions(encoding_speed, decoding_speed);
}

void Encoder::SetTrackEncodedProperties(bool flag) {
  encoder_.SetTrackEncodedProperties(flag);
}

int Encoder::EncodeMeshToDracoBuffer(draco::Mesh *mesh, DracoInt8Array *draco_buffer) {
  if (!mesh)
    return 0;
  draco::EncoderBuffer buffer;
  if (mesh->GetNamedAttributeId(draco::GeometryAttribute::POSITION) == -1)
    return 0;
  if (!mesh->DeduplicateAttributeValues())
    return 0;
  mesh->DeduplicatePointIds();
  if (!encoder_.EncodeMeshToBuffer(*mesh, &buffer).ok()) {
    return 0;
  }
  draco_buffer->SetValues(buffer.data(), buffer.size());
  return buffer.size();
}

int Encoder::EncodePointCloudToDracoBuffer(draco::PointCloud *pc,
                                           bool deduplicate_values,
                                           DracoInt8Array *draco_buffer) {
  // TODO(ostava): Refactor common functionality with EncodeMeshToDracoBuffer().
  if (!pc)
    return 0;
  draco::EncoderBuffer buffer;
  if (pc->GetNamedAttributeId(draco::GeometryAttribute::POSITION) == -1)
    return 0;
  if (deduplicate_values) {
    if (!pc->DeduplicateAttributeValues())
      return 0;
    pc->DeduplicatePointIds();
  }
  if (!encoder_.EncodePointCloudToBuffer(*pc, &buffer).ok()) {
    return 0;
  }
  draco_buffer->SetValues(buffer.data(), buffer.size());
  return buffer.size();
}

int Encoder::GetNumberOfEncodedPoints() {
  return encoder_.num_encoded_points();
}

int Encoder::GetNumberOfEncodedFaces() { return encoder_.num_encoded_faces(); }

EMSCRIPTEN_BINDINGS(dracoEncoder) {
  emscripten::class_<DracoInt8Array>("DracoInt8Array")
      .constructor<>()
      .function("size", &DracoInt8Array::size)
      .function("GetView", &DracoInt8Array::GetView)
      ;

  emscripten::enum_<draco_GeometryAttribute_Type>("draco_GeometryAttribute_Type")
      .value("INVALID", draco::GeometryAttribute::INVALID)
      .value("POSITION", draco::GeometryAttribute::POSITION)
      .value("NORMAL", draco::GeometryAttribute::NORMAL)
      .value("COLOR", draco::GeometryAttribute::COLOR)
      .value("TEX_COORD", draco::GeometryAttribute::TEX_COORD)
      .value("GENERIC", draco::GeometryAttribute::GENERIC)
      ;

  emscripten::enum_<draco_MeshEncoderMethod>("draco_MeshEncoderMethod")
      .value("MESH_SEQUENTIAL_ENCODING", draco::MESH_SEQUENTIAL_ENCODING)
      .value("MESH_EDGEBREAKER_ENCODING", draco::MESH_EDGEBREAKER_ENCODING)
      ;

  
  emscripten::class_<draco::PointCloud>("PointCloud")
      .constructor<>()
      .function("num_attributes", &draco::PointCloud::num_attributes)
      .function("num_points", &draco::PointCloud::num_points)
      ;

  emscripten::class_<draco::Mesh, emscripten::base<draco::PointCloud>>("Mesh")
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

  emscripten::class_<Encoder>("Encoder")
      .constructor<>()
      .function("SetEncodingMethod", &Encoder::SetEncodingMethod)
      .function("SetAttributeQuantization", &Encoder::SetAttributeQuantization)
      .function("SetAttributeExplicitQuantization", &Encoder::SetAttributeExplicitQuantization,
                emscripten::allow_raw_pointers())
      .function("SetSpeedOptions", &Encoder::SetSpeedOptions)
      .function("SetTrackEncodedProperties", &Encoder::SetTrackEncodedProperties)
      .function("EncodeMeshToDracoBuffer", &Encoder::EncodeMeshToDracoBuffer,
                emscripten::allow_raw_pointers())
      .function("EncodePointCloudToDracoBuffer", &Encoder::EncodePointCloudToDracoBuffer,
                emscripten::allow_raw_pointers())
      .function("GetNumberOfEncodedPoints", &Encoder::GetNumberOfEncodedPoints)
      .function("GetNumberOfEncodedFaces", &Encoder::GetNumberOfEncodedFaces)
      ;
}
