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
#include "draco/compression/mesh/mesh_quantization_carbon.h"
#include "draco/core/decoder_buffer.h"
#include "draco/io/file_utils.h"
#include "draco/io/obj_decoder.h"
#include "draco/io/mesh_io.h"
#include "draco/io/parser_utils.h"
#include "draco/io/ply_decoder.h"
#include "draco/io/stl_decoder.h"
#include "draco/mesh/mesh.h"
#include "draco/point_cloud/point_cloud.h"


typedef draco::GeometryAttribute::Type draco_GeometryAttribute_Type;
typedef draco::MeshEncoderMethod draco_MeshEncoderMethod;

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

draco::Status ReadMeshFromBuffer(draco::DecoderBuffer* buffer,
                                 const draco::Options &options,
                                 const std::string file_type,
                                 draco::Mesh* mesh) {
  std::string extension = draco::parser::ToLower(file_type);
  // Analyze file extension.
  if (extension == "obj") {
    // Wavefront OBJ file format.
    draco::ObjDecoder obj_decoder;
    obj_decoder.set_use_metadata(options.GetBool("use_metadata", false));
    const draco::Status obj_status = obj_decoder.DecodeFromBuffer(buffer, mesh);
    return obj_status;
  }
  if (extension == "ply") {
    // Wavefront PLY file format.
    draco::PlyDecoder ply_decoder;
    return ply_decoder.DecodeFromBuffer(buffer, mesh);
  }
  if (extension == "stl") {
    // STL file format.
    draco::StlDecoder stl_decoder;
    return stl_decoder.DecodeFromBuffer(buffer, mesh);
  }
  return draco::Status(draco::Status::ERROR, "Unknown file type");
}

std::string DecodeFileBufferToMesh(draco::DecoderBuffer* buffer,
                                   std::string file_type,
                                   draco::Mesh *out_mesh) {
  draco::Status status;
  if (file_type == "drc") {
    draco::Decoder decoder;
    status = decoder.DecodeBufferToGeometry(buffer, out_mesh);
  } else {
    // Reads a mesh from a decoder buffer.
    status = ReadMeshFromBuffer(buffer, draco::Options(),
                                file_type, out_mesh);
  }
  if (! status.ok()) return status.error_msg();
  else return "";
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

struct TriangleSpec {
  unsigned int v0;
  unsigned int v1;
  unsigned int v2;
};

struct PointSpec {
  float x;
  float y;
  float z;
};

TriangleSpec GetFace(draco::Mesh* mesh, int face_index) {
  TriangleSpec tri_spec;
  draco::Mesh::Face face = mesh->face(draco::FaceIndex(face_index));
  tri_spec.v0 = face[0].value();
  tri_spec.v1 = face[1].value();
  tri_spec.v2 = face[2].value();
  return tri_spec;
}

PointSpec GetPoint(draco::Mesh* mesh, int point_index) {
  const int pos_att_id = mesh->GetNamedAttributeId(draco::GeometryAttribute::POSITION);
  const auto *const pos_att = mesh->attribute(pos_att_id);
  PointSpec point_spec;
  draco::Vector3f vs;
  pos_att->GetMappedValue(draco::PointIndex(point_index), &vs);
  point_spec.x = vs[0];
  point_spec.y = vs[1];
  point_spec.z = vs[2];  
  return point_spec;
}

EMSCRIPTEN_BINDINGS(DracoEncoder) {
  emscripten::class_<TriangleSpec>("TriangleSpec")
      .property("v0", &TriangleSpec::v0)
      .property("v1", &TriangleSpec::v1)
      .property("v2", &TriangleSpec::v2)
      ;
  emscripten::class_<PointSpec>("PointSpec")
      .property("x", &PointSpec::x)
      .property("y", &PointSpec::y)
      .property("z", &PointSpec::z)
      ;
  
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

  emscripten::function("GetFace", GetFace, emscripten::allow_raw_pointers());
  emscripten::function("GetPoint", GetPoint, emscripten::allow_raw_pointers());

}
