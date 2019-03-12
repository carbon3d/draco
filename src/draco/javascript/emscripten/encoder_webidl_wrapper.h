// Copyright 2017 The Draco Authors.
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
#ifndef DRACO_JAVASCRIPT_EMSCRITPEN_ENCODER_WEBIDL_WRAPPER_H_
#define DRACO_JAVASCRIPT_EMSCRITPEN_ENCODER_WEBIDL_WRAPPER_H_

#include <vector>

#include "draco/attributes/point_attribute.h"
#include "draco/compression/config/compression_shared.h"
#include "draco/compression/config/encoder_options.h"
#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/core/decoder_buffer.h"
#include "draco/mesh/mesh.h"

typedef draco::GeometryAttribute draco_GeometryAttribute;
typedef draco::GeometryAttribute::Type draco_GeometryAttribute_Type;
typedef draco::EncodedGeometryType draco_EncodedGeometryType;
typedef draco::MeshEncoderMethod draco_MeshEncoderMethod;

class DracoInt8Array {
 public:
  DracoInt8Array();
  int8_t GetValue(int index) const;
  bool SetValues(const char *values, int count);

  size_t size() { return values_.size(); }

 private:
  std::vector<int8_t> values_;
};

class MetadataBuilder {
 public:
  MetadataBuilder();
  bool AddStringEntry(draco::Metadata *metadata, const char *entry_name,
                      const char *entry_value);
  bool AddIntEntry(draco::Metadata *metadata, const char *entry_name,
                   long entry_value);
  bool AddDoubleEntry(draco::Metadata *metadata, const char *entry_name,
                      double entry_value);
};

class PointCloudBuilder {
 public:
  PointCloudBuilder() {}
  int AddFloatAttribute(draco::PointCloud *pc,
                        draco_GeometryAttribute_Type type, long num_vertices,
                        long num_components, const float *att_values);
  int AddInt8Attribute(draco::PointCloud *pc, draco_GeometryAttribute_Type type,
                       long num_vertices, long num_components,
                       const char *att_values);
  int AddUInt8Attribute(draco::PointCloud *pc,
                        draco_GeometryAttribute_Type type, long num_vertices,
                        long num_components, const uint8_t *att_values);
  int AddInt16Attribute(draco::PointCloud *pc,
                        draco_GeometryAttribute_Type type, long num_vertices,
                        long num_components, const int16_t *att_values);
  int AddUInt16Attribute(draco::PointCloud *pc,
                         draco_GeometryAttribute_Type type, long num_vertices,
                         long num_components, const uint16_t *att_values);
  int AddInt32Attribute(draco::PointCloud *pc,
                        draco_GeometryAttribute_Type type, long num_vertices,
                        long num_components, const int32_t *att_values);
  int AddUInt32Attribute(draco::PointCloud *pc,
                         draco_GeometryAttribute_Type type, long num_vertices,
                         long num_components, const uint32_t *att_values);
  bool SetMetadataForAttribute(draco::PointCloud *pc, long attribute_id,
                               const draco::Metadata *metadata);
  bool AddMetadata(draco::PointCloud *pc, const draco::Metadata *metadata);

 private:
  template <typename DataTypeT>
  int AddAttribute(draco::PointCloud *pc, draco_GeometryAttribute_Type type,
                   long num_vertices, long num_components,
                   const DataTypeT *att_values,
                   draco::DataType draco_data_type) {
    if (!pc)
      return -1;
    draco::PointAttribute att;
    att.Init(type, NULL, num_components, draco_data_type,
             /* normalized */ false,
             /* stride */ sizeof(DataTypeT) * num_components,
             /* byte_offset */ 0);
    const int att_id =
        pc->AddAttribute(att, /* identity_mapping */ true, num_vertices);
    draco::PointAttribute *const att_ptr = pc->attribute(att_id);

    for (draco::PointIndex i(0); i < num_vertices; ++i) {
      att_ptr->SetAttributeValue(att_ptr->mapped_index(i),
                                 &att_values[i.value() * num_components]);
    }
    if (pc->num_points() == 0) {
      pc->set_num_points(num_vertices);
    } else if (pc->num_points() != num_vertices) {
      return -1;
    }
    return att_id;
  }
};

// TODO(draco-eng): Regenerate wasm decoder.
// TODO(draco-eng): Add script to generate and test all Javascipt code.
class MeshBuilder : public PointCloudBuilder {
 public:
  MeshBuilder();
  // Decodes a triangular mesh from the provided buffer.
  // The buffer can be the data from an obj, stl, ply, or draco encoded file.
  //  file_type should be the three letter file extension, so one of obj stl ply or
  //  if not one of those it is assumed to be a draco encoded mesh.
  bool DecodeFileBufferToMesh(const char *data, size_t data_size,
                              const char *file_type,
                              draco::Mesh *out_mesh);

  bool SetNumFaces(draco::Mesh *mesh, long num_faces);
  
  bool AddFacesToMesh(draco::Mesh *mesh, long num_faces, const int *faces);

  // Deprecated: Use AddFloatAttribute() instead.
  int AddFloatAttributeToMesh(draco::Mesh *mesh,
                              draco_GeometryAttribute_Type type,
                              long num_vertices, long num_components,
                              const float *att_values);

  // Deprecated: Use AddInt32Attribute() instead.
  int AddInt32AttributeToMesh(draco::Mesh *mesh,
                              draco_GeometryAttribute_Type type,
                              long num_vertices, long num_components,
                              const int32_t *att_values);

  // Deprecated: Use AddMetadata() instead.
  bool AddMetadataToMesh(draco::Mesh *mesh, const draco::Metadata *metadata);
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
                                        const float *origin, float range);
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

class ExpertEncoder {
 public:
  ExpertEncoder(draco::PointCloud *pc);

  void SetEncodingMethod(long method);
  void SetAttributeQuantization(long att_id, long quantization_bits);
  void SetAttributeExplicitQuantization(long att_id, long quantization_bits,
                                        long num_components,
                                        const float *origin, float range);
  void SetSpeedOptions(long encoding_speed, long decoding_speed);
  void SetTrackEncodedProperties(bool flag);

  int EncodeToDracoBuffer(bool deduplicate_values, DracoInt8Array *buffer);

  int GetNumberOfEncodedPoints();
  int GetNumberOfEncodedFaces();

 private:
  std::unique_ptr<draco::ExpertEncoder> encoder_;

  draco::PointCloud *pc_;
};


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


#endif  // DRACO_JAVASCRIPT_EMSCRITPEN_ENCODER_WEBIDL_WRAPPER_H_
