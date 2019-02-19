// Copyright 2016 The Draco Authors.
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
#include "draco/io/stl_decoder.h"

#include <cstring>
#include <fstream>
#include <limits>
#include <string>

#include "draco/io/file_utils.h"
#include "draco/io/parser_utils.h"

#include <iostream>

namespace draco {

StlDecoder::StlDecoder()
    : num_stl_faces_(0),
      is_binary_mode_(true),
      attribute_element_types_(2, -1),
      out_mesh_(nullptr)
{}

Status StlDecoder::DecodeFromFile(const std::string &file_name, Mesh *out_mesh) {
  std::ifstream file(file_name, std::ios::binary);
  if (!file)
    return Status(Status::IO_ERROR);
  // Read the whole file into a buffer.
  auto pos0 = file.tellg();
  file.seekg(0, std::ios::end);
  auto file_size = file.tellg() - pos0;
  if (file_size == 0)
    return Status(Status::IO_ERROR);
  file.seekg(0, std::ios::beg);
  std::vector<char> data(file_size);
  file.read(&data[0], file_size);
  buffer_.Init(&data[0], file_size);

  out_mesh_ = out_mesh;
  return DecodeInternal();
}

Status StlDecoder::DecodeFromBuffer(DecoderBuffer *buffer, Mesh *out_mesh) {
  out_mesh_ = out_mesh;
  buffer_.Init(buffer->data_head(), buffer->remaining_size());  
  return DecodeInternal();
}

Status StlDecoder::ParseHeader() {
  char ascii_buffer[5];
  parser::SkipWhitespace(buffer());
  if (! buffer()->Decode(ascii_buffer, 5)) {
    return Status(Status::IO_ERROR, "STL file has invalid header.");
  }
  // If the file begins with "solid" it is likely an ascii stl file
  if (! strncmp(ascii_buffer, "solid ", 5)) {
    std::string tmp_str;
    num_stl_faces_ = 0;
    is_binary_mode_ = false;
    parser::SkipWhitespace(buffer());
    int64_t buffer_name_seek_point = buffer()->decoded_size();
    if (! parser::ParseString(buffer(), &tmp_str)) {
      return Status(Status::IO_ERROR, "STL file is missing face data.");
    }
    // The ascii stl file format allows for an optional name parameter after
    // the word solid before listing the facets.  Check for the weird choice of
    // facet as a solid name
    if (tmp_str == "facet") {
      std::string repeat_str;
      int64_t buffer_post_facet_seek_point = buffer()->decoded_size();      
      if (! parser::ParseString(buffer(), &repeat_str)) {
        return Status(Status::IO_ERROR, "STL file has invalid header.");        
      }
      if (repeat_str == "facet") {
        solid_name_ = tmp_str;
        buffer()->StartDecodingFrom(buffer_post_facet_seek_point);
      } else {
        buffer()->StartDecodingFrom(buffer_name_seek_point);
      }
    } else {
      std::cout << " WE SHALL NAME OUR SOLID " << tmp_str << std::endl;
      solid_name_ = tmp_str;
    }
  } else {
    buffer()->StartDecodingFrom(80);
    uint32_t tmp_num_faces = 0;
    if (!buffer()->Decode<uint32_t>(&tmp_num_faces)) {
      return Status(Status::IO_ERROR, "Binary STL file has invalid header.");
    }
    num_stl_faces_ = tmp_num_faces;
    is_binary_mode_ = true;
  }
  return Status(Status::OK);
}

Status StlDecoder::ParseBinaryFace(Vector3f* v0, Vector3f* v1, Vector3f* v2,
                                   Vector3f* normal) {
  const Status facet_error(Status::IO_ERROR, "Incomplete STL facet description.");  
  // While not specified in the standard, floats in the binary stl are typically little endian
  for (int i = 0; i < 3; ++i) {
    if (! buffer()->Decode(&((*normal)[i]))) return facet_error;
  }
  for (int i = 0; i < 3; ++i) {
    if (! buffer()->Decode(&((*v0)[i]))) return facet_error;
  }
  for (int i = 0; i < 3; ++i) {
    if (! buffer()->Decode(&((*v1)[i]))) return facet_error;
  }
  for (int i = 0; i < 3; ++i) {
    if (! buffer()->Decode(&((*v2)[i]))) return facet_error;
  }
  buffer()->Advance(2);
  return Status(Status::OK);
}

Status StlDecoder::ParseAsciiFace(Vector3f* v0, Vector3f* v1, Vector3f* v2, Vector3f* normal,
                                  bool* is_valid_triangle) {
  // The ASCII stl file format describes triangles in the form:
  //
  // facet normal ni nj nk
  //     outer loop
  //     vertex v1x v1y v1z
  //     vertex v2x v2y v2z
  //     vertex v3x v3y v3z
  //     endloop
  // endfacet
  auto ExpectString = [&] (std::string expected) -> bool {
    std::string tmp_str;
    if (!parser::ParseString(buffer(), &tmp_str)) return false;
    if (tmp_str != expected) {
      std::cout << "Expected '" << expected << "' Got '" << tmp_str << "'" << std::endl;
      return false;
    }
    return true;
  };
  auto FillThreeVec = [&] (Vector3f* vec) -> Status {
    std::string tmp_str;
    for (int i = 0; i < 3; ++i) {
      try {
        if (! parser::ParseString(buffer(), &tmp_str)) {
          return Status(Status::IO_ERROR, "Invalid float in STL facet description.");
        }
        (*vec)[i] = stof(tmp_str);
      } catch (const std::invalid_argument& ia) {
        return Status(Status::IO_ERROR, "Invalid float in STL facet description.");
      }
    }
    return Status(Status::OK);
  };
  *is_valid_triangle = false;
  const Status facet_error(Status::IO_ERROR, "Invalid STL facet description.");
  Status status(Status::OK);
  std::string tmp_str;
  if (!parser::ParseString(buffer(), &tmp_str)) return facet_error;
  if (tmp_str == "endsolid") {
    return status;
  } else if (tmp_str != "facet") return facet_error;
  if (! ExpectString("normal")) return facet_error;
  status = FillThreeVec(normal);
  if (! status.ok()) return status;
  if (! ExpectString("outer")) return facet_error;
  if (! ExpectString("loop")) return facet_error;
  if (! ExpectString("vertex")) return facet_error;
  status = FillThreeVec(v0);
  if (! status.ok()) return status;
  if (! ExpectString("vertex")) return facet_error;
  status = FillThreeVec(v1);
  if (! status.ok()) return status;
  if (! ExpectString("vertex")) return facet_error;  
  status = FillThreeVec(v2);
  if (! status.ok()) return status;
  if (! ExpectString("endloop")) return facet_error;
  if (! ExpectString("endfacet")) return facet_error;
  *is_valid_triangle = true;  
  return status;
}

Status StlDecoder::DecodeInternal() {
  Vector3f tmp_norm;
  Vector3f tmp_v0;
  Vector3f tmp_v1;
  Vector3f tmp_v2;
  std::vector<Vector3f> tmp_three_vec_storage;
  Status status(Status::OK);
  status = ParseHeader();
  if (! status.ok()) return status;
  if (! is_binary_mode_) {
    // For the ASCII formatted STL file we do not know how many triangles are specified in the file
    // without reading the entire file.  Fortunately, the ascii encoding is so innefficient we
    // can, hopefully, assume that few enough vertices are stored that
    // an extra copy of the vertex information will not require too much memory use
    bool is_valid_triangle;
    Status status;
    do {
      status = ParseAsciiFace(&tmp_v0, &tmp_v1, &tmp_v2, &tmp_norm, &is_valid_triangle);
      if (! status.ok()) return status;
      if (is_valid_triangle) {
        tmp_three_vec_storage.push_back(tmp_norm);
        tmp_three_vec_storage.push_back(tmp_v0);
        tmp_three_vec_storage.push_back(tmp_v1);
        tmp_three_vec_storage.push_back(tmp_v2);
      }
    } while (is_valid_triangle);
    num_stl_faces_ = tmp_three_vec_storage.size() / 4;
  }
  out_mesh_->SetNumFaces(num_stl_faces_);
  out_mesh_->set_num_points(num_stl_faces_ * 3);

  GeometryAttribute pos_va;
  pos_va.Init(GeometryAttribute::POSITION, nullptr, 3, DT_FLOAT32, false,
              DataTypeLength(DT_FLOAT32) * 3, 0);
  const int pos_att_id = out_mesh_->AddAttribute(pos_va, true, out_mesh_->num_points());

  GeometryAttribute norm_va;
  norm_va.Init(GeometryAttribute::NORMAL, nullptr, 3, DT_FLOAT32, false,
               DataTypeLength(DT_FLOAT32) * 3, 0);
  const int norm_att_id = out_mesh_->AddAttribute(norm_va, true, out_mesh_->num_points());

  if (norm_att_id >= 2 || pos_att_id) {
    return Status(Status::IO_ERROR, "The programmers understanding of this library is poor");
  }
  PointAttribute *const pos_att = out_mesh_->attribute(pos_att_id);
  PointAttribute *const norm_att = out_mesh_->attribute(norm_att_id);
  attribute_element_types_[pos_att_id] = MESH_CORNER_ATTRIBUTE;
  attribute_element_types_[norm_att_id] = MESH_FACE_ATTRIBUTE;
  
  for (int i = 0; i < num_stl_faces_; ++i) {
    // Read a triangle face
    if (is_binary_mode_) {
      status = ParseBinaryFace(&tmp_v0, &tmp_v1, &tmp_v2, &tmp_norm);
      if (! status.ok()) return status;
    } else {
      tmp_norm = tmp_three_vec_storage[4 * i];
      tmp_v0 = tmp_three_vec_storage[4 * i + 1];
      tmp_v1 = tmp_three_vec_storage[4 * i + 2];
      tmp_v2 = tmp_three_vec_storage[4 * i + 3];
      std::cout << "Norm " << tmp_norm[0] << " " << tmp_norm[1] << " " << tmp_norm[2] << std::endl;
      std::cout << "V0 " << tmp_v0[0] << " " << tmp_v0[1] << " " << tmp_v0[2] << std::endl;
      std::cout << "V1 " << tmp_v1[0] << " " << tmp_v1[1] << " " << tmp_v1[2] << std::endl;
      std::cout << "V2 " << tmp_v2[0] << " " << tmp_v2[1] << " " << tmp_v2[2] << std::endl;
    }
    // Store the values in the mesh
    FaceIndex face_id(i);
    const int start_index = 3 * face_id.value();
    pos_att->SetAttributeValue(AttributeValueIndex(start_index), tmp_v0.data());
    pos_att->SetAttributeValue(AttributeValueIndex(start_index + 1), tmp_v1.data());
    pos_att->SetAttributeValue(AttributeValueIndex(start_index + 2), tmp_v2.data());

    norm_att->SetAttributeValue(AttributeValueIndex(start_index), tmp_norm.data());
    norm_att->SetAttributeValue(AttributeValueIndex(start_index + 1), tmp_norm.data());
    norm_att->SetAttributeValue(AttributeValueIndex(start_index + 2), tmp_norm.data());

    out_mesh_->SetFace(face_id,
                       {{PointIndex(start_index), PointIndex(start_index + 1),
                               PointIndex(start_index + 2)}});
  }
  std::cout << "Starting dedup" << std::endl;
#ifdef DRACO_ATTRIBUTE_VALUES_DEDUPLICATION_SUPPORTED
  // First deduplicate attribute values.
  std::cout << " dedup vals " << std::endl;    
  if (!out_mesh_->DeduplicateAttributeValues())
    return status;
#endif
#ifdef DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED
  std::cout << " dedup pnts " << std::endl;  
  // Also deduplicate vertex indices.
  out_mesh_->DeduplicatePointIds();
#endif

  for (size_t i = 0; i < attribute_element_types_.size(); ++i) {
    if (attribute_element_types_[i] >= 0) {
      out_mesh_->SetAttributeElementType(
          static_cast<int>(i),
          static_cast<MeshAttributeElementType>(attribute_element_types_[i]));
    }
  }
  std::cout << " Done dedup " << std::endl;
  return status;
 }
}  // namespace draco
