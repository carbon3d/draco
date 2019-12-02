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
#include <cmath>
#include <fstream>
#include <limits>
#include <string>

#include "draco/io/file_utils.h"
#include "draco/io/parser_utils.h"

namespace draco {

StlDecoder::StlDecoder()
    : num_stl_faces_(0),
      attribute_element_types_(1, -1),
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

Status StlDecoder::ParseHeader(bool force_binary, bool* is_binary) {
  parser::SkipWhitespace(&buffer_);
  bool is_ascii_file;
  if (force_binary) {
    is_ascii_file = false;
  } else {
    char ascii_buffer[5];
    if (! buffer_.Decode(ascii_buffer, 5)) {
      return Status(Status::IO_ERROR, "STL file has invalid header.");
    }
    is_ascii_file = ! strncmp(ascii_buffer, "solid", 5);
  }
  // If the file begins with "solid" it is likely an ascii stl file
  if (is_ascii_file) {
    std::string tmp_str;
    num_stl_faces_ = 0;
    (*is_binary) = false;
    int64_t buffer_seek_point = buffer_.decoded_size();
    int loop_counter = 0;
    do {
      parser::SkipWhitespace(&buffer_);
      buffer_seek_point = buffer_.decoded_size();
      if (! parser::ParseString(&buffer_, &tmp_str)) {
        return Status(Status::IO_ERROR, "STL file is missing face data.");
      }
      // If the file is not composed of a series of strings or we loop too much on this, it is likely
      // a binary stl.
      if (buffer_seek_point == buffer_.decoded_size() || loop_counter > 4) {
        is_ascii_file = false;
      }
      ++loop_counter;
    } while (tmp_str != "facet" && is_ascii_file);
    if (is_ascii_file) {
      buffer_.StartDecodingFrom(buffer_seek_point);
    }
  }
  if (! is_ascii_file) {
    buffer_.StartDecodingFrom(80);
    uint32_t tmp_num_faces = 0;
    if (!buffer_.Decode<uint32_t>(&tmp_num_faces)) {
      return Status(Status::IO_ERROR, "Binary STL file has invalid header.");
    }
    num_stl_faces_ = tmp_num_faces;
    (*is_binary) = true;
  }
  return Status(Status::OK);
}

Status StlDecoder::ParseBinaryFace(Vector3f* v0, Vector3f* v1, Vector3f* v2,
                                   Vector3f* normal) {
  const Status facet_error(Status::IO_ERROR, "Incomplete STL facet description.");
  float vec_buffer[12];
  if (!buffer_.Decode(vec_buffer, 12 * sizeof(float))) return facet_error;
  buffer_.Advance(2);
  // While not specified in the standard, floats in the binary stl are typically little endian
  for (int i = 0; i < 3; ++i) (*normal)[i] = vec_buffer[i + 0];
  for (int i = 0; i < 3; ++i) (*v0)[i] = vec_buffer[i + 3];
  for (int i = 0; i < 3; ++i) (*v1)[i] = vec_buffer[i + 6];
  for (int i = 0; i < 3; ++i) (*v2)[i] = vec_buffer[i + 9];
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
    if (!parser::ParseString(&buffer_, &tmp_str)) return false;
    if (tmp_str != expected) {
      return false;
    }
    return true;
  };
  auto FillThreeVec = [&] (Vector3f* vec) -> Status {
    std::string tmp_str;
    for (int i = 0; i < 3; ++i) {
      try {
        if (! parser::ParseString(&buffer_, &tmp_str)) {
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
  if (!parser::ParseString(&buffer_, &tmp_str)) return facet_error;
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
  bool is_binary = false;
  status = ParseHeader(false, &is_binary);
  if (! status.ok()) return status;
  if (! is_binary) {
    // For the ASCII formatted STL file we do not know how many triangles are specified in the file
    // without reading the entire file.  We attempt to read as many ascii solids as possible.
    // If we have an error while parsing the first ascii solid, we assume the file is a binary stl
    // and fall back to that.
    bool error_while_parsing_ascii = false;
    bool is_first_solid = true;
    bool is_valid_triangle;
    Status status;
    while (1) {
      do {
        status = ParseAsciiFace(&tmp_v0, &tmp_v1, &tmp_v2, &tmp_norm, &is_valid_triangle);
        if (! status.ok()) {
          // if we have an error while parsing the face data only assume it's a binary stl if this
          // is the first solid
          error_while_parsing_ascii = is_first_solid;
          break;
        }
        if (is_valid_triangle) {
          tmp_three_vec_storage.push_back(tmp_norm);
          tmp_three_vec_storage.push_back(tmp_v0);
          tmp_three_vec_storage.push_back(tmp_v1);
          tmp_three_vec_storage.push_back(tmp_v2);
        }
      } while (is_valid_triangle);
      bool is_binary_local;
      if (error_while_parsing_ascii || buffer_.remaining_size() < 5 || ! ParseHeader(false, &is_binary_local).ok() || is_binary_local) {
        break;
      }
      is_first_solid = false;
    }
    if (error_while_parsing_ascii) {
      status = ParseHeader(true, &is_binary);
      if (! status.ok()) return status;
    } else {
      num_stl_faces_ = tmp_three_vec_storage.size() / 4;
    }
  }
  out_mesh_->SetNumFaces(num_stl_faces_);
  out_mesh_->set_num_points(num_stl_faces_ * 3);

  GeometryAttribute pos_va;
  pos_va.Init(GeometryAttribute::POSITION, nullptr, 3, DT_FLOAT32, false,
              DataTypeLength(DT_FLOAT32) * 3, 0);
  const int pos_att_id = out_mesh_->AddAttribute(pos_va, true, out_mesh_->num_points());
  PointAttribute *const pos_att = out_mesh_->attribute(pos_att_id);
  attribute_element_types_[pos_att_id] = MESH_VERTEX_ATTRIBUTE;
  for (int i = 0; i < num_stl_faces_; ++i) {
    // Read a triangle face
    if (is_binary) {
      status = ParseBinaryFace(&tmp_v0, &tmp_v1, &tmp_v2, &tmp_norm);
      if (! status.ok()) return status;
    } else {
      tmp_norm = tmp_three_vec_storage[4 * i];
      tmp_v0 = tmp_three_vec_storage[4 * i + 1];
      tmp_v1 = tmp_three_vec_storage[4 * i + 2];
      tmp_v2 = tmp_three_vec_storage[4 * i + 3];
    }
    // We intentionally do not check for NaN's in the normal.
    // We have encoded the normal information in the winding order of the vertices of the triangle
    // so the normal is only useful when setting that winding order.
    bool finite_0 = std::isfinite(tmp_v0[0]) && std::isfinite(tmp_v0[1]) && std::isfinite(tmp_v0[2]);
    bool finite_1 = std::isfinite(tmp_v1[0]) && std::isfinite(tmp_v1[1]) && std::isfinite(tmp_v1[2]);
    bool finite_2 = std::isfinite(tmp_v2[0]) && std::isfinite(tmp_v2[1]) && std::isfinite(tmp_v2[2]);
    if (! (finite_0 && finite_1 && finite_2)) {
      if (! (finite_0 || finite_1 || finite_2)) {
        return Status(Status::IO_ERROR, "Every vertex in a triangle is NaN.");
      }      
      const Vector3f& finite_vert = (finite_0 ? tmp_v0 : (finite_1 ? tmp_v1 : tmp_v2));
      if (! finite_0) tmp_v0 = finite_vert;
      if (! finite_1) tmp_v1 = finite_vert;
      if (! finite_2) tmp_v2 = finite_vert;
    }
    // Store the values in the mesh
    FaceIndex face_id(i);
    const int start_index = 3 * face_id.value();
    pos_att->SetAttributeValue(AttributeValueIndex(start_index), tmp_v0.data());
    pos_att->SetAttributeValue(AttributeValueIndex(start_index + 1), tmp_v1.data());
    pos_att->SetAttributeValue(AttributeValueIndex(start_index + 2), tmp_v2.data());
    out_mesh_->SetFace(face_id,
                       {{PointIndex(start_index), PointIndex(start_index + 1),
                               PointIndex(start_index + 2)}});
  }

#ifdef DRACO_ATTRIBUTE_VALUES_DEDUPLICATION_SUPPORTED
  // First deduplicate attribute values.
  if (!out_mesh_->DeduplicateAttributeValues())return status;
#endif
#ifdef DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED
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
  return status;
}

}  // namespace draco
