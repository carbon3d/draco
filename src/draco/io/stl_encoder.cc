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
#include "draco/io/stl_encoder.h"

#include <array>
#include <fstream>
#include <limits>

#include "draco/metadata/geometry_metadata.h"

namespace draco {

StlEncoder::StlEncoder()
    : pos_att_(nullptr),
      normal_att_(nullptr),
      out_buffer_(nullptr)
{}

bool StlEncoder::EncodeToFile(const Mesh &mesh, const std::string &file_name) {
  std::ofstream file(file_name);
  if (!file)
    return false;  // File could not be opened.
  file_name_ = file_name;
  // Encode the mesh into a buffer.
  EncoderBuffer buffer;
  if (!EncodeToBuffer(mesh, &buffer))
    return false;
  // Write the buffer into the file.
  file.write(buffer.data(), buffer.size());
  return true;
}

bool StlEncoder::EncodeToBuffer(const Mesh &mesh, EncoderBuffer *out_buffer) {
  in_mesh_ = &mesh;
  out_buffer_ = out_buffer;
  if (!EncodeInternal())
    return ExitAndCleanup(false);
  return ExitAndCleanup(true);
}

bool StlEncoder::ExitAndCleanup(bool return_value) {
  in_mesh_ = nullptr;
  out_buffer_ = nullptr;
  pos_att_ = nullptr;
  normal_att_ = nullptr;
  file_name_.clear();
  return return_value;
}

bool StlEncoder::EncodeFloatList(float* floats, uint32_t num_floats) {
  for (uint32_t i = 0; i < num_floats; ++i) {
    if (! buffer()->Encode<float>(*(floats + i))) return false;
  }
  return true;
}

bool StlEncoder::EncodeInternal() {
  // We require position and normal information
  const PointAttribute *const pos_att =
      in_mesh_->GetNamedAttribute(GeometryAttribute::POSITION);
  if (pos_att == nullptr || pos_att->size() == 0) return false;
  const PointAttribute *const norm_att =
      in_mesh_->GetNamedAttribute(GeometryAttribute::NORMAL);
  bool has_norm = true;
  if (norm_att == nullptr || norm_att->size() == 0) has_norm = false;
  // The 80 byte binary stl header is not specified so we use an excerpt
  // from a beautiful poem, To A Mouse by Robert Burns
  std::string header_preamble("BinarySTLFile But Mousie, thou art no thy-lane, "
                              "In proving foresight may be vain. The best laid"
                              " schemes o' Mice an' Men Gang aft agley,");
  buffer()->Encode(header_preamble.c_str(), 80);
  if (in_mesh_->num_faces() > std::numeric_limits<uint32_t>::max()) return false;
  buffer()->Encode<uint32_t>(in_mesh_->num_faces());
  Vector3f tmp_value;
  std::cout << " Has norm " << has_norm << std::endl;
  for (size_t i_face = 0; i_face < in_mesh_->num_faces(); ++i_face) {
    // we assume this is running on a little endian machine
    Mesh::Face face = in_mesh_->face(FaceIndex(i_face));
    Vector3f vs[3];
    for (int i_vert = 0; i_vert < 3; ++i_vert) {
      pos_att->GetMappedValue(face[i_vert], &vs[i_vert]);
    }
    if (has_norm) {
      norm_att->GetMappedValue(face[0], &tmp_value);
      if (! EncodeFloatList(&tmp_value[0], 3)) return false;
    } else {
      Vector3f norm = CrossProduct(vs[2] - vs[1], vs[0] - vs[1]);
      norm.Normalize();
      if (! EncodeFloatList(&norm[0], 3)) return false;
    }
    for (int i_vert = 0; i_vert < 3; ++i_vert) {
      if (! EncodeFloatList(&(vs[i_vert][0]), 3)) return false;
    }
    uint16_t blank = 0;
    if (! buffer()->Encode(&blank, 2)) return false;
  }
  return true;
}

}  // namespace draco
