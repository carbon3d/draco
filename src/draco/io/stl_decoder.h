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
#ifndef DRACO_IO_STL_DECODER_H_
#define DRACO_IO_STL_DECODER_H_

#include "draco/core/decoder_buffer.h"
#include "draco/core/vector_d.h"
#include "draco/mesh/mesh.h"
#include "draco/mesh/triangle_soup_mesh_builder.h"

namespace draco {

// Decodes 3D Systems stl file format.  Decodes either the ascii or binary stl file.
// Only normals and vertex locations are decoded. STL extensions are not supported.
class StlDecoder {
 public:
  StlDecoder();
  // Decodes an st file stored in the input file.
  // Returns nullptr if the decoding failed.
  Status DecodeFromFile(const std::string &file_name, Mesh *out_mesh);
  Status DecodeFromBuffer(DecoderBuffer *buffer, Mesh *out_mesh);
 protected:
  DecoderBuffer *buffer() { return &buffer_; }
  Status ParseHeader(bool force_binary, bool* is_binary);
  Status ParseAsciiFace(Vector3f* v0, Vector3f* v1, Vector3f* v2,
                        Vector3f* normal, bool* is_valid_triangle);
  Status ParseBinaryFace(Vector3f* v0, Vector3f* v1, Vector3f* v2,
                         Vector3f* normal);
  Status DecodeInternal();
 private:
  uint32_t num_stl_faces_;
  DecoderBuffer buffer_;
  std::vector<int8_t> attribute_element_types_;
  // Data structure that stores the decoded data. |out_mesh_| must be set
  Mesh *out_mesh_;
};

}  // namespace draco

#endif  // DRACO_IO_STL_DECODER_H_
