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
#ifndef DRACO_IO_STL_ENCODER_H_
#define DRACO_IO_STL_ENCODER_H_

#include "draco/core/encoder_buffer.h"
#include "draco/mesh/mesh.h"

namespace draco {

class StlEncoder{
 public:
  StlEncoder();
  bool EncodeToFile(const Mesh &mesh, const std::string &file_name);
  bool EncodeToBuffer(const Mesh &mesh, EncoderBuffer *out_buffer);
 protected:
  bool EncodeInternal();
  EncoderBuffer *buffer() const { return out_buffer_; }
  bool ExitAndCleanup(bool return_value);
  bool EncodeFloatList(float* floats, uint32_t num_floats);
 private:
  const Mesh *in_mesh_;
  const PointAttribute *pos_att_;
  const PointAttribute *normal_att_;
  std::string file_name_;
  EncoderBuffer *out_buffer_;
};

}  // namespace draco

#endif  // DRACO_IO_STL_ENCODER_H_
