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
#include <sstream>

#include "draco/core/draco_test_base.h"
#include "draco/core/draco_test_utils.h"
#include "draco/io/stl_decoder.h"

namespace draco {

class StlDecoderTest : public ::testing::Test {
 protected:
  template <class Geometry>
  std::unique_ptr<Geometry> DecodeStl(const std::string &file_name) const {
    const std::string path = GetTestFileFullPath(file_name);
    StlDecoder decoder;
    std::unique_ptr<Geometry> geometry(new Geometry());
    if (!decoder.DecodeFromFile(path, geometry.get()).ok())
      return nullptr;
    return geometry;
  }
  void test_decoding(const std::string &file_name) {
    const std::unique_ptr<Mesh> mesh(DecodeStl<Mesh>(file_name));
    ASSERT_NE(mesh, nullptr) << "Failed to load test model " << file_name;
    ASSERT_GT(mesh->num_faces(), 0);
  }
};

TEST_F(StlDecoderTest, ValidBinarySTL) {
  const std::string file_name = "teapot.stl";
  test_decoding(file_name);
}

TEST_F(StlDecoderTest, ValidAsciiSTL) {
  const std::string file_name = "sphere.stl";
  test_decoding(file_name);
}

}  // namespace draco
