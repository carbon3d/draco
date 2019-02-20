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
#include <fstream>
#include <sstream>

#include "draco/core/draco_test_base.h"
#include "draco/core/draco_test_utils.h"
#include "draco/io/stl_decoder.h"
#include "draco/io/stl_encoder.h"

namespace draco {

class StlEncoderTest : public ::testing::Test {
 protected:
  void CompareMeshes(const Mesh *mesh0, const Mesh *mesh1) {
    ASSERT_EQ(mesh0->num_faces(), mesh1->num_faces());
    ASSERT_EQ(mesh0->num_attributes(), mesh1->num_attributes());
    for (size_t att_id = 0; att_id < mesh0->num_attributes(); ++att_id) {
      ASSERT_EQ(mesh0->attribute(att_id)->size(),
                mesh1->attribute(att_id)->size());
    }
  }

  // Encode a mesh using the StlEncoder and then decode to verify the encoding.
  std::unique_ptr<Mesh> EncodeAndDecodeMesh(const Mesh *mesh) {
    EncoderBuffer encoder_buffer;
    StlEncoder encoder;
    if (!encoder.EncodeToBuffer(*mesh, &encoder_buffer))
      return nullptr;

    DecoderBuffer decoder_buffer;
    decoder_buffer.Init(encoder_buffer.data(), encoder_buffer.size());
    std::unique_ptr<Mesh> decoded_mesh(new Mesh());
    StlDecoder decoder;
    if (!decoder.DecodeFromBuffer(&decoder_buffer, decoded_mesh.get()).ok())
      return nullptr;
    return decoded_mesh;
  }

  void test_encoding(const std::string &file_name) {
    const std::unique_ptr<Mesh> mesh(ReadMeshFromTestFile(file_name, true));

    ASSERT_NE(mesh, nullptr) << "Failed to load test model " << file_name;
    ASSERT_GT(mesh->num_faces(), 0);

    const std::unique_ptr<Mesh> decoded_mesh = EncodeAndDecodeMesh(mesh.get());
    CompareMeshes(mesh.get(), decoded_mesh.get());
  }
};

TEST_F(StlEncoderTest, TestStlEncodingAll) {
  // Test decoded mesh from encoded stl file stays the same.
  test_encoding("teapot.stl");
  test_encoding("40mmcube.stl");
  test_encoding("ascii_sphere.stl");
  test_encoding("ascii_block100.stl");
  test_encoding("ascii_facet_name.stl");
  test_encoding("ascii_noname.stl");
}

}  // namespace draco
