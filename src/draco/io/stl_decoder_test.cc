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
#include "draco/core/vector_d.h"

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
  void test_decoding(const std::string &file_name, int expected_num_faces) {
    const std::unique_ptr<Mesh> mesh(DecodeStl<Mesh>(file_name));
    ASSERT_NE(mesh, nullptr) << "Failed to load test model " << file_name;
    ASSERT_EQ(mesh->num_faces(), expected_num_faces);
  }
  void check_failure_to_decode(const std::string &file_name) {
    const std::unique_ptr<Mesh> mesh(DecodeStl<Mesh>(file_name));
    ASSERT_FALSE(mesh);
  }
  void test_decoding_with_points(const std::string &file_name,
                                 int expected_num_faces,
                                 const std::vector<Vector3f>& verts) {
    const float allowed_error = 1e-3;
    const std::unique_ptr<Mesh> mesh(DecodeStl<Mesh>(file_name));
    ASSERT_NE(mesh, nullptr) << "Failed to load test model " << file_name;
    ASSERT_EQ(mesh->num_faces(), expected_num_faces);
    const PointAttribute* pos_att = mesh->GetNamedAttribute(GeometryAttribute::POSITION);
    ASSERT_TRUE(pos_att->IsValid());
    for (const Vector3f vert : verts) {
      bool found_vert = false;
      for (AttributeValueIndex i(0); i < static_cast<uint32_t>(pos_att->size()); ++i) {
        Vector3f tmp_v;
        pos_att->GetValue(i, &tmp_v);
        tmp_v = tmp_v - vert;
        if ((fabs(tmp_v[0]) < allowed_error) &&
            (fabs(tmp_v[1]) < allowed_error) &&
            (fabs(tmp_v[2]) < allowed_error)) {
          found_vert = true;
          break;
        }
      }
      ASSERT_TRUE(found_vert);
    }
  }
};

TEST_F(StlDecoderTest, ValidBinarySTL) {
  test_decoding("teapot.stl", 946);
  test_decoding("40mmcube.stl", 12);
  test_decoding("square_15_15_250.STL", 12);
  test_decoding("jczs_evil_mesh.stl", 12);
  
  std::vector<Vector3f> expected_verts;
  for (float i_z = 0; i_z < 2; ++i_z)
    for (float i_y = 0; i_y < 2; ++i_y)
      for (float i_x = 0; i_x < 2; ++i_x)
        expected_verts.push_back({40.0f * (i_x - 0.5f), 40.0f * (i_y - 0.5f), 40.0f * i_z});
  test_decoding_with_points("40mmcube.stl", 12, expected_verts);
  
}

TEST_F(StlDecoderTest, ValidAsciiSTL) {
  test_decoding("ascii_sphere.stl", 228);
  test_decoding("ascii_block100.stl", 12);
  test_decoding("ascii_noname.stl", 12);
  test_decoding("ascii_multisolid.stl", 24);  
}

TEST_F(StlDecoderTest, InfinityBinarySTL) {
  const std::string file_name = "invalid/inf_teapot.stl";
  test_decoding(file_name, 946);
}

TEST_F(StlDecoderTest, NaNBinarySTL) {
  const std::string file_name = "invalid/nan_teapot.stl";
  test_decoding(file_name, 946);
}

TEST_F(StlDecoderTest, ImproperNumberBinarySTL) {
  const std::string file_name = "invalid/tri_num_too_small_teapot.stl";
  test_decoding(file_name, 926);
  check_failure_to_decode("invalid/tri_num_too_large_teapot.stl");
}

}  // namespace draco
