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
#include <cmath>
#include <random>
#include <sstream>
#include <vector>

#include "draco/compression/mesh/mesh_quantization_carbon.h"

#include "draco/compression/encode.h"
#include "draco/compression/mesh/mesh_edgebreaker_decoder.h"
#include "draco/compression/mesh/mesh_edgebreaker_encoder.h"
#include "draco/core/draco_test_base.h"
#include "draco/core/draco_test_utils.h"
#include "draco/io/mesh_io.h"
#include "draco/io/obj_decoder.h"
#include "draco/mesh/mesh_are_equivalent.h"
#include "draco/mesh/mesh_cleanup.h"
#include "draco/mesh/triangle_soup_mesh_builder.h"

#include <iostream>

namespace draco {

class MeshQuantizationCarbonTest : public ::testing::Test {
 protected:
  void TestTriangleQuantization(const std::vector<float>& in_vert_locs,
                                      float grid_spacing, bool expect_exact) {
    ASSERT_TRUE(in_vert_locs.size() % 9 == 0);
    // create our mesh
    TriangleSoupMeshBuilder mesh_builder;
    mesh_builder.Start(in_vert_locs.size() / 9);
    int pos_id = mesh_builder.AddAttribute(GeometryAttribute::Type::POSITION, 3, DataType::DT_FLOAT32);
    for (int i = 0; i < in_vert_locs.size(); i += 9) {
      mesh_builder.SetAttributeValuesForFace(
          pos_id, FaceIndex(i / 9), &(in_vert_locs[i]), &(in_vert_locs[i + 3]), &(in_vert_locs[i + 6]));
    }
    std::unique_ptr<Mesh> mesh = mesh_builder.Finalize();
    // set quantization parameters
    MeshQuantizationCarbon mesh_quant_params;
    ASSERT_STREQ(mesh_quant_params.FillFromMesh(mesh.get(), grid_spacing).c_str(), "");
    // Encode
    Encoder encoder;
    float origin[3] = {mesh_quant_params.min_values_x(),
                       mesh_quant_params.min_values_y(),
                       mesh_quant_params.min_values_z()};
    encoder.SetAttributeExplicitQuantization(
        GeometryAttribute::Type::POSITION,
        mesh_quant_params.quantization_bits(), 3,
        origin, mesh_quant_params.range());
    EncoderBuffer enc_buffer;
    Status status = encoder.EncodeMeshToBuffer(*mesh, &enc_buffer);
    if (! status.ok()) std::cout << status.error_msg() << std::endl;
    ASSERT_TRUE(status.ok());
    // Decode
    Decoder decoder;
    DecoderBuffer dec_buffer;
    dec_buffer.Init(enc_buffer.data(), enc_buffer.size());
    Mesh mesh_round_trip;
    status = decoder.DecodeBufferToGeometry(&dec_buffer, &mesh_round_trip);
    ASSERT_TRUE(status.ok());
    // Check that vertex locations survived the trip
    const PointAttribute* pos_att = mesh->GetNamedAttribute(GeometryAttribute::POSITION);
    ASSERT_TRUE(pos_att->IsValid());
    std::array<float, 3> vert_loc;
    for (int i_vert = 0; i_vert < in_vert_locs.size() / 3; ++i_vert) {
      for (int i_dim = 0; i_dim < 3; ++i_dim) {
        vert_loc = pos_att->GetValue<float, 3>(AttributeValueIndex(i_vert));
        if (expect_exact) ASSERT_EQ(vert_loc[i_dim], in_vert_locs[i_vert*3 + i_dim]);
        else ASSERT_LE(fabs(vert_loc[i_dim] - in_vert_locs[i_vert*3 + i_dim]), grid_spacing / 2.0);
      }
    }
  }
};

TEST_F(MeshQuantizationCarbonTest, TestExplicitParameters) {
  // create our mesh
  TriangleSoupMeshBuilder mesh_builder;
  mesh_builder.Start(1);
  int pos_id = mesh_builder.AddAttribute(GeometryAttribute::Type::POSITION, 3, DataType::DT_FLOAT32);
  float vs[9] = {0, 0, -1,
                 0, 2, -1,
                 1, 0, -1};
  mesh_builder.SetAttributeValuesForFace(pos_id, FaceIndex(0), vs, vs + 3, vs + 6);
  std::unique_ptr<Mesh> mesh = mesh_builder.Finalize();
  // set quantization parameters
  MeshQuantizationCarbon mesh_quant_params;
  ASSERT_STREQ(mesh_quant_params.FillFromMesh(mesh.get(), 1.0).c_str(), "");
  ASSERT_EQ(mesh_quant_params.quantization_bits(), 2);
  ASSERT_EQ(mesh_quant_params.min_values_x(), 0);
  ASSERT_EQ(mesh_quant_params.min_values_y(), 0);
  ASSERT_EQ(mesh_quant_params.min_values_z(), -1);
}

TEST_F(MeshQuantizationCarbonTest, TestExactQuantization) {
  std::vector<float> in_vert_locs = {0, 0, -1,
                                     0, 2, -1,
                                     1, 0, -1};
  TestTriangleQuantization(in_vert_locs, 1.0, true);
  int seed = 373;
  std::mt19937 gen(seed);
  constexpr int kNumTests = 100;
  std::uniform_int_distribution<> num_triangle_gen(1, 3000);
  std::uniform_int_distribution<> vert_gen(-33554, 33554);
  std::uniform_real_distribution<> spacing_gen(1e-6, 1);
  std::uniform_real_distribution<> offset_gen(1e-6, 1);
  for (int i = 0; i < kNumTests; ++i) {
    int num_tris = num_triangle_gen(gen);
    float grid_spacing = spacing_gen(gen);
    float offsets[3];
    for (int j = 0; j < 3; ++j) offsets[j] = offset_gen(gen);
    std::vector<float> triangles(num_tris * 9);
    for (int i_vert = 0; i_vert < triangles.size(); ++i_vert) {
      triangles[i_vert] = grid_spacing * vert_gen(gen) + offsets[i_vert % 3];
    }
    TestTriangleQuantization(triangles, grid_spacing, true);
  }
}


TEST_F(MeshQuantizationCarbonTest, TestInexactQuantization) {
  int seed = 372;
  std::mt19937 gen(seed);
  constexpr int kNumTests = 100;
  std::uniform_int_distribution<> num_triangle_gen(1, 3000);
  std::uniform_real_distribution<> vert_gen(-33, 33);
  std::uniform_real_distribution<> spacing_gen(0.1, 1);
  std::uniform_real_distribution<> offset_gen(1e-6, 1);
  for (int i = 0; i < kNumTests; ++i) {
    int num_tris = num_triangle_gen(gen);
    float grid_spacing = spacing_gen(gen);
    float offsets[3];
    for (int j = 0; j < 3; ++j) offsets[j] = offset_gen(gen);
    std::vector<float> triangles(num_tris * 9);
    for (int i_vert = 0; i_vert < triangles.size(); ++i_vert) {
      triangles[i_vert] = vert_gen(gen) + offsets[i_vert % 3];
    }
    TestTriangleQuantization(triangles, grid_spacing, false);
  }
}

}  // namespace draco
