#ifndef DRACO_C3D_ENCODER_WRAPPER_H_
#define DRACO_C3D_ENCODER_WRAPPER_H_

#include "draco/encoder_buffer.h"
#include "draco/decoder_buffer.h"

namespace draco {

enum MeshFileType {
  UNKNOWN = 0,
  STL = 1,
  OBJ = 2,
  PLY = 3
};

class DracoC3dEncoder {
 public:
  DracoC3dEncoder(float quantization_scale_nm, int encoding_speed, int decoding_speed);  
  void EncodeFile(const EncoderBuffer& input_buffer, MeshFileType file_type,
                  DecoderBuffer* output_buffer);
 private:
  float quantization_scale_nm_;
  int encoding_speed_;
  int decoding_speed_;
};

} //  namespace draco

#endif DRACO_C3D_ENCODER_WRAPPER_H_
