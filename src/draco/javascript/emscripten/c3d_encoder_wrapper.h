#ifndef DRACO_C3D_ENCODER_WRAPPER_H_
#define DRACO_C3D_ENCODER_WRAPPER_H_

#include "draco/core/encoder_buffer.h"
#include "draco/core/decoder_buffer.h"
#include "draco/core/status.h"

namespace draco {

class DracoC3dEncoder {
 public:
  DracoC3dEncoder(int quantization_num_bits, int encoding_speed, int decoding_speed);  
  const Status* ConvertInputBufferToDracoBuffer(DecoderBuffer* input_buffer,
                                                const std::string& file_extension,
                                                EncoderBuffer* output_buffer);
 private:
  int quantization_num_bits_;
  int encoding_speed_;
  int decoding_speed_;
  Status last_status_;
};

} //  namespace draco

#endif // DRACO_C3D_ENCODER_WRAPPER_H_
