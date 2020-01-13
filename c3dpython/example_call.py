

'''
#To build draco just recite these magic incantations:

git clone https://github.com/carbon3d/draco.git
cd draco
git checkout embind_bindings
mkdir build
cd build
cmake ..
make
'''

import subprocess

def encode_file(input_file_name, output_file_name,
                PATH_TO_DRACO_ENCODER_BINARY = '../build/draco_encoder',
                suppress_print_out = False):
    assert output_file_name.lower().split('.')[-1] == 'drc'
    if suppress_print_out:
        stdin = subprocess.DEVNULL
        stdout = subprocess.DEVNULL
    else:
        stdin = None
        stdout = None
    subprocess.check_call([PATH_TO_DRACO_ENCODER_BINARY, '-i', input_file_name, '-o', output_file_name,
                           '--use_carbon_quantization'],
                          stdin=stdin, stdout=stdout, stderr=None, shell=False)
if __name__ == '__main__':
    import argparse

    # change this
    input_file = '/home/nharrington/CompressionStlFiles/minifig.stl'

    #the only important thing is that the file name ends with drc after encoding.
    output_file = 'thingy.drc'
    encode_file(input_file_name = input_file, output_file_name = output_file)
