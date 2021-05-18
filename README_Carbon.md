# Carbon Extensions

The standard draco bindings were fairly limitted and have a use after free error, see [the draco bindings bug report](https://github.com/google/draco/issues/513), so they have been extended to provide the needed functionality and not have this subtle bug.  The bug is fairly pernicious, so it required a bit of a rewrite to make work and a slight changing of the draco API.


## Building the code
Draco uses the cmake build system.  To compile the webassembly code it uses the [emscripten library](https://emscripten.org).

To get started, install Emscripten on your system. Just follow the instructions at the [emscripten downloads page.](https://emscripten.org/docs/getting_started/downloads.html)

After you have installed Emscripten you will need to setup your environment.  The emsdk repository provides a simple file for doing that called emsdk_env.sh

```bash
source /PATH/TO/emsdk/emsdk_env.sh
```

You will also need to set your environment variable EMSCRIPTEN to be the directory that contains all the binaries for emscripten to run.
```
export EMSCRIPTEN=/PATH/TO/emsdk/upstream/emscripten/
```


Once that is done, to build the webassembly type:

```bash
cd /path/to/draco
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=/PATH/TO/YOUR/emsdk/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake -DENABLE_WASM=ON -DENABLE_EMBIND=ON ..
make
```

You will need to find the right path the the Emscripten.cmake which depends on where you installed emscripten.  Also, remember that this will only work if you have setup the environment appropriately.  Hopefully at this point you have compiled code.  In the build directory there should be a number of biproducts from the build and the two files, DracoEncoderEmbind.js  DracoEncoderEmbind.wasm.  If you have those, you are doing great.  We copy those files into c3d/js/src/common/draco/ to make the javascript availble to us.


********WARNING***********

You may need to manually modify the header of the generated js file with the diff:
-  var _scriptDir = import.meta.url;
-  
+  var _scriptDir = typeof document !== 'undefined' && document.currentScript ? document.currentScript.src : undefined; 




## Example Code

There is a (terrible) javascript example at: draco/CarbonDracoExample.html

Running this script requires getting around some cross-origin requests.  Chrome/firefox complain when attempting to load webassembly from local sources.  To get around that, you can run [live-server](https://www.npmjs.com/package/live-server) which is a simple program that presents your local filesystem as a webserver.

The draco javascript bindings have been rewritten using [embind](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html) so our initialization of the webassembly is a bit different.

The following code initializes the draco webassembly module.

```javascript
// The Module object provides a set of callbacks to the embind based webassembly
// initialization.  We are only using the onRuntimeInitialized callback which
// is called after the module is initialized.
var Module = {
    onRuntimeInitialized: function() {
      console.log("Runtime initialization");
      document.getElementById('encodeOne').disabled = false;
    }
};
console.log("Creating our encoder module");
// This creates our webassembly module.  Our C++ code exists as members and
// methods of this object
var OurEncoderModule = DracoEncoderEmbind(Module);
```

To actually use the web assembly you first need to create the objects with methods we want.
```javascript
let decoder_buffer_owner = new OurEncoderModule.DecoderBufferOwner( 
    document.getElementById("fileItem").files[0].size);
let mesh = new OurEncoderModule.Mesh(); 
let encoder = new OurEncoderModule.Encoder();
let mesh_quantization = new OurEncoderModule.MeshQuantizationCarbon();
let output_buffer = new OurEncoderModule.DracoInt8Array();
```

Because javascript does not have a finalizer/destructor for these objects, when you are done using them  you need to manually delete them.

```
decoder_buffer_owner.delete();
mesh.delete();
encoder.delete();
mesh_quantization.delete();
output_buffer.delete();
```

If you forget to manually delete them, congratulations, you have a memory leak!

Because our bindings were written using embind, the initialization of the module and the destruction of our constructed objects have slightly different syntax than the standard Draco javascript bindings.

We've also added a few extra classes and functions to extend the functionality of Draco.

* DecoderBufferOwner

  DecoderBufferOwner owns a block of memory on the heap.  The constructor for it accepts the number of bytes it owns as its sole argument.  It has two methods, GetDecoderBuffer which returns a DecoderBuffer (which is documented in the draco readme) and GetBufferView() which returns a typed array view of the memory owned by the DecoderBufferOwner.  This memory can be modified in javascript and is available to the compiled C++ code.

* DecodeFileBufferToMesh(DecoderBuffer buffer, string file_extension, Mesh mesh)

  DecodeFileBufferToMesh is a function that takes an stl, obj or ply file loaded into memory and converts it into a triangle mesh.  buffer is a decoder buffer that stores the contents of the file.  The file extension is one of 'stl', 'ply' or 'obj' specifying the type of the file.  The mesh represented in the file is loaded into the mesh.

* MeshQuantizationCarbon

  MeshQuantizationCarbon is a class for calculating our desired quantization of the mesh.  It has several getter methods and one setter method, FillFromMesh(Mesh mesh, float quantization_scale_mm), which accepts a Mesh object and our desired quantization scale in mm (the spacing of the gridding when quantizing vertex locations) and fills in the quantization parameters used by draco.

