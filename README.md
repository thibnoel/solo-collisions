# Solo Collisions

## Leg to leg collision detection
Approach for the legs : approximate them as capsules, and generate a quicker, C version of some pinocchio functions to check the distance between legs segments.
### Try it

```
mkdir build
cd build
cmake ..
make
./src/main_codegen
```
**Expected output** : 
The main currently generates source code for 2 functions : the relative placement of 2 frames in a given configuration; and the minimal distance between 2 given segments. More precisely, the outputs are as follows :
- console : the generated C source code is printed to the console for both functions
- libraries : the generated code is compiled and 2 libraries are created on the fly, under `build/libCGssd.so` and `build/libCGrel_placement.so`

## Base to upper leg collision detection 
Approach for the shoulder : first, map the collision space of the shoulder in terms of shoulder rotations at high angular resolutions with FCL using mesh colliders, then derive a low-computation model/approximation that can be embarked.
