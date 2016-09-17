# MMDLoader

![MMDLoaderExample](https://github.com/syoyo/MMDLoader/blob/master/mmdview.gif?raw=true)

(c) model : http://mikumikudance.wikia.com/wiki/Miku_Hatsune_(Lat)
(c) motion: http://mmd.nomeu.org/thumbs/sm13622845

Simple MMD(PMD, VMD) loader in C++.
MMDLoader is written in portable manner. No dependency except for C++ STL.

## Usage

Simply copy `mmd_*`, `pmd_*` and `vmd_*` files to your project.

## Code sample in quick

```
#include "pmd_reader.h"
#include "vmd_reader.h"
#include "mmd_scene.h"
#include "mmd_math.h"

PMDModel* model = NULL;
VMDAnimation* anim = NULL;
MMDScene* scene = NULL;

char* pmdmodel = "input.pmd";
char* vmdmodel = "input.vmd";

PMDReader pmdreader;
model = pmdreader.LoadFromFile(pmdmodel);
assert(model);

VMDReader vmdreader;
anim = vmdreader.LoadFromFile(vmdmodel);
assert(anim);

MMDScene* scene = new MMDScene();
scene->SetModel(model);
scene->AttachAnimation(anim);
```

## Example

OpenGL(GLUT) example viewer is included(see viewer_main.cc).

## Features

Supported

* PMD model loading.
* VMD motion loading.
* Bone animation(IK).
* Morph.

TODOs
(Contributors welcome!)

* [ ] Physics(Bullet)
  * W.I.P.

## Compiling example

Here is the list of premake options.

    --with-glm          : Use glm
    --with-bullet       : Use Bullet physics(Set path to bullet in `premake4.lua`)
    --with-euler-camera : Use Euler camera(Adds glm dependency)

## Author

Syoyo Fujita(syoyo@lighttransport.com)

### Contributor(s)

Jerry Chen(onlyuser@gmail.com) : glm, Bullet physics, Euler camera, split screen vr

## License

3-clause BSD.
