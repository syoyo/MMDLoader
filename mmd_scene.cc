#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <map>

#include "mmd_math.h"
#include "mmd_scene.h"

using namespace mmd;

struct MotionSegment {
  int m0;
  int m1;
};

MotionSegment FindMotionSegment(Bone &bone, float frame) {
  MotionSegment ms;
  ms.m0 = 0;
  assert(0);
  return ms;
}

MMDScene::MMDScene() : model_(NULL), anim_(NULL) {}

MMDScene::~MMDScene() {}

void MMDScene::AttachAnimation(VMDAnimation *anim) {
  assert(model_);

  // Create hash map for bone <-> motion list
  typedef std::map<std::string, std::vector<Motion> > MotionMap;
  MotionMap motionMap;

  for (int i = 0; i < anim->motions_.size(); i++) {
    VMDMotion &vmdMotion = anim->motions_[i];
    // bone_name[15] might not be null-terminated.
    // So append '\0' to reconstrct a string.
    char buf[16];
    memcpy(buf, vmdMotion.bone_name, 15);
    buf[15] = '\0';
    std::string boneName(buf);

    Motion motion;

    motion.frameNo = vmdMotion.frame_no;
    motion.pos[0] = vmdMotion.location[0];
    motion.pos[1] = vmdMotion.location[1];
    motion.pos[2] = vmdMotion.location[2];
    motion.rotation[0] = vmdMotion.rotation[0];
    motion.rotation[1] = vmdMotion.rotation[1];
    motion.rotation[2] = vmdMotion.rotation[2];
    motion.rotation[3] = vmdMotion.rotation[3];

    // http://harigane.at.webry.info/201103/article_1.html
    for (int k = 0; k < 4; k++) {
      motion.interpX[k] = vmdMotion.interpolation[k * 4 + 0];
      motion.interpY[k] = vmdMotion.interpolation[k * 4 + 1];
      motion.interpZ[k] = vmdMotion.interpolation[k * 4 + 2];
      motion.interpR[k] = vmdMotion.interpolation[k * 4 + 3];
    }

    motionMap[boneName].push_back(motion);
  }

  // Assign motion list to a bone.
  for (int i = 0; i < model_->bones_.size(); i++) {
    Bone &bone = model_->bones_[i];
    bone.motions.clear();
    MotionMap::iterator it = motionMap.find(bone.name);
    if (it != motionMap.end()) {
      bone.motions = it->second;
    } else {
      printf("[MMD] Cannot find bone [ %s ] in PMD.\n", bone.name.c_str());
    }
  }

  anim_ = anim; // save for a reference.
}

void MMDScene::UpdateBone(float frame, float step) {
  //printf("UpdateBone()\n");
  if (model_->bones_.empty()) {
    return;
  }

  assert(0);
}
