#ifndef __MMD_SCENE_H__
#define __MMD_SCENE_H__

#include "pmd_model.h"
#include "vmd_animation.h"

namespace mmd {

class MMDScene {
public:
  MMDScene();
  ~MMDScene();

  // Update bone position(resolve IK).
  void UpdateBone(float frame, float step);

  void SetModel(PMDModel *model) { model_ = model; }

  PMDModel *GetModel() const { return model_; }

  void AttachAnimation(VMDAnimation *anim);

  Vector3 static_min;
  Vector3 static_max;
  Vector3 static_dim;
  Vector3 static_center;
  Vector3 dynamic_min;
  Vector3 dynamic_max;
  Vector3 dynamic_dim;

private:
  PMDModel *model_;
  VMDAnimation *anim_;
};
};

#endif // __MMD_SCENE_H__
