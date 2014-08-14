#include <cassert>
#include <cmath>
#include <cstring>
#include <algorithm>

#include "vmd_animation.h"
#include "pmd_model.h"
#include "mmd_math.h"

//
// MMD IK computation is based on MikuMikuDroid:
// http://svn.sourceforge.jp/svnroot/mikumikudroid/trunk/MikuMikuDroid
//
// IK ccd solver:
// http://www.tmps.org/index.php?CCD-IK%20and%20Particle-IK
//

using namespace mmd;

static void SetupBoneMatrix(Bone &bone, const PMDModel *model,
                            const VMDAnimation *anim, float frame) {
  assert(0); // @todo { get motion at frame. }
  Quaternion q;
  VMDMotion motion;

  q.x = motion.rotation[0];
  q.y = motion.rotation[1];
  q.z = motion.rotation[2];
  q.w = motion.rotation[3];

  QuatToMatrix(bone.matrixTemp, q);

  if (bone.parentIndex == (unsigned short)(-1)) {
    bone.matrixTemp[12] = motion.location[0] + bone.pos[0];
    bone.matrixTemp[13] = motion.location[1] + bone.pos[1];
    bone.matrixTemp[14] = motion.location[2] + bone.pos[2];
  } else {
    const Bone &p = model->bones_.at(bone.parentIndex);
    bone.matrixTemp[12] = motion.location[0] + (bone.pos[0] - p.pos[0]);
    bone.matrixTemp[13] = motion.location[1] + (bone.pos[1] - p.pos[1]);
    bone.matrixTemp[14] = motion.location[2] + (bone.pos[2] - p.pos[2]);
  }
}

// Recurvesly update bone matrix.
static void UpdateBoneMatrix(Bone *bone, PMDModel *model) {
  if (bone->updated == false) {
    if (bone->parentIndex != (unsigned short)(-1)) {
      Bone *p = &(model->bones_.at(bone->parentIndex));
      UpdateBoneMatrix(p, model);
      MatMul(bone->matrix, p->matrix, bone->matrixTemp);
    } else {
      for (int i = 0; i < 16; i++) {
        bone->matrix[i] = bone->matrixTemp[i];
      }
    }
    bone->updated = true;
  }
}

static void GetCurrentBoneMatrix(float mat[16], Bone &bone, PMDModel *model) {
  UpdateBoneMatrix(&bone, model);
  for (int i = 0; i < 16; i++) {
    mat[i] = bone.matrix[i];
  }
}

static inline void GetCurrentBonePosition(Vector3 &v, Bone &bone,
                                          PMDModel *model) {
  UpdateBoneMatrix(&bone, model);
  v.x = bone.matrix[12];
  v.y = bone.matrix[13];
  v.z = bone.matrix[14];
}

static void ClearUpdateFlags(int rootIndex, int boneIndex, PMDModel *model) {
  Bone *bone = &model->bones_.at(boneIndex);
  while (rootIndex != boneIndex) {
    bone->updated = false;
    if (bone->parentIndex != (unsigned short)(-1)) {
      bone = &model->bones_.at(bone->parentIndex);
    } else {
      return;
    }
  }
  Bone &root = model->bones_.at(rootIndex);
  root.updated = false;
}

static void ClearUpdateFlags(std::vector<Bone> &bones) {
  for (int i = 0; i < bones.size(); i++) {
    bones[i].updated = false;
  }
}

IKSolver::IKSolver() {}

IKSolver::~IKSolver() {}

void IKSolver::Solve(PMDModel *model, IK *ik, float errToleranceSq) {
  //
  // Solve IK with CCD algorithm.
  //

  Bone &effector = model->bones_.at(ik->boneIndex);
  Bone &target = model->bones_.at(ik->targetBoneIndex);

  Vector3 localTargetPos = {0, 0, 0};
  Vector3 localEffectorPos = {0, 0, 0};

  Vector3 effectorPos;
  GetCurrentBonePosition(effectorPos, effector, model);

  for (int i = 0; i < ik->iterations; i++) {
    for (int j = 0; j < ik->chainLength; j++) {
      Bone &bone = model->bones_.at(ik->childBoneIndices[j]);

      ClearUpdateFlags(ik->childBoneIndices[j], ik->targetBoneIndex, model);

      Vector3 targetPos;
      GetCurrentBonePosition(targetPos, target, model);

      if (bone.isLeg) {

        if (i == 0) {
          Bone &base =
              model->bones_.at(ik->childBoneIndices[ik->chainLength - 1]);
          GetCurrentBonePosition(localTargetPos, bone, model);
          GetCurrentBonePosition(localEffectorPos, base, model);

          Vector3 effectorVec;
          Vector3 boneVec;
          Vector3 targetVec;
          VSub(effectorVec, effectorPos, localEffectorPos);
          VSub(boneVec, localTargetPos, localEffectorPos);
          VSub(targetVec, targetPos, localTargetPos);
          float el = VLength(effectorVec);
          float bl = VLength(boneVec);
          float tl = VLength(targetVec);
          float c = (el * el - bl * bl - tl * tl) / (2.0f * bl * tl);
          if (c < -1.0)
            c = -1.0;
          if (c > 1.0)
            c = 1.0;
          float angle = acos(c);

          Vector3 axis;
          axis.x = -1.0f;
          axis.y = 0.0f;
          axis.z = 0.0f;

          Quaternion q, qa, qb;
          AxisToQuat(qa, axis, angle);
          qb.x = bone.rotation[0];
          qb.y = bone.rotation[1];
          qb.z = bone.rotation[2];
          qb.w = bone.rotation[3];
          QMult(q, qb, qa);
          bone.rotation[0] = q.x;
          bone.rotation[1] = q.y;
          bone.rotation[2] = q.z;
          bone.rotation[3] = q.w;

          // Preserve translation
          float m[16];
          QuatToMatrix(m, q);
          for (int k = 0; k < 3; k++) {
            bone.matrixTemp[4 * k + 0] = m[4 * k + 0];
            bone.matrixTemp[4 * k + 1] = m[4 * k + 1];
            bone.matrixTemp[4 * k + 2] = m[4 * k + 2];
            bone.matrixTemp[4 * k + 3] = m[4 * k + 3];
          }
          bone.matrixTemp[15] = 1.0f; // for safety.
        }

      } else {

        Vector3 d;
        VSub(d, effectorPos, targetPos);
        float diffSq = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        if (diffSq < errToleranceSq) {
          // converged.
          ClearUpdateFlags(model->bones_);
          return;
        }

        // world -> local
        float invM[16];
        GetCurrentBoneMatrix(invM, bone, model);
        M44Invert(invM);

        MatVMul(localEffectorPos, invM, effectorPos);
        MatVMul(localTargetPos, invM, targetPos);

        // basis -> effector
        Vector3 basis2Effector = localEffectorPos;
        VNormalize(basis2Effector);

        // basis -> target
        Vector3 basis2Target = localTargetPos;
        VNormalize(basis2Target);

        // Calculate shortest rotation angle.
        float rotationDotProduct = VDot(basis2Effector, basis2Target);

        if (rotationDotProduct < -1.0f) {
          rotationDotProduct = -1.0f;
        }

        if (rotationDotProduct > 1.0f) {
          rotationDotProduct = 1.0f;
        }

        float rotationAngle = (float)acos(rotationDotProduct);
        rotationAngle *= ik->weight;
        // if (rotationAngle > 1.0e-5f) {
        {
          Vector3 rotationAxis;
          VCross(rotationAxis, basis2Target, basis2Effector);
          VNormalize(rotationAxis);

          Quaternion q0;
          AxisToQuat(q0, rotationAxis, rotationAngle);

          Quaternion q1;
          q1.x = bone.rotation[0];
          q1.y = bone.rotation[1];
          q1.z = bone.rotation[2];
          q1.w = bone.rotation[3];
          Quaternion qq;
          QMult(qq, q1, q0);
          bone.rotation[0] = qq.x;
          bone.rotation[1] = qq.y;
          bone.rotation[2] = qq.z;
          bone.rotation[3] = qq.w;

          // Preserve translation
          float m[16];
          QuatToMatrix(m, qq);
          for (int k = 0; k < 3; k++) {
            bone.matrixTemp[4 * k + 0] = m[4 * k + 0];
            bone.matrixTemp[4 * k + 1] = m[4 * k + 1];
            bone.matrixTemp[4 * k + 2] = m[4 * k + 2];
            bone.matrixTemp[4 * k + 3] = m[4 * k + 3];
          }
          bone.matrixTemp[15] = 1.0f; // for safety.
        }
      }
    }
  }

  ClearUpdateFlags(model->bones_);
}
