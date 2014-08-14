#ifndef __MMD_VMD_ANIMATION_H__
#define __MMD_VMD_ANIMATION_H__

#include <vector>
#include <string>

//
// VMD format:
// http://blog.goo.ne.jp/torisu_tetosuki/e/bc9f1c4d597341b394bd02b64597499d
//
//

namespace mmd {

class PMDModel;
class IK;

#if defined(__GNUC__)
#define ATTRIBUTE_PACKED __attribute__((packed))
#else
#define ATTRIBUTE_PACKED
#endif

#pragma pack(1)
struct VMDMotion {
  // 111 bytes
  char bone_name[15];
  unsigned int frame_no;
  float location[3];
  float rotation[4];               // quaternion
  unsigned char interpolation[64]; // [4][4][4]
} ATTRIBUTE_PACKED;

struct VMDMorph {
  // 23 bytes
  char morph_name[15];
  unsigned int frame_no;
  float weight;
} ATTRIBUTE_PACKED;

struct VMDCamera {
  // 61 bytes
  unsigned int frame_no;
  float length; // -distance
  float location[3];
  float rotation[3];               // Euler. X axis has been flipped?
  unsigned char interpolation[24]; // Unused?
  unsigned int viewing_angle;
  unsigned char perspective; // 0:on, 1:off
} ATTRIBUTE_PACKED;

struct VMDLight {
  // 28 bytes
  unsigned int frame_no;
  float rgb[3]; // [0.0~1.0]
  float location[3];
} ATTRIBUTE_PACKED;

struct VMDSelfShadow {
  unsigned int frame_no;
  unsigned char mode; // 00-02
  float distance;     // 0.1 - (dist * 0.000001)
} ATTRIBUTE_PACKED;

#pragma pack()

class VMDAnimation {
public:
  VMDAnimation() {};
  ~VMDAnimation() {};

  std::vector<VMDMotion> motions_;
  std::vector<VMDMorph> morphs_;
  std::vector<VMDCamera> camera_frames_;
  std::vector<VMDLight> light_frames_;
  std::vector<VMDSelfShadow> self_shadows_;

  std::string name_;
};

class IKSolver {
public:
  IKSolver();
  ~IKSolver();

  void Solve(PMDModel *model, IK *ik, float errToleranceSq);
};

} // namespace

#endif // __MMD_VMD_ANIMATION_H__
