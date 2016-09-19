#ifndef __MMD_PMD_MODEL_H__
#define __MMD_PMD_MODEL_H__

#include <vector>
#include <string>
#include <mmd_math.h>

//
// PMD format:
// http://blog.goo.ne.jp/torisu_tetosuki/e/209ad341d3ece2b1b4df24abf619d6e4
//
// textencoding: bytes(cp932)
// coordinate: left handed y-up(DirectX)
//

namespace mmd {

#if defined(__GNUC__)
#define ATTRIBUTE_PACKED __attribute__((packed))
#else
#define ATTRIBUTE_PACKED
#endif

#pragma pack(1)
struct PMDVertex {
  // 38 bytes
  float pos[3];           // vertex
  float normal[3];        // normal
  float uv[2];            // uv
  unsigned short bone[2]; // bone1, bone2
  unsigned char weight;   // [0,100]. bone1: weight, bone2: (100-weight);
  unsigned char edge;     // 0: on, 1: off
} ATTRIBUTE_PACKED;

struct PMDMaterial {
  // 70 bytes
  float diffuse[3];
  float alpha;
  float specularity;
  float specular[3];
  float ambient[3];
  unsigned char toon_index; // toon??.bmp // 0.bmp:0xFF, 1(01).bmp:0x00 ...
                            // 10.bmp:0x09
  unsigned char edge_flag;  // contour, shadow
  unsigned int vertex_count;
  char texture_filename[20];
} ATTRIBUTE_PACKED;

struct PMDBone {
  // 39 bytes
  char bone_name[20];
  unsigned short parent_bone_index; // root=0xFFFF
  unsigned short tail_bone_index;   // tail=0xFFFF
  unsigned char bone_type; // 0:rot, 1:rot+trans, 2:IK, 3:?, 4:AffectedbyIK,
                           // 5:AfftectedByRot, 6:IKTarget, 7:Invisible
                           // ver4.0~ 8:twist, 9:rot
  unsigned short ik_parent_bone_index; // NoIK=0
  float bone_pos[3];
} ATTRIBUTE_PACKED;

struct PMDIK {
  // 11 + 2 * chain_length bytes
  unsigned short bone_index;
  unsigned short target_bone_index;
  unsigned char chain_length;
  unsigned short iterations;
  float weight;
  // unsigned short  child_bone_indices[]; // len=chain_length
} ATTRIBUTE_PACKED;

struct PMDMorphVertex {
  unsigned int vertex_index;
  float pos[3];
} ATTRIBUTE_PACKED;

struct PMDMorph {
  char name[20];
  unsigned int vertex_count;
  unsigned char type;
  // PMDMorphVertex  vertices[];   // len = vertex_count;
} ATTRIBUTE_PACKED;

#pragma pack()

// @todo { morph_indices, bone_group_list, bone_display_list }

struct Motion {
  int frameNo;
  float pos[3];
  float rotation[4]; // quaternion
  unsigned char interpX[4];
  unsigned char interpY[4];
  unsigned char interpZ[4];
  unsigned char interpR[4];
};

struct Bone {
  Bone() : isLeg(0), updated(0) {
    motionOffsetPos[0] = 0.0f;
    motionOffsetPos[1] = 0.0f;
    motionOffsetPos[2] = 0.0f;
  }

  ~Bone() {};

  std::string name;
  std::string ascii_name;
  unsigned short parentIndex;
  unsigned short tailIndex;
  unsigned char type;
  unsigned short parentIndexIK;

  float pos[4];       // w = 1
  double rotation[4]; // quaternion

  float matrix[16];
  float matrixTemp[16]; // temporal

  std::vector<Motion> motions;

  float motionOffsetPos[3];

  bool hasVertices;
  bool isLeg;
  bool isChain;
  bool isPinnedChain;
  bool updated;

  Vector3 min;
  Vector3 max;
  Vector3 dim;

  void* bulletDynamicObject;
};

struct IK {
  unsigned short boneIndex;
  unsigned short targetBoneIndex;
  unsigned char chainLength;
  unsigned short iterations;
  float weight;
  std::vector<unsigned short> childBoneIndices;
};

struct Morph {
  std::string name;
  unsigned int vertexCount;
  unsigned char type;
  std::vector<PMDMorphVertex> vertices;
};

class PMDModel {
public:
  PMDModel() {};
  ~PMDModel() {};

  std::vector<PMDVertex> vertices_;
  std::vector<unsigned short> indices_;
  std::vector<PMDMaterial> materials_;
  std::vector<Bone> bones_;
  std::vector<IK> iks_;
  std::vector<Morph> morphs_;

  std::string name_;
  std::string comment_;
  float version_;
};

} // namespace

#endif // __MMD_PMD_MODEL_H__
