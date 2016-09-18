#include <cassert>
#include <cstring> // memcpy
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <iostream>

#include "pmd_reader.h"

#define MAX_BUF_LEN 20

struct sjis_table_rec_t
{
  char unicode_name[MAX_BUF_LEN];
  char ascii_name[MAX_BUF_LEN];
  char sjis_name[MAX_BUF_LEN];
};

using namespace mmd;

// http://akemiwhy.deviantart.com/art/mmd-reference-japanese-bone-names-430962605
// http://ash.jp/code/unitbl21.htm
sjis_table_rec_t sjis_table[] = {
    {"グルーブ", "groove", {0x83, 0x4F, 0x83, 0x8B, 0x81, 0x5B, 0x83, 0x75, 0x00}},
    {"センター", "center", {0x83, 0x5A, 0x83, 0x93, 0x83, 0x5E, 0x81, 0x5B, 0x00}},
    {"上半身", "upper_body", {0x8F, 0xE3, 0x94, 0xBC, 0x90, 0x67, 0x00}},
    {"下半身", "lower_body", {0x89, 0xBA, 0x94, 0xBC, 0x90, 0x67, 0x00}},
    {"両目", "eyes", {0x97, 0xBC, 0x96, 0xDA, 0x00}},
    {"全ての親", "mother", {0x91, 0x53, 0x82, 0xC4, 0x82, 0xCC, 0x90, 0x65, 0x00}},
    {"右つま先ＩＫ", "toe_IK_R", {0x89, 0x45, 0x82, 0xC2, 0x82, 0xDC, 0x90, 0xE6, 0x82, 0x68, 0x82, 0x6A, 0x00}},
    {"右ひざ", "knee_R", {0x89, 0x45, 0x82, 0xD0, 0x82, 0xB4, 0x00}},
    {"右ひじ", "elbow_R", {0x89, 0x45, 0x82, 0xD0, 0x82, 0xB6, 0x00}},
    {"右中指１", "middle1_R", {0x89, 0x45, 0x92, 0x86, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"右中指２", "middle2_R", {0x89, 0x45, 0x92, 0x86, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"右中指３", "middle3_R", {0x89, 0x45, 0x92, 0x86, 0x8E, 0x77, 0x82, 0x52, 0x00}},
    {"右人指１", "fore1_R", {0x89, 0x45, 0x90, 0x6C, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"右人指２", "fore2_R", {0x89, 0x45, 0x90, 0x6C, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"右人指３", "fore3_R", {0x89, 0x45, 0x90, 0x6C, 0x8E, 0x77, 0x82, 0x52, 0x00}},
    {"右小指１", "little1_R", {0x89, 0x45, 0x8F, 0xAC, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"右小指２", "little2_R", {0x89, 0x45, 0x8F, 0xAC, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"右小指３", "little3_R", {0x89, 0x45, 0x8F, 0xAC, 0x8E, 0x77, 0x82, 0x52, 0x00}},
    {"右手首", "wrist_R", {0x89, 0x45, 0x8E, 0xE8, 0x8E, 0xF1, 0x00}},
    {"右目", "eye_R", {0x89, 0x45, 0x96, 0xDA, 0x00}},
    {"右肩", "shoulder_R", {0x89, 0x45, 0x8C, 0xA8, 0x00}},
    {"右腕", "arm_R", {0x89, 0x45, 0x98, 0x72, 0x00}},
    {"右薬指１", "third1_R", {0x89, 0x45, 0x96, 0xF2, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"右薬指２", "third2_R", {0x89, 0x45, 0x96, 0xF2, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"右薬指３", "third3_R", {0x89, 0x45, 0x96, 0xF2, 0x8E, 0x77, 0x82, 0x52, 0x00}},
    {"右袖", "sleeve_R", {0x89, 0x45, 0x91, 0xb3, 0x00}},
    {"右袖先", "cuff_R", {0x89, 0x45, 0x91, 0xb3, 0x90, 0xe6, 0x00}},
    {"右親指１", "thumb1_R", {0x89, 0x45, 0x90, 0x65, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"右親指２", "thumb2_R", {0x89, 0x45, 0x90, 0x65, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"右足", "leg_R", {0x89, 0x45, 0x91, 0xAB, 0x00}},
    {"右足首", "ankle_R", {0x89, 0x45, 0x91, 0xAB, 0x8E, 0xF1, 0x00}},
    {"右足ＩＫ", "leg_IK_R", {0x89, 0x45, 0x91, 0xAB, 0x82, 0x68, 0x82, 0x6A, 0x00}},
    {"左つま先ＩＫ", "toe_IK_L", {0x8D, 0xB6, 0x82, 0xC2, 0x82, 0xDC, 0x90, 0xE6, 0x82, 0x68, 0x82, 0x6A, 0x00}},
    {"左ひざ", "knee_L", {0x8D, 0xB6, 0x82, 0xD0, 0x82, 0xB4, 0x00}},
    {"左ひじ", "elbow_L", {0x8D, 0xB6, 0x82, 0xD0, 0x82, 0xB6, 0x00}},
    {"左中指１", "middle1_L", {0x8D, 0xB6, 0x92, 0x86, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"左中指２", "middle2_L", {0x8D, 0xB6, 0x92, 0x86, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"左中指３", "middle3_L", {0x8D, 0xB6, 0x92, 0x86, 0x8E, 0x77, 0x82, 0x52, 0x00}},
    {"左人指１", "fore1_L", {0x8D, 0xB6, 0x90, 0x6C, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"左人指２", "fore2_L", {0x8D, 0xB6, 0x90, 0x6C, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"左人指３", "fore3_L", {0x8D, 0xB6, 0x90, 0x6C, 0x8E, 0x77, 0x82, 0x52, 0x00}},
    {"左小指１", "little1_L", {0x8D, 0xB6, 0x8F, 0xAC, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"左小指２", "little2_L", {0x8D, 0xB6, 0x8F, 0xAC, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"左小指３", "little3_L", {0x8D, 0xB6, 0x8F, 0xAC, 0x8E, 0x77, 0x82, 0x52, 0x00}},
    {"左手首", "wrist_L", {0x8D, 0xB6, 0x8E, 0xE8, 0x8E, 0xF1, 0x00}},
    {"左目", "eye_L", {0x8D, 0xB6, 0x96, 0xDA, 0x00}},
    {"左肩", "shoulder_L", {0x8D, 0xB6, 0x8C, 0xA8, 0x00}},
    {"左腕", "arm_L", {0x8D, 0xB6, 0x98, 0x72, 0x00}},
    {"左薬指１", "third1_L", {0x8D, 0xB6, 0x96, 0xF2, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"左薬指２", "third2_L", {0x8D, 0xB6, 0x96, 0xF2, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"左薬指３", "third3_L", {0x8D, 0xB6, 0x96, 0xF2, 0x8E, 0x77, 0x82, 0x52, 0x00}},
    {"左袖", "sleeve_L", {0x8D, 0xB6, 0x91, 0xb3, 0x00}},
    {"左袖先", "cuff_L", {0x8D, 0xB6, 0x91, 0xb3, 0x90, 0xe6, 0x00}},
    {"左親指１", "thumb1_L", {0x8D, 0xB6, 0x90, 0x65, 0x8E, 0x77, 0x82, 0x50, 0x00}},
    {"左親指２", "thumb2_L", {0x8D, 0xB6, 0x90, 0x65, 0x8E, 0x77, 0x82, 0x51, 0x00}},
    {"左足", "leg_L", {0x8D, 0xB6, 0x91, 0xAB, 0x00}},
    {"左足首", "ankle_L", {0x8D, 0xB6, 0x91, 0xAB, 0x8E, 0xF1, 0x00}},
    {"左足ＩＫ", "leg_IK_L", {0x8D, 0xB6, 0x91, 0xAB, 0x82, 0x68, 0x82, 0x6A, 0x00}},
    {"頭", "head", {0x93, 0xAA, 0x00}},
    {"首", "neck", {0x8E, 0xF1, 0x00}},
    };

typedef std::map<std::string, sjis_table_rec_t*> sjis_map_t;
sjis_map_t sjis_map;

std::set<std::string> unvisited_bones;

//
//
//
static bool ReadBytes(unsigned char *out_data, int size, FILE *fp) {
  size_t sz;
  sz = fread(out_data, 1, size, fp);
  assert(sz == size);
  return true;
}

static bool ParseBone(PMDModel *model, FILE *fp) {
  unvisited_bones.clear();
  for(sjis_map_t::iterator p = sjis_map.begin(); p != sjis_map.end(); p++) {
    unvisited_bones.insert((*p).second->ascii_name);
  }

  // cp932 encoding of 'leg'(hi-za) in Japanese
  const char kLegName[4 + 1] = {0x82, 0xd0, 0x82, 0xb4, 0x00}; // +1 for \0

  unsigned short numBones;
  ReadBytes(reinterpret_cast<unsigned char *>(&numBones),
            sizeof(unsigned short), fp);
  printf("[PMD] Num bones: %d\n", numBones);
  assert(sizeof(PMDBone) == 39);
  std::vector<PMDBone> pmdBones(numBones);
  ReadBytes(reinterpret_cast<unsigned char *>(&(pmdBones[0])),
            sizeof(PMDBone) * numBones, fp);

  model->bones_.clear();

  // Bone with name containing 'leg' in Japanese need special treatment
  // when computing IK.
  for (int i = 0; i < numBones; i++) {
    Bone bone;

    bone.parentIndex = pmdBones[i].parent_bone_index;
    bone.tailIndex = pmdBones[i].tail_bone_index;
    bone.type = pmdBones[i].bone_type;
    bone.parentIndexIK = pmdBones[i].ik_parent_bone_index;

    // printf("[PMD] [%d] parent %d, tail %d, ty %d, ik %d\n", i,
    // pmdBones[i].parent_bone_index, pmdBones[i].tail_bone_index,
    // pmdBones[i].bone_type, pmdBones[i].ik_parent_bone_index);

    if (pmdBones[i].tail_bone_index == (unsigned short)(-1)) {
      // skip
      printf("[PMD] Bone [%d] is the tail. Skipping.\n", i);
      continue;
    }

    bone.pos[0] = pmdBones[i].bone_pos[0];
    bone.pos[1] = pmdBones[i].bone_pos[1];
    bone.pos[2] = pmdBones[i].bone_pos[2];
    bone.pos[3] = 1.0f;

    char buf[21];
    memcpy(buf, pmdBones[i].bone_name, 20);
    buf[20] = '\0'; // add '\0' for safety
    bone.name = std::string(buf);
    if (bone.name.find(kLegName) != std::string::npos) {
      printf("[PMD] Bone [%d] is leg\n", i);
      bone.isLeg = true;
    } else {
      bone.isLeg = false;
    }

    sjis_map_t::iterator p = sjis_map.find(buf);
    if (p != sjis_map.end()) {
      bone.ascii_name = (*p).second->ascii_name;
      std::set<std::string>::iterator q = unvisited_bones.find((*p).second->ascii_name);
      if(q != unvisited_bones.end()) {
        unvisited_bones.erase(q);
      }
    }

    model->bones_.push_back(bone);
  }

  for(std::set<std::string>::iterator r = unvisited_bones.begin(); r != unvisited_bones.end(); r++) {
    std::cout << "[PMD] Cannot find bone [ " << *r << " ] in PMD." << std::endl;
  }

  return true;
}

static bool ParseMorph(PMDModel *model, FILE *fp){
  unsigned short numMorphs;
  ReadBytes(reinterpret_cast<unsigned char *>(&numMorphs), sizeof(unsigned short),
            fp);
  printf("[PMD] Num Morphs: %d\n", numMorphs);           
  
  std::vector<Morph> morphs(numMorphs);
  for (int i = 0; i < numMorphs; i++) {
    PMDMorph pmdMorph;
    ReadBytes(reinterpret_cast<unsigned char *>(&pmdMorph), sizeof(PMDMorph), fp);
    std::vector<PMDMorphVertex> vertices(pmdMorph.vertex_count);
    for (int j = 0; j < pmdMorph.vertex_count; j++){
    ReadBytes(reinterpret_cast<unsigned char *>(&vertices[j]), sizeof(PMDMorphVertex), fp);
  }
  }
  return true;
}

static bool ParseIK(PMDModel *model, FILE *fp) {
  unsigned short numIKs;
  ReadBytes(reinterpret_cast<unsigned char *>(&numIKs), sizeof(unsigned short),
            fp);
  printf("[PMD] Num IKs: %d\n", numIKs);
  assert(sizeof(PMDIK) == 11);
  std::vector<IK> iks(numIKs);
  for (int i = 0; i < numIKs; i++) {
    PMDIK pmdIK;
    ReadBytes(reinterpret_cast<unsigned char *>(&pmdIK), sizeof(PMDIK), fp);

    iks[i].boneIndex = pmdIK.bone_index;
    iks[i].targetBoneIndex = pmdIK.target_bone_index;
    iks[i].chainLength = pmdIK.chain_length;
    iks[i].iterations = pmdIK.iterations;
    iks[i].weight = pmdIK.weight;

    iks[i].childBoneIndices.resize(iks[i].chainLength);
    ReadBytes(reinterpret_cast<unsigned char *>(&(iks[i].childBoneIndices[0])),
              sizeof(unsigned short) * iks[i].chainLength, fp);
  }

  model->iks_ = iks;

  return true;
}

//
//
//

PMDReader::PMDReader() {
  size_t n = sizeof(sjis_table)/sizeof(sjis_table_rec_t);
  for(int i = 0; i < n; i++) {
    // DBG
    // wchar_t* unicode_name = (wchar_t*)sjis_table[i].unicode_name;
    // char* ascii_name = sjis_table[i].ascii_name;
    // wchar_t* sjis_name = (wchar_t*)sjis_table[i].sjis_name;
    // printf("unicode_name: %ls, ascii_name: %s, sjis_name: %ls\n", unicode_name, ascii_name, sjis_name);
    sjis_map.insert(sjis_map_t::value_type(sjis_table[i].sjis_name, &sjis_table[i]));
  }
}

PMDReader::~PMDReader() {}

PMDModel *PMDReader::LoadFromFile(const std::string &filename) {
  FILE *fp = fopen(filename.c_str(), "rb");
  if (!fp) {
    fprintf(stderr, "Can't read PMD file [ %s ]\n", filename.c_str());
    return NULL;
  }

  PMDModel *model = new PMDModel();

  // file header
  {
    const char kMagic[] = "Pmd";
    const float kVersion = 1.0f; // 0x3F800000

    char magic[3];
    ReadBytes(reinterpret_cast<unsigned char *>(magic), 3, fp);
    assert(magic[0] == kMagic[0]);
    assert(magic[1] == kMagic[1]);
    assert(magic[2] == kMagic[2]);

    float version = 0.0f;
    ReadBytes(reinterpret_cast<unsigned char *>(&version), 4, fp);
    assert(version == kVersion);

    model->version_ = version;
  }

  // name&comment
  {
    unsigned char name[20];
    unsigned char comment[256];
    ReadBytes(name, 20, fp);
    ReadBytes(comment, 256, fp);

    model->name_ = std::string(reinterpret_cast<char *>(name));
    model->comment_ = std::string(reinterpret_cast<char *>(comment));

    printf("[PMDReader] name = %s\n", model->name_.c_str());
    printf("[PMDReader] comment = %s\n", model->comment_.c_str());
  }

  // Vertices
  {
    int numVertices;
    ReadBytes(reinterpret_cast<unsigned char *>(&numVertices), 4, fp);
    printf("[PMD] Num vertices: %d\n", numVertices);
    assert(sizeof(PMDVertex) == 38);
    model->vertices_.resize(numVertices);
    ReadBytes(reinterpret_cast<unsigned char *>(&(model->vertices_[0])),
              sizeof(PMDVertex) * numVertices, fp);
  }

  // Indices
  {
    int numIndices;
    ReadBytes(reinterpret_cast<unsigned char *>(&numIndices), 4, fp);
    printf("[PMD] Num indices: %d\n", numIndices);
    model->indices_.resize(numIndices);
    ReadBytes(reinterpret_cast<unsigned char *>(&(model->indices_[0])),
              sizeof(unsigned short) * numIndices, fp);

    // validate
    for (int i = 0; i < numIndices; i++) {
      assert(model->indices_[i] < model->vertices_.size());
    }
  }

  // Materials
  {
    int numMaterials;
    ReadBytes(reinterpret_cast<unsigned char *>(&numMaterials), 4, fp);
    printf("[PMD] Num materials: %d\n", numMaterials);
    assert(sizeof(PMDMaterial) == 70);
    model->materials_.resize(numMaterials);
    ReadBytes(reinterpret_cast<unsigned char *>(&(model->materials_[0])),
              sizeof(PMDMaterial) * numMaterials, fp);

    // validate
    size_t sumVertexCount = 0;
    for (int i = 0; i < numMaterials; i++) {
      assert((model->materials_[i].vertex_count % 3) == 0);
      sumVertexCount += model->materials_[i].vertex_count;

      // DBG
      // printf("mat[%d] texname = %s\n", i,
      // model->materials_[i].texture_filename);
    }
    assert(sumVertexCount == model->indices_.size());
  }

  assert(ParseBone(model, fp));
  assert(ParseIK(model, fp));
  assert(ParseMorph(model, fp));

  fclose(fp);

  printf("[PMD] Load OK\n");

  return model;
}
