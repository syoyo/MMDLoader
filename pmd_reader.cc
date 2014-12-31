#include <cassert>
#include <cstring> // memcpy
#include <cstdio>

#include "pmd_reader.h"

using namespace mmd;

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

    model->bones_.push_back(bone);
  }

  return true;
}

static bool ParseMorph(PMDModel *model,FILE *fp){
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

PMDReader::PMDReader() {}

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
