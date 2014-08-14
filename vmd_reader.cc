#include <cassert>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <map>

#include "vmd_reader.h"
#include "pmd_model.h"

using namespace mmd;

static bool ReadBytes(unsigned char *out_data, int size, FILE *fp) {
  size_t sz;
  sz = fread(out_data, 1, size, fp);
  assert(sz == size);
  return true;
}

static bool VMDMotionSorter(const VMDMotion &m1, const VMDMotion &m2) {
  return m1.frame_no < m2.frame_no;
}

static bool VMDMorphSorter(const VMDMorph &m1, const VMDMorph &m2) {
  return m1.frame_no < m2.frame_no;
}

static bool VMDCameraFrameSorter(const VMDCamera &m1, const VMDCamera &m2) {
  return m1.frame_no < m2.frame_no;
}

static bool VMDLightFrameSorter(const VMDLight &m1, const VMDLight &m2) {
  return m1.frame_no < m2.frame_no;
}

static void ParseMotion(VMDAnimation *anim, FILE *fp) {
  int numMotions;
  ReadBytes(reinterpret_cast<unsigned char *>(&numMotions), 4, fp);
  printf("[VMD] Num motions: %d\n", numMotions);
  assert(sizeof(VMDMotion) == 111);
  anim->motions_.resize(numMotions);
  ReadBytes(reinterpret_cast<unsigned char *>(&(anim->motions_[0])),
            sizeof(VMDMotion) * numMotions, fp);

  // Motion data may not be ordererd in frame number. Sort it here.
  std::sort(anim->motions_.begin(), anim->motions_.end(), VMDMotionSorter);
}

VMDReader::VMDReader() {}

VMDReader::~VMDReader() {}

VMDAnimation *VMDReader::LoadFromFile(const std::string &filename) {
  FILE *fp = fopen(filename.c_str(), "rb");
  if (!fp) {
    fprintf(stderr, "Can't read VMD file [ %s ]\n", filename.c_str());
    return NULL;
  }

  VMDAnimation *anim = new VMDAnimation();

  // file header
  {
    const char kMagic[] = "Vocaloid Motion Data 0002";

    char header[30];
    ReadBytes(reinterpret_cast<unsigned char *>(header), 30, fp);
    int ret = strncmp(kMagic, header, strlen(kMagic));
    assert(ret == 0);

    char name[20];
    ReadBytes(reinterpret_cast<unsigned char *>(name), 20, fp);

    anim->name_ = std::string(name);

    printf("[VMDReader] name = %s\n", anim->name_.c_str());
  }

  // Motion
  ParseMotion(anim, fp);

  // Morph
  {
    int numMorphs;
    ReadBytes(reinterpret_cast<unsigned char *>(&numMorphs), 4, fp);
    printf("[VMD] Num morphs: %d\n", numMorphs);
    assert(sizeof(VMDMorph) == 23);
    anim->morphs_.resize(numMorphs);
    ReadBytes(reinterpret_cast<unsigned char *>(&(anim->morphs_[0])),
              sizeof(VMDMorph) * numMorphs, fp);

    // Morph data may not be ordererd in frame number. Sort it here.
    std::sort(anim->morphs_.begin(), anim->morphs_.end(), VMDMorphSorter);
    // for (int i = 0; i < anim->morphs_.size(); i++) {
    //  printf("frame_no: %d\n", anim->morphs_[i].frame_no);
    //}
  }

  // Camera frames
  {
    int numCameraFrames;
    ReadBytes(reinterpret_cast<unsigned char *>(&numCameraFrames), 4, fp);
    printf("[VMD] Num camera frames: %d\n", numCameraFrames);
    assert(sizeof(VMDCamera) == 61);
    anim->camera_frames_.resize(numCameraFrames);
    ReadBytes(reinterpret_cast<unsigned char *>(&(anim->camera_frames_[0])),
              sizeof(VMDCamera) * numCameraFrames, fp);

    // Camera frame data may not be ordererd in frame number. Sort it here.
    std::sort(anim->camera_frames_.begin(), anim->camera_frames_.end(),
              VMDCameraFrameSorter);
  }

  // Light frames
  {
    int numLightFrames;
    ReadBytes(reinterpret_cast<unsigned char *>(&numLightFrames), 4, fp);
    printf("[VMD] Num light frames: %d\n", numLightFrames);
    assert(sizeof(VMDLight) == 28);
    anim->light_frames_.resize(numLightFrames);
    ReadBytes(reinterpret_cast<unsigned char *>(&(anim->light_frames_[0])),
              sizeof(VMDLight) * numLightFrames, fp);

    // Light frame data may not be ordererd in frame number. Sort it here.
    std::sort(anim->light_frames_.begin(), anim->light_frames_.end(),
              VMDLightFrameSorter);
  }

  // @todo { Shadows }
  {}

  fclose(fp);

  printf("[VMD] Load OK\n");

  return anim;
}
