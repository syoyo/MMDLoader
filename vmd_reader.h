#ifndef __MMD_VMD_READER_H__
#define __MMD_VMD_READER_H__

#include "vmd_animation.h"

namespace mmd {

class VMDReader {
public:
  VMDReader();
  ~VMDReader();

  VMDAnimation *LoadFromFile(const std::string &filename);

private:
};

} // namespace

#endif // __MMD_VMD_READER_H__

// vim:set sw=2 ts=2 expandtab:
