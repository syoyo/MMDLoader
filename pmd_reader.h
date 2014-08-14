#ifndef __MMD_PMD_READER_H__
#define __MMD_PMD_READER_H__

#include "pmd_model.h"

namespace mmd {

class PMDReader {
public:
  PMDReader();
  ~PMDReader();

  PMDModel *LoadFromFile(const std::string &filename);

private:
};

} // namespace

#endif // __MMD_PMD_READER_H__
