#include "planning/common/path/path_data.h"

namespace planning {

bool PathData::SetDiscretizedPath(DiscretizedPath path){
    discretized_path_ = std::move(path);
}

bool PathData::GetPathPointWithPathS(const double s,
                                     common::PathPoint *const path_point) const {
    *path_point = discretized_path_.Evaluate(s);
    return true;
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

bool PathData::Empty() const {
  return discretized_path_.empty();
}

void PathData::Clear() {
  discretized_path_.clear();
}





}
