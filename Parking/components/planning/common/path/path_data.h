#ifndef PLANNING_COMMON_PATH_PATH_DATA_H_
#define PLANNING_COMMON_PATH_PATH_DATA_H_

#include "planning/common/path/discretized_path.h"

namespace planning {

class PathData
{
public:
    PathData() = default;

public:

    bool SetDiscretizedPath(DiscretizedPath path);

    bool GetPathPointWithPathS(const double s,
                               common::PathPoint *const path_point) const;

    const DiscretizedPath &discretized_path() const;

    const bool& IsTruncated() const{
        return truncated_;
    }

    const double& Offset() const{
        return offset_;
    }

    const size_t number_points() const{
        return discretized_path_.size();
    }

    void set_truncated(){
        truncated_ = true;
    }

    void set_offset(const double& offset){
        offset_ = offset;
    }

    void Clear();

    bool Empty() const;

private:

    DiscretizedPath discretized_path_;

    bool truncated_ = false;
    double offset_;

};

}



#endif
