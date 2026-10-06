#include "Map.h"
#include <cstdio>
int main() {
    ORB_SLAM3::Map map(0);
    const auto id = map.GetId();
    const auto initial = map.GetLastBigChangeIdx();
    map.InformNewBigChange();
    if(map.GetLastBigChangeIdx() <= initial) return 1;
    const auto corrected = map.GetLastBigChangeIdx();
    map.SetImuInitialized();
    map.clear();
    if(map.GetId() != id || map.isImuInitialized() ||
       map.GetLastBigChangeIdx() <= corrected) {
        std::fprintf(stderr, "Same-ID inertial map reset lost its correction epoch\n");
        return 2;
    }
    return 0;
}
