
#pragma once

#include <vector>

#include "../math/vec3.h"

struct Sounding {
    Vec3 position;
};

struct Beam {
    std::vector<Sounding> soundings;
};

struct Ping {
    std::vector<Beam> beams;
};

struct SwathFile {
    std::vector<Ping> pings;
};
