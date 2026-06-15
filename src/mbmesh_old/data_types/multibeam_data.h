#pragma once

#include "../core/vec3.h"

#include <string>
#include <vector>

struct SwathFile {
  std::string path;
  int format;
  double weight;

  SwathFile() : format(0), weight(1.0) {}

  SwathFile(const std::string& file_path, int file_format, double file_weight = 1.0)
      : path(file_path), format(file_format), weight(file_weight) {}
};

struct Beam {
  int index;
  char flag;
  double depth;
  double amplitude;
  double acrosstrack;
  double alongtrack;
  double travel_time;
  double angle;

  Beam()
      : index(-1),
        flag(0),
        depth(0.0),
        amplitude(0.0),
        acrosstrack(0.0),
        alongtrack(0.0),
        travel_time(0.0),
        angle(0.0) {}
};

struct Ping {
  int index;
  double time_d;
  double longitude;
  double latitude;
  double speed;
  double heading;
  double sensor_depth;
  double altitude;
  std::vector<Beam> beams;

  Ping()
      : index(-1),
        time_d(0.0),
        longitude(0.0),
        latitude(0.0),
        speed(0.0),
        heading(0.0),
        sensor_depth(0.0),
        altitude(0.0) {}
};

struct Sounding {
  Vec3 position;
  Vec3 sensor_position;
  Vec3 look_vector;
  double time_d;
  int ping_index;
  int beam_index;
  char quality_flag;
  double amplitude;

  Sounding()
      : position{0.0, 0.0, 0.0},
        sensor_position{0.0, 0.0, 0.0},
        look_vector{0.0, 0.0, 0.0},
        time_d(0.0),
        ping_index(-1),
        beam_index(-1),
        quality_flag(0),
        amplitude(0.0) {}
};
