#pragma once

#include "../data_types/geometry.h"

[[nodiscard]] PointCloud generate_sphere_pointcloud(double radius = 1.0,
                                                    int latitude_steps = 48,
                                                    int longitude_steps = 96);

[[nodiscard]] PointCloud generate_torus_pointcloud(double major_radius = 1.0,
                                                   double minor_radius = 0.25,
                                                   int major_steps = 96,
                                                   int minor_steps = 32);

[[nodiscard]] PointCloud generate_arch_pointcloud(double radius = 1.0,
                                                  double length = 3.0,
                                                  int arch_steps = 48,
                                                  int length_steps = 96);

[[nodiscard]] PointCloud generate_cave_wall_pointcloud(double width = 4.0,
                                                       double height = 2.5,
                                                       int width_steps = 120,
                                                       int height_steps = 80);

[[nodiscard]] PointCloud generate_two_sheet_overhang_pointcloud(double width = 4.0,
                                                                double length = 4.0,
                                                                int width_steps = 120,
                                                                int length_steps = 120);

[[nodiscard]] PointCloud generate_terrain_half_torus_arch_pointcloud(double width = 5.0,
                                                                     double length = 5.0,
                                                                     int terrain_width_steps = 150,
                                                                     int terrain_length_steps = 150,
                                                                     int arch_steps = 96,
                                                                     int tube_steps = 28);
