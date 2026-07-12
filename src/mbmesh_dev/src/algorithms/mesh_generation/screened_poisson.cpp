#include "algorithms/mesh_generation.h"

#include "algorithms/normal_estimation.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

struct ReconstructionDomain {
    Bounds3D bounds;
    Vec3 origin;
    double cell_size = 1.0;
    int node_count_x = 0;
    int node_count_y = 0;
    int node_count_z = 0;
};

struct SplattedNormalField {
    VectorGrid3D accumulated_normals;
    ScalarGrid3D accumulated_weights;
};

struct PoissonRightHandSide {
    ScalarGrid3D normal_divergence;
};

struct ScreeningConstraintField {
    ScalarGrid3D target_values;
    ScalarGrid3D accumulated_weights;
};

struct PoissonSolution {
    ScalarGrid3D scalar_indicator_field;
};

Bounds3D oriented_pointcloud_bounds(const OrientedPointCloud &oriented_pointcloud) {
    Bounds3D bounds(oriented_pointcloud.oriented_points[0].point,
                    oriented_pointcloud.oriented_points[0].point);
    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        bounds.include(oriented_point.point);
    }
    return bounds;
}

int node_count_for_extent(double extent, double cell_size) {
    return static_cast<int>(std::ceil(extent / cell_size)) + 1;
}

ReconstructionDomain make_reconstruction_domain(const OrientedPointCloud &oriented_pointcloud,
                                                ScreenedPoissonOptions options) {
    ReconstructionDomain domain;
    domain.bounds = oriented_pointcloud_bounds(oriented_pointcloud);
    domain.bounds.expand(options.padding);
    domain.origin = domain.bounds.min;
    domain.cell_size = options.cell_size;

    const Vec3 padded_size = domain.bounds.size();
    domain.node_count_x = node_count_for_extent(padded_size.x, options.cell_size);
    domain.node_count_y = node_count_for_extent(padded_size.y, options.cell_size);
    domain.node_count_z = node_count_for_extent(padded_size.z, options.cell_size);
    return domain;
}

ScalarGrid3D make_scalar_grid_for_domain(const ReconstructionDomain &domain) {
    return ScalarGrid3D(domain.node_count_x,
                        domain.node_count_y,
                        domain.node_count_z,
                        domain.cell_size,
                        domain.origin);
}

VectorGrid3D make_vector_grid_for_domain(const ReconstructionDomain &domain) {
    return VectorGrid3D(domain.node_count_x,
                        domain.node_count_y,
                        domain.node_count_z,
                        domain.cell_size,
                        domain.origin);
}

double compact_quadratic_weight(double distance_squared, double radius_squared) {
    if (radius_squared <= 0.0 || distance_squared > radius_squared) {
        return 0.0;
    }

    const double normalized_distance_squared = distance_squared / radius_squared;
    const double falloff = 1.0 - normalized_distance_squared;
    return falloff * falloff;
}

void splat_oriented_point_normal(SplattedNormalField &normal_field,
                                 const OrientedPoint &oriented_point,
                                 double splat_radius) {
    const int center_x = static_cast<int>(cell_index(oriented_point.point.x,
                                                     normal_field.accumulated_normals.origin.x,
                                                     normal_field.accumulated_normals.cell_size));
    const int center_y = static_cast<int>(cell_index(oriented_point.point.y,
                                                     normal_field.accumulated_normals.origin.y,
                                                     normal_field.accumulated_normals.cell_size));
    const int center_z = static_cast<int>(cell_index(oriented_point.point.z,
                                                     normal_field.accumulated_normals.origin.z,
                                                     normal_field.accumulated_normals.cell_size));
    const int radius_in_cells = std::max(1, static_cast<int>(std::ceil(splat_radius /
                                                                       normal_field.accumulated_normals.cell_size)));
    const double radius_squared = splat_radius * splat_radius;
    const Vec3 unit_normal = oriented_point.normal.length_squared() > 1.0e-20
                                 ? normalize(oriented_point.normal)
                                 : Vec3(0.0, 0.0, 1.0);

    for (int z = center_z - radius_in_cells; z <= center_z + radius_in_cells; z++) {
        for (int y = center_y - radius_in_cells; y <= center_y + radius_in_cells; y++) {
            for (int x = center_x - radius_in_cells; x <= center_x + radius_in_cells; x++) {
                if (!normal_field.accumulated_normals.contains(x, y, z)) {
                    continue;
                }

                const Vec3 grid_node_position = normal_field.accumulated_normals.position(x, y, z);
                const double distance_squared = (grid_node_position - oriented_point.point).length_squared();
                const double weight = compact_quadratic_weight(distance_squared, radius_squared);
                if (weight <= vec3_epsilon) {
                    continue;
                }

                normal_field.accumulated_normals.at(x, y, z) += unit_normal * weight;
                normal_field.accumulated_weights.at(x, y, z) += weight;
            }
        }
    }
}

void normalize_splatted_normal_field(SplattedNormalField &normal_field) {
    for (int z = 0; z < normal_field.accumulated_normals.nz; z++) {
        for (int y = 0; y < normal_field.accumulated_normals.ny; y++) {
            for (int x = 0; x < normal_field.accumulated_normals.nx; x++) {
                const double weight = normal_field.accumulated_weights.at(x, y, z);
                if (weight <= vec3_epsilon) {
                    continue;
                }

                normal_field.accumulated_normals.at(x, y, z) /= weight;
            }
        }
    }
}

SplattedNormalField splat_oriented_normals_to_grid(const OrientedPointCloud &oriented_pointcloud,
                                                   const ReconstructionDomain &domain,
                                                   ScreenedPoissonOptions options) {
    SplattedNormalField normal_field{
        make_vector_grid_for_domain(domain),
        make_scalar_grid_for_domain(domain),
    };

    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        splat_oriented_point_normal(normal_field, oriented_point, options.normal_splat_radius);
    }

    normalize_splatted_normal_field(normal_field);
    return normal_field;
}

void splat_zero_level_surface_constraint(ScreeningConstraintField &screening_constraints,
                                         const OrientedPoint &oriented_point,
                                         double splat_radius) {
    const int center_x = static_cast<int>(cell_index(oriented_point.point.x,
                                                     screening_constraints.accumulated_weights.origin.x,
                                                     screening_constraints.accumulated_weights.cell_size));
    const int center_y = static_cast<int>(cell_index(oriented_point.point.y,
                                                     screening_constraints.accumulated_weights.origin.y,
                                                     screening_constraints.accumulated_weights.cell_size));
    const int center_z = static_cast<int>(cell_index(oriented_point.point.z,
                                                     screening_constraints.accumulated_weights.origin.z,
                                                     screening_constraints.accumulated_weights.cell_size));
    const int radius_in_cells = std::max(1, static_cast<int>(std::ceil(splat_radius /
                                                                       screening_constraints.accumulated_weights.cell_size)));
    const double radius_squared = splat_radius * splat_radius;
    constexpr double surface_target_value = 0.0;

    for (int z = center_z - radius_in_cells; z <= center_z + radius_in_cells; z++) {
        for (int y = center_y - radius_in_cells; y <= center_y + radius_in_cells; y++) {
            for (int x = center_x - radius_in_cells; x <= center_x + radius_in_cells; x++) {
                if (!screening_constraints.accumulated_weights.contains(x, y, z)) {
                    continue;
                }

                const Vec3 grid_node_position = screening_constraints.accumulated_weights.position(x, y, z);
                const double distance_squared = (grid_node_position - oriented_point.point).length_squared();
                const double weight = compact_quadratic_weight(distance_squared, radius_squared);
                if (weight <= 0.0) {
                    continue;
                }

                screening_constraints.target_values.at(x, y, z) += surface_target_value * weight;
                screening_constraints.accumulated_weights.at(x, y, z) += weight;
            }
        }
    }
}

void normalize_screening_constraint_targets(ScreeningConstraintField &screening_constraints) {
    for (int z = 0; z < screening_constraints.target_values.nz; z++) {
        for (int y = 0; y < screening_constraints.target_values.ny; y++) {
            for (int x = 0; x < screening_constraints.target_values.nx; x++) {
                const double weight = screening_constraints.accumulated_weights.at(x, y, z);
                if (weight <= 0.0) {
                    continue;
                }

                screening_constraints.target_values.at(x, y, z) /= weight;
            }
        }
    }
}

ScreeningConstraintField splat_screening_constraints_to_grid(const OrientedPointCloud &oriented_pointcloud,
                                                             const ReconstructionDomain &domain,
                                                             ScreenedPoissonOptions options) {
    ScreeningConstraintField screening_constraints{
        make_scalar_grid_for_domain(domain),
        make_scalar_grid_for_domain(domain),
    };

    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        splat_zero_level_surface_constraint(screening_constraints, oriented_point, options.normal_splat_radius);
    }

    normalize_screening_constraint_targets(screening_constraints);
    return screening_constraints;
}

double vector_component_at(const VectorGrid3D &vector_grid, int x, int y, int z, int component) {
    const Vec3 value = vector_grid.at(x, y, z);
    return value[component];
}

double finite_difference_component(const VectorGrid3D &vector_grid,
                                   int x,
                                   int y,
                                   int z,
                                   int component) {
    int previous_x = x;
    int previous_y = y;
    int previous_z = z;
    int next_x = x;
    int next_y = y;
    int next_z = z;

    if (component == 0) {
        previous_x = std::max(0, x - 1);
        next_x = std::min(vector_grid.nx - 1, x + 1);
    } else if (component == 1) {
        previous_y = std::max(0, y - 1);
        next_y = std::min(vector_grid.ny - 1, y + 1);
    } else {
        previous_z = std::max(0, z - 1);
        next_z = std::min(vector_grid.nz - 1, z + 1);
    }

    const int index_delta = (next_x - previous_x) +
                            (next_y - previous_y) +
                            (next_z - previous_z);
    if (index_delta == 0) {
        return 0.0;
    }

    const double previous_value = vector_component_at(vector_grid,
                                                      previous_x,
                                                      previous_y,
                                                      previous_z,
                                                      component);
    const double next_value = vector_component_at(vector_grid,
                                                  next_x,
                                                  next_y,
                                                  next_z,
                                                  component);
    return (next_value - previous_value) /
           (static_cast<double>(index_delta) * vector_grid.cell_size);
}

double divergence_at_node(const VectorGrid3D &normal_vector_field, int x, int y, int z) {
    const double normal_x_change_along_x = finite_difference_component(normal_vector_field, x, y, z, 0);
    const double normal_y_change_along_y = finite_difference_component(normal_vector_field, x, y, z, 1);
    const double normal_z_change_along_z = finite_difference_component(normal_vector_field, x, y, z, 2);
    return normal_x_change_along_x + normal_y_change_along_y + normal_z_change_along_z;
}

PoissonRightHandSide compute_poisson_right_hand_side(const SplattedNormalField &splatted_normal_field,
                                                     const ReconstructionDomain &domain) {
    PoissonRightHandSide right_hand_side{
        make_scalar_grid_for_domain(domain),
    };

    for (int z = 0; z < splatted_normal_field.accumulated_normals.nz; z++) {
        for (int y = 0; y < splatted_normal_field.accumulated_normals.ny; y++) {
            for (int x = 0; x < splatted_normal_field.accumulated_normals.nx; x++) {
                // Nodes far from all samples retain a zero normal vector and zero weight. Their divergence
                // is therefore zero except where finite differences cross from empty space into a splatted
                // neighborhood; those transitions are exactly the source/sink signal the Poisson solve uses.
                right_hand_side.normal_divergence.at(x, y, z) =
                    divergence_at_node(splatted_normal_field.accumulated_normals, x, y, z);
            }
        }
    }

    return right_hand_side;
}

double jacobi_poisson_update(const ScalarGrid3D &previous_solution,
                             const ScalarGrid3D &right_hand_side,
                             const ScreeningConstraintField *screening_constraints,
                             double screening_weight,
                             int x,
                             int y,
                             int z) {
    const double neighbor_sum =
        previous_solution.at(x - 1, y, z) +
        previous_solution.at(x + 1, y, z) +
        previous_solution.at(x, y - 1, z) +
        previous_solution.at(x, y + 1, z) +
        previous_solution.at(x, y, z - 1) +
        previous_solution.at(x, y, z + 1);
    const double cell_size_squared = previous_solution.cell_size * previous_solution.cell_size;
    double diagonal = 6.0;
    double numerator = neighbor_sum - cell_size_squared * right_hand_side.at(x, y, z);

    if (screening_constraints != nullptr && screening_weight > 0.0) {
        const double sample_weight = screening_constraints->accumulated_weights.at(x, y, z);
        if (sample_weight > 0.0) {
            const double scaled_screening_weight = cell_size_squared * screening_weight * sample_weight;
            diagonal += scaled_screening_weight;
            numerator += scaled_screening_weight * screening_constraints->target_values.at(x, y, z);
        }
    }

    return numerator / diagonal;
}

PoissonSolution solve_poisson_equation(const PoissonRightHandSide &right_hand_side,
                                       const ScreeningConstraintField *screening_constraints,
                                       const ReconstructionDomain &domain,
                                       ScreenedPoissonOptions options) {
    PoissonSolution solution{
        make_scalar_grid_for_domain(domain),
    };

    ScalarGrid3D next_iteration = make_scalar_grid_for_domain(domain);
    const int iteration_count = std::max(0, options.solver_iterations);

    for (int iteration = 0; iteration < iteration_count; iteration++) {
        // The outer boundary is held at zero. This gives the finite grid a simple Dirichlet boundary
        // condition and leaves the padded margin as the place where the field fades away from the samples.
        std::fill(next_iteration.values.begin(), next_iteration.values.end(), 0.0);

        for (int z = 1; z < solution.scalar_indicator_field.nz - 1; z++) {
            for (int y = 1; y < solution.scalar_indicator_field.ny - 1; y++) {
                for (int x = 1; x < solution.scalar_indicator_field.nx - 1; x++) {
                    next_iteration.at(x, y, z) =
                        jacobi_poisson_update(solution.scalar_indicator_field,
                                              right_hand_side.normal_divergence,
                                              screening_constraints,
                                              options.screening_weight,
                                              x,
                                              y,
                                              z);
                }
            }
        }

        solution.scalar_indicator_field.values.swap(next_iteration.values);
    }

    return solution;
}

double trilinear_sample_scalar_field(const ScalarGrid3D &scalar_field, const Vec3 &position) {
    if (scalar_field.nx < 2 || scalar_field.ny < 2 || scalar_field.nz < 2) {
        return 0.0;
    }

    const Vec3 grid_coordinate = (position - scalar_field.origin) / scalar_field.cell_size;
    const int x0 = std::clamp(static_cast<int>(std::floor(grid_coordinate.x)), 0, scalar_field.nx - 2);
    const int y0 = std::clamp(static_cast<int>(std::floor(grid_coordinate.y)), 0, scalar_field.ny - 2);
    const int z0 = std::clamp(static_cast<int>(std::floor(grid_coordinate.z)), 0, scalar_field.nz - 2);
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const int z1 = z0 + 1;

    const double tx = std::clamp(grid_coordinate.x - static_cast<double>(x0), 0.0, 1.0);
    const double ty = std::clamp(grid_coordinate.y - static_cast<double>(y0), 0.0, 1.0);
    const double tz = std::clamp(grid_coordinate.z - static_cast<double>(z0), 0.0, 1.0);

    const double c00 = scalar_field.at(x0, y0, z0) * (1.0 - tx) + scalar_field.at(x1, y0, z0) * tx;
    const double c10 = scalar_field.at(x0, y1, z0) * (1.0 - tx) + scalar_field.at(x1, y1, z0) * tx;
    const double c01 = scalar_field.at(x0, y0, z1) * (1.0 - tx) + scalar_field.at(x1, y0, z1) * tx;
    const double c11 = scalar_field.at(x0, y1, z1) * (1.0 - tx) + scalar_field.at(x1, y1, z1) * tx;
    const double c0 = c00 * (1.0 - ty) + c10 * ty;
    const double c1 = c01 * (1.0 - ty) + c11 * ty;
    return c0 * (1.0 - tz) + c1 * tz;
}

double average_scalar_value_at_samples(const ScalarGrid3D &scalar_field,
                                       const OrientedPointCloud &oriented_pointcloud) {
    if (oriented_pointcloud.oriented_points.empty()) {
        return 0.0;
    }

    double scalar_sum = 0.0;
    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        scalar_sum += trilinear_sample_scalar_field(scalar_field, oriented_point.point);
    }

    return scalar_sum / static_cast<double>(oriented_pointcloud.oriented_points.size());
}

void subtract_scalar_offset(ScalarGrid3D &scalar_field, double scalar_offset) {
    for (double &value : scalar_field.values) {
        value -= scalar_offset;
    }
}

} // namespace

ScalarGrid3D screened_poisson(const OrientedPointCloud &oriented_pointcloud, ScreenedPoissonOptions options) {
    if (oriented_pointcloud.oriented_points.empty() ||
        options.cell_size <= 0.0 ||
        options.normal_splat_radius <= 0.0) {
        return ScalarGrid3D();
    }

    // Stage 1: Build the reconstruction domain.
    // The Poisson solve happens on a regular 3D grid around the samples. Padding gives the implicit
    // field room to transition away from the observed surface instead of being clipped at the input bounds.
    const ReconstructionDomain reconstruction_domain = make_reconstruction_domain(oriented_pointcloud, options);

    // Stage 2: Splat oriented samples into a vector field.
    // Each oriented point contributes its unit normal to nearby grid nodes. The accumulated vector field
    // is the discrete version of the normal field whose divergence will drive the Poisson equation.
    const SplattedNormalField splatted_normal_field =
        splat_oriented_normals_to_grid(oriented_pointcloud, reconstruction_domain, options);

    // Stage 3: Splat sample constraints for screened Poisson reconstruction.
    // The normal field says what gradient the implicit function should have. Screening adds a direct
    // surface-position constraint: near observed samples, the scalar field should stay close to zero.
    const ScreeningConstraintField screening_constraints =
        splat_screening_constraints_to_grid(oriented_pointcloud, reconstruction_domain, options);

    // Stage 4: Compute the Poisson right-hand side from the normal field divergence.
    // Divergence turns the vector field of normals into a scalar source term. Intuitively, it marks where
    // the splatted normals behave like they are flowing out of or into the grid, which is the signal that
    // the scalar reconstruction solve will try to match.
    const PoissonRightHandSide poisson_right_hand_side =
        compute_poisson_right_hand_side(splatted_normal_field, reconstruction_domain);

    // Stage 5: Solve the screened Poisson equation for a scalar indicator field.
    // Without screening, this is laplacian(field) = divergence(normals). With screening enabled, nearby
    // samples add soft constraints that pull the scalar field toward zero at the observed surface.
    const ScreeningConstraintField *active_screening_constraints =
        options.use_screening ? &screening_constraints : nullptr;
    PoissonSolution poisson_solution =
        solve_poisson_equation(poisson_right_hand_side,
                               active_screening_constraints,
                               reconstruction_domain,
                               options);

    // Stage 6: Choose a practical iso-surface convention for marching cubes.
    // The Poisson solution has an arbitrary additive offset. By subtracting the average scalar value at
    // the input samples, the surface samples cluster around iso_value = 0.0.
    if (options.estimate_iso_value_from_samples) {
        const double sample_iso_value =
            average_scalar_value_at_samples(poisson_solution.scalar_indicator_field, oriented_pointcloud);
        subtract_scalar_offset(poisson_solution.scalar_indicator_field, sample_iso_value);
    }

    return poisson_solution.scalar_indicator_field;
}

// Stub, needs to be implemented
Mesh generate_mesh_poisson(const PointCloud &pointcloud, PoissonOptions options) {
    const OrientedPointCloud oriented_pointcloud = estimate_oriented_points(pointcloud, options.normal_neighbors);
    return generate_mesh_poisson(oriented_pointcloud, options);
}

// Stub, needs to be implemented
Mesh generate_mesh_poisson(const OrientedPointCloud &oriented_pointcloud, PoissonOptions options) {
    ScalarGrid3D scalar_grid = screened_poisson(oriented_pointcloud, options.screened_poisson);
    MarchingCubesOptions marching_cubes_options = options.marching_cubes;
    if (options.screened_poisson.estimate_iso_value_from_samples) {
        marching_cubes_options.iso_value = options.screened_poisson.iso_value;
    }
    return marching_cubes(scalar_grid, marching_cubes_options);
}
