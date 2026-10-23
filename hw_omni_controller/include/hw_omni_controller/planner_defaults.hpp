#pragma once
// Derived from HW nav_executor_init.cpp and pinned configuration; flat-ground adapter.
#include <nav_executor/path_planner/path_planner.hpp>
#include <algorithm>
#include <ranges>
#include <numbers>
namespace hw_omni {
using namespace nav_executor;
inline bool positive_finite(double v){return std::isfinite(v)&&v>0;}
inline bool nonnegative_finite(double v){return std::isfinite(v)&&v>=0;}
inline void require_parameter(bool ok,const char* message){if(!ok)throw std::invalid_argument(message);}
inline nav_executor::PlannerConfig planner_defaults(
    const CapabilityProfile& normal_profile,
    const std::array<CapabilityProfile, 3>& step_profiles
) {
    PlannerConfig c;
    c.occupied_cost_threshold = static_cast<int>(200);
    c.endpoint_direction_norm_max = 0.1;
    c.endpoint_nudge_max_distance = 1.0;
    c.endpoint_goal_reached_distance = 0.5;
    c.minco_seed = {
        .max_point_spacing = 0.5,
        .max_heading_change = 0.5,
    };
    c.directional_terrain.min_alignment_cosine = 0.8;
    const KinoAStar::Params::StartYawRelaxationParams start_yaw_relaxation {
        .root_count = static_cast<int>(6),
        .root_bias_seconds = 0.67,
        .yaw_bias_seconds_per_rad = 0.17,
        .max_discarded_velocity = 0.5,
    };
    const ShapingDynamicsLimits shaping_dynamics {
        .velocity_max = 3.0,
        .tangential_acceleration_max = 1.5,
        .angular_velocity_max = 6.0,
        .angular_acceleration_max = 7.0,
        .lateral_acceleration_max = 2.0,
    };
    const GeometryLimits geometry_limits {
        .curvature_max = 4.0,
        .curvature_rate_max = 8.0,
        .tangent_regularization = 1e-06,
    };
    c.motion_primitives.xy_resolution = 0.1;
    c.motion_primitives.heading_bins = static_cast<int>(80);
    const std::vector<double> curvature_magnitudes = std::vector<double>{1.0,4.0,0.25,1.0,0.0625,0.25};
    const std::vector<double> band_lengths = std::vector<double>{0.32,0.63,1.26};
    require_parameter(
        curvature_magnitudes.size() == c.motion_primitives.curvature_magnitudes.size()
            && band_lengths.size() == c.motion_primitives.band_lengths.size(),
        "motion primitive curvature_magnitudes or band_lengths has an invalid size"
    );
    std::ranges::copy(
        curvature_magnitudes, c.motion_primitives.curvature_magnitudes.begin()
    );
    std::ranges::copy(band_lengths, c.motion_primitives.band_lengths.begin());
    c.motion_primitives.straight_length = 1.26;
    c.motion_primitives.curvature_max = geometry_limits.curvature_max;
    c.spatial_a_star = {
        .obstacle_weight = 1.0,
        .max_expansions = static_cast<int>(1000000),
    };
    const double runup_transition_distance = 0.1;
    c.reference_path = {
        .resample_spacing = 0.1,
        .tangent_lookahead = 0.3,
        .runup_transition_distance = runup_transition_distance,
    };
    c.guide_field = {
        .corridor_width = 1.5,
        .start_bulb_radius = 2.5,
    };
    c.kino_a_star = {
        .start_yaw_relaxation = start_yaw_relaxation,
        .dynamics = shaping_dynamics,
        .speed_bin_count = static_cast<int>(13),
        .collision_check_resolution = 0.075,
        .goal_connection_max_distance = 1.0,
        .guidance_weight = 1.1,
        .deviation_weight = 1.0,
        .heading_weight = 0.25,
        .speed_weight = 0.5,
        .approach_alignment_weight = 4.0,
        .approach_window_weight = 1.0,
        .max_expansions = static_cast<int>(500000),
    };
    c.minco = {
        .weights = {
            .energy = 0.8,
            .time = 12.0,
            .obstacle = 1000.0,
            .curvature = 100.0,
            .curvature_rate = 100.0,
            .directed_regularity = 1000.0,
            .traversal_velocity_window = 2000.0,
            .traversal_alignment = 2000.0,
            .prohibited_traversal = 1000.0,
            .runup_curvature = 200.0,
        },
        .geometry = geometry_limits,
        .directed_cosine_min = 0.1,
        .terrain_gate = {
            .norm_lo = 0.1,
            .norm_hi = 0.9,
        },
        .samples_per_segment = static_cast<int>(16),
        .max_iterations = static_cast<int>(3000),
        .optimizer = {
            .position_scale = 0.5,
            .physical_time_scale = 0.1,
            .max_virtual_time_scale = 20.0,
            .max_function_evaluations = static_cast<int>(4000),
            .history_size = static_cast<int>(8),
            .gradient_tolerance = 3e-06,
            .cost_window_relative_tolerance = 3e-06,
            .cost_window_size = static_cast<int>(10),
            .cost_plateau_gradient_tolerance = 3e-05,
            .scaled_step_tolerance = 1e-12,
            .step_control = {
                .initial_step_cap = 0.5,
                .min_step_cap = 1e-10,
                .max_step_cap = 2.0,
                .expansion_min_model_ratio = 0.75,
                .backtrack_factor = 0.25,
                .expansion_factor = 2.0,
                .max_rejections_per_iteration = static_cast<int>(20),
                .recovery_after_rejections = static_cast<int>(3),
            },
            .curvature_cosine_threshold = 1e-08,
            .history_update_min_model_ratio = 0.25,
        },
        .min_segment_time = 0.05,
        .runup_body_norm_lo = 0.9,
        .runup_body_norm_hi = 0.95,
        .runup_saturation_length = 0.05,
        .runup_transition_distance = runup_transition_distance,
        .debug_diagnostics = false,
    };

    c.traversal_annotation = {
        .sample_spacing = 0.05,
    };
    c.step_execution_timing = {
        .profile_prepare_distance = 2.5,
        .chassis_activation_distance = 1.5,
        .fsm_release_distance = 0.3,
    };
    c.traversal_constraint_gate = {
        .gate_transition_distance = 0.3,
    };
    c.trajectory_validation = {
        .samples_per_segment = static_cast<int>(20),
        .alignment_warning_angle = 0.4,
    };
    c.speed_profile = {
        .discretization = {
            .max_spacing = 0.15,
            .traversal_max_spacing = 0.05,
            .curvature_refine_threshold = 0.15,
            .envelope_sample_spacing = 0.025,
        },
        .objective = {
            .traversal_window = 20.0,
            .lateral_acceleration = 18.0,
            .global_speed_reward = 0.03,
            .velocity_scale = 2.2,
        },
        .stationary_velocity_threshold = 0.1,
        .geometry = geometry_limits,
        .normal_profile = normal_profile,
        .step_profiles = step_profiles,
    };
    require_parameter(
        c.occupied_cost_threshold > 0 && c.occupied_cost_threshold <= 255,
        "planner occupied cost threshold must be in (0, 255]"
    );
    require_parameter(
        positive_finite(c.endpoint_direction_norm_max)
            && c.endpoint_direction_norm_max <= 1.0
            && nonnegative_finite(c.endpoint_nudge_max_distance)
            && nonnegative_finite(c.endpoint_goal_reached_distance),
        "planner endpoint handling parameters are invalid"
    );
    require_parameter(
        positive_finite(c.minco_seed.max_point_spacing)
            && positive_finite(c.minco_seed.max_heading_change)
            && c.minco_seed.max_heading_change <= std::numbers::pi,
        "MINCO seed spacing or heading threshold is invalid"
    );
    require_parameter(
        std::isfinite(c.directional_terrain.min_alignment_cosine)
            && c.directional_terrain.min_alignment_cosine > 0.0
            && c.directional_terrain.min_alignment_cosine < 1.0,
        "directional terrain minimum alignment cosine must be in (0, 1)"
    );
    require_parameter(
        c.kino_a_star.start_yaw_relaxation.root_count >= 1
            && c.kino_a_star.start_yaw_relaxation.root_count
                <= c.motion_primitives.heading_bins
            && nonnegative_finite(
                c.kino_a_star.start_yaw_relaxation.root_bias_seconds
            )
            && nonnegative_finite(
                c.kino_a_star.start_yaw_relaxation.yaw_bias_seconds_per_rad
            )
            && nonnegative_finite(
                c.kino_a_star.start_yaw_relaxation.max_discarded_velocity
            ),
        "start yaw relaxation parameters are invalid"
    );
    require_parameter(
        positive_finite(shaping_dynamics.velocity_max)
        && positive_finite(shaping_dynamics.tangential_acceleration_max)
        && positive_finite(shaping_dynamics.angular_velocity_max)
        && positive_finite(shaping_dynamics.angular_acceleration_max)
        && positive_finite(shaping_dynamics.lateral_acceleration_max)
        && positive_finite(c.motion_primitives.xy_resolution)
        && positive_finite(geometry_limits.curvature_max)
        && positive_finite(geometry_limits.curvature_rate_max)
        && positive_finite(geometry_limits.tangent_regularization)
        && positive_finite(c.motion_primitives.straight_length)
        && nonnegative_finite(c.spatial_a_star.obstacle_weight)
        && positive_finite(c.reference_path.resample_spacing)
        && positive_finite(c.reference_path.tangent_lookahead)
        && positive_finite(c.guide_field.corridor_width)
        && positive_finite(c.guide_field.start_bulb_radius)
        && c.guide_field.start_bulb_radius >= c.guide_field.corridor_width
        && positive_finite(c.kino_a_star.collision_check_resolution)
        && positive_finite(c.kino_a_star.goal_connection_max_distance)
        && nonnegative_finite(c.kino_a_star.guidance_weight)
        && nonnegative_finite(c.kino_a_star.deviation_weight)
        && nonnegative_finite(c.kino_a_star.heading_weight)
        && nonnegative_finite(c.kino_a_star.speed_weight)
        && nonnegative_finite(c.kino_a_star.approach_alignment_weight)
        && nonnegative_finite(c.kino_a_star.approach_window_weight)
        && nonnegative_finite(c.reference_path.runup_transition_distance),
        "spatial/reference/corridor geometry, dynamics, and guidance weights must be finite and valid"
    );
    require_parameter(
        std::ranges::all_of(
            c.motion_primitives.curvature_magnitudes,
            [&](const double value) {
                return positive_finite(value)
                    && value <= c.motion_primitives.curvature_max;
            }
        ) && std::ranges::all_of(
            c.motion_primitives.band_lengths, positive_finite
        ),
        "motion primitive bands must be finite, positive, and within curvature_max"
    );
    require_parameter(
        c.motion_primitives.heading_bins > 0
        && c.spatial_a_star.max_expansions > 0
        && c.kino_a_star.speed_bin_count >= 2
        && c.kino_a_star.max_expansions > 0,
        "lattice pose/speed bins and expansion limit must be valid"
    );
    require_parameter(
        std::isfinite(c.minco.directed_cosine_min)
        && c.minco.directed_cosine_min > 0.0
        && c.minco.directed_cosine_min < 1.0,
        "MINCO directed_cosine_min must be in (0, 1)"
    );
    const auto& minco_weights = c.minco.weights;
    require_parameter(
        nonnegative_finite(minco_weights.energy)
        && nonnegative_finite(minco_weights.time)
        && nonnegative_finite(minco_weights.obstacle)
        && nonnegative_finite(minco_weights.curvature)
        && nonnegative_finite(minco_weights.curvature_rate)
        && nonnegative_finite(minco_weights.directed_regularity)
        && nonnegative_finite(minco_weights.traversal_velocity_window)
        && nonnegative_finite(minco_weights.traversal_alignment)
        && nonnegative_finite(minco_weights.prohibited_traversal)
        && nonnegative_finite(minco_weights.runup_curvature),
        "MINCO penalty weights must be finite and non-negative"
    );
    require_parameter(
        positive_finite(c.minco.min_segment_time),
        "MINCO min_segment_time must be finite and positive"
    );
    require_parameter(
        c.minco.samples_per_segment > 0 && c.minco.max_iterations > 0,
        "MINCO sample and iteration counts must be positive"
    );
    const auto& minco_optimizer = c.minco.optimizer;
    const auto& step_control = minco_optimizer.step_control;
    require_parameter(
        positive_finite(minco_optimizer.position_scale)
        && positive_finite(minco_optimizer.physical_time_scale)
        && positive_finite(minco_optimizer.max_virtual_time_scale)
        && minco_optimizer.max_virtual_time_scale >= minco_optimizer.physical_time_scale,
        "MINCO optimizer variable scales must be finite, positive, and ordered"
    );
    require_parameter(
        minco_optimizer.max_function_evaluations > 0
        && minco_optimizer.history_size > 0
        && positive_finite(minco_optimizer.gradient_tolerance)
        && positive_finite(minco_optimizer.cost_window_relative_tolerance)
        && minco_optimizer.cost_window_size > 0
        && positive_finite(minco_optimizer.cost_plateau_gradient_tolerance)
        && minco_optimizer.cost_plateau_gradient_tolerance
            >= minco_optimizer.gradient_tolerance
        && positive_finite(minco_optimizer.scaled_step_tolerance),
        "MINCO optimizer limits and convergence tolerances are invalid"
    );
    require_parameter(
        positive_finite(step_control.min_step_cap)
        && positive_finite(step_control.initial_step_cap)
        && positive_finite(step_control.max_step_cap)
        && step_control.min_step_cap <= step_control.initial_step_cap
        && step_control.initial_step_cap <= step_control.max_step_cap,
        "MINCO optimizer step caps are invalid"
    );
    require_parameter(
        std::isfinite(step_control.expansion_min_model_ratio)
        && step_control.expansion_min_model_ratio > 0.0
        && step_control.expansion_min_model_ratio < 1.0
        && positive_finite(step_control.backtrack_factor)
        && step_control.backtrack_factor < 1.0
        && std::isfinite(step_control.expansion_factor)
        && step_control.expansion_factor > 1.0
        && step_control.max_rejections_per_iteration > 0
        && step_control.recovery_after_rejections > 0
        && step_control.recovery_after_rejections
            < step_control.max_rejections_per_iteration,
        "MINCO optimizer step-control factors or rejection limits are invalid"
    );
    require_parameter(
        positive_finite(minco_optimizer.curvature_cosine_threshold)
        && minco_optimizer.curvature_cosine_threshold < 1.0
        && std::isfinite(minco_optimizer.history_update_min_model_ratio)
        && minco_optimizer.history_update_min_model_ratio >= 0.0
        && minco_optimizer.history_update_min_model_ratio < 1.0,
        "MINCO optimizer curvature parameters are invalid"
    );
    require_parameter(
        c.minco.runup_body_norm_lo >= 0.9
        && c.minco.runup_body_norm_hi > c.minco.runup_body_norm_lo
        && c.minco.runup_body_norm_hi <= 1.0
        && positive_finite(c.minco.runup_saturation_length)
        && nonnegative_finite(c.minco.runup_transition_distance),
        "MINCO runup body gate or distance parameters are invalid"
    );
    require_parameter(
        c.minco.terrain_gate.norm_lo >= 0.0
        && c.minco.terrain_gate.norm_hi > c.minco.terrain_gate.norm_lo
        && c.minco.terrain_gate.norm_hi <= 1.0,
        "MINCO terrain_gate thresholds are invalid"
    );
    require_parameter(
        positive_finite(c.traversal_annotation.sample_spacing)
            && nonnegative_finite(c.step_execution_timing.profile_prepare_distance)
            && nonnegative_finite(c.step_execution_timing.chassis_activation_distance)
            && c.step_execution_timing.profile_prepare_distance
                >= c.step_execution_timing.chassis_activation_distance
            && nonnegative_finite(c.step_execution_timing.fsm_release_distance)
            && nonnegative_finite(
                c.traversal_constraint_gate.gate_transition_distance
            ),
        "traversal annotation, execution timing, or constraint gate is invalid"
    );
    require_parameter(
        c.trajectory_validation.samples_per_segment > 0
        && nonnegative_finite(c.trajectory_validation.alignment_warning_angle)
        && c.trajectory_validation.alignment_warning_angle
            <= std::numbers::pi,
        "environment validation parameters are invalid"
    );
    const auto& speed_profile = c.speed_profile;
    require_parameter(
        positive_finite(speed_profile.discretization.max_spacing)
        && positive_finite(speed_profile.discretization.traversal_max_spacing)
        && speed_profile.discretization.traversal_max_spacing
            <= speed_profile.discretization.max_spacing
        && positive_finite(speed_profile.discretization.curvature_refine_threshold)
        && positive_finite(speed_profile.discretization.envelope_sample_spacing)
        && nonnegative_finite(speed_profile.objective.traversal_window)
        && nonnegative_finite(speed_profile.objective.lateral_acceleration)
        && nonnegative_finite(speed_profile.objective.global_speed_reward)
        && positive_finite(speed_profile.objective.velocity_scale)
        && nonnegative_finite(speed_profile.stationary_velocity_threshold),
        "speed profile discretization, objective, or stationary threshold is invalid"
    );
    c.enable_diagnostics = false;
    return c;
}
}
