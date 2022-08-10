import random
from copy import deepcopy
from dataclasses import dataclass
from typing import List
from tabulate import tabulate

import numpy as np
from vehiclegym.road import RoadOptions, Road
from vehicle_models.model_kinematic import KinematicModelParameters
from vehiclegym.trajectory_planner_acados_20221 import VehiclePlannerAcados20221
from vehiclegym.trajectory_planner_acados_20222 import VehiclePlannerAcados20222
from vehiclegym.trajectory_planner_base import VehicleObstacleModel
from vehiclegym.simulator_simple import SimpleSimulator, SimulatorOptions, SimulationStatistics
from vehiclegym.trajectory_planner_base import PlannerOptions
from vehiclegym.trajectory_planner_acados_20221 import VehicleObstacleModel
import time
from vehiclegym.animator import AnimationParameters, Animator, AnimationPlanningColorType
from vehiclegym.old_files.trajectory_planner_20221 import PlanningDataContainer


@dataclass
class ValidationResult:
    obstacle_avoidance_label: str
    class_label: str
    lifting_used: bool
    comp_times: List
    comp_time_mean: float
    comp_time_min: float
    comp_time_max: float
    min_distance: float
    collisions: float


if __name__ == "__main__":
    # general scenario parameters
    simulator_options = SimulatorOptions()
    simulator_options.idx_skip_traj = 2

    n_runs = 1
    n_vehicles = 2
    simulator_options.n_sim = int(20 + 20 * n_vehicles)

    range_velocity = (10, 20)

    planner_classes = [VehiclePlannerAcados20221, VehiclePlannerAcados20222, VehiclePlannerAcados20222]
    planner_classes_labels = ["20221", "20222", "20222"]
    planner_related_lifting = [False, False, True]

    validations = []

    # Create Test vehicle parameters
    model_parameters = []
    for i in range(n_vehicles):
        model_parameters.append(KinematicModelParameters())
        model_parameters[-1].maximum_deceleration_force = 10e3
        model_parameters[-1].maximum_acceleration_force = 15e3
        model_parameters[-1].maximum_velocity = 40.
        model_parameters[-1].maximum_lateral_acc = 10.
        model_parameters[-1].safety_radius = 3.
        model_parameters[-1].chassis_length = 4.7
        model_parameters[-1].chassis_width = 1.8

    # Create planner options
    planner_options_list = []
    planner_options_list_label = []

    planner_options = PlannerOptions()
    planner_options.n_nodes = 30
    planner_options.time_disc = 0.1
    planner_options.v_max_terminal_set = 15
    planner_options.debug_mode = True
    planner_options.n_circles_opp = 2
    planner_options.n_circles_ego = 2
    planner_options.auto_size_circles = False
    planner_options.use_lifting = True
    planner_options.actions_max[0] = 100
    planner_options.actions_min[0] = -100
    planner_options.obstacle_model = VehicleObstacleModel.ELIPSE
    planner_options.panos_logmaxepx = 0.1
    planner_options.increase_opp = 0.0
    planner_options.contraints_slack_weight_sqr = 1e5

    # Add variants of planner options
    planner_options_list.append(planner_options)
    planner_options_list_label.append("ellipse")

    planner_options = deepcopy(planner_options)
    planner_options.obstacle_model = VehicleObstacleModel.CIRCLES
    planner_options.n_circles_opp = 3
    planner_options.n_circles_ego = 3
    planner_options_list.append(planner_options)
    planner_options_list_label.append("circles3x3")

    # planner_options = deepcopy(planner_options)
    # planner_options.obstacle_model = VehicleObstacleModel.CIRCLES
    # planner_options_list.append(planner_options)
    # planner_options_list_label.append("circles2x2")
    #
    # planner_options = deepcopy(planner_options)
    # planner_options.obstacle_model = VehicleObstacleModel.CIRCLES
    # planner_options.n_circles_opp = 2
    # planner_options.n_circles_ego = 1
    # planner_options_list.append(planner_options)
    # planner_options_list_label.append("circles1x2")
    #
    # planner_options = deepcopy(planner_options)
    # planner_options.obstacle_model = VehicleObstacleModel.CIRCLES
    # planner_options.n_circles_opp = 1
    # planner_options.n_circles_ego = 1
    # planner_options_list.append(planner_options)
    # planner_options_list_label.append("circles1x1")
    #
    # planner_options = deepcopy(planner_options)
    # planner_options.obstacle_model = VehicleObstacleModel.PANOS
    # planner_options.contraints_slack_weight_sqr = 1e4
    # planner_options_list.append(planner_options)
    # planner_options_list_label.append("leuven")
    #
    # planner_options = deepcopy(planner_options)
    # planner_options.obstacle_model = VehicleObstacleModel.HYPERPLANE
    # planner_options.contraints_slack_weight_sqr = 1e4
    # planner_options_list.append(planner_options)
    # planner_options_list_label.append("hyperplane")

    # Create test road
    road_options = RoadOptions()
    road_options.road_width = 11
    road_options.n_points = 400
    road_options.random_road_parameters.maximum_kappa = 1 / 150
    road = Road(road_options)
    road.randomize(seed=-1)

    # Create planner
    planner_sets = []
    list_class_labels = []
    list_obstacle_labels = []

    for planner_class, planner_class_label, do_lift in zip(planner_classes, planner_classes_labels,
                                                           planner_related_lifting):
        for planner_opts_current, planner_opts_labels in zip(planner_options_list, planner_options_list_label):
            planner_opts_current.use_lifting = do_lift
            vehicle_planner = []
            for i in range(n_vehicles):
                other_parameters = model_parameters[:i] + model_parameters[i + 1:]
                vehicle_planner.append(planner_class(ego_model_params=model_parameters[i],
                                                     road=road,
                                                     planner_options=planner_opts_current,
                                                     opp_model_params=other_parameters))
            planner_sets.append(vehicle_planner)
            list_class_labels.append(planner_class_label)
            list_obstacle_labels.append(planner_opts_labels)

    for current_planner_set, label_class, label_obstacle in zip(planner_sets, list_class_labels, list_obstacle_labels):
        t_full, t_average, t_max, t_min, collisions_lead, collisions_follow, min_distances = [], [], [], [], [], [], []
        for i_run in range(n_runs):
            # randomize road
            road.randomize()

            rand_velocity = np.random.uniform(range_velocity[0], range_velocity[1])
            initial_states = []
            actions = []
            # Set parameters
            initial_state_ego_f = np.array([1, 0., 0., rand_velocity - 5, 0])

            # Default actions
            action_ego = np.array([0., 50., 2e4])

            initial_states.append(initial_state_ego_f)
            actions.append(action_ego)
            for i in range(n_vehicles - 1):
                s = 30 * i + 50
                n = -1.5 + np.mod(i, 2) * 3
                initial_states.append(np.array([s, n, 0, rand_velocity, 0]))
                actions.append(np.array([n, rand_velocity, 1e5]))

            # Set parameters
            simulator = SimpleSimulator(options=simulator_options,
                                        initial_states=initial_states,
                                        vehicle_parameters=model_parameters,
                                        planners=current_planner_set,
                                        road=road,
                                        default_actions=actions)

            # simulate
            simulator.simulate()

            t_full.append(simulator.statistics.solver_times)

            t_average.append(np.mean(simulator.statistics.solver_times) * 1000.)
            t_max.append(np.max(simulator.statistics.solver_times) * 1000.)
            t_min.append(np.min(simulator.statistics.solver_times) * 1000.)

            collisions_lead.append(np.sum(simulator.statistics.collisions_lead))
            collisions_follow.append(np.sum(simulator.statistics.collisions_follow))
            min_distances.append(np.mean(np.min(simulator.statistics.min_distances, axis=1)))

        collisions_lead = np.mean(np.array(collisions_lead))
        collisions_follow = np.mean(np.array(collisions_follow))
        validations.append(ValidationResult(obstacle_avoidance_label=label_obstacle,
                                            class_label=label_class,
                                            lifting_used=current_planner_set[0].planner_opts.use_lifting,
                                            comp_times=t_full,
                                            comp_time_min=np.mean(np.array(t_min)),
                                            comp_time_max=np.mean(np.array(t_max)),
                                            comp_time_mean=np.mean(np.array(t_average)),
                                            min_distance=np.mean(np.array(min_distances)),
                                            collisions=collisions_lead + collisions_follow))

    table_output = []
    table_header = ["Class", "Obstacle", "Lifting", "t_ave", "t_min", "t_max", "collisions_mean", "min_dist"]
    for validation in validations:
        table_row = [validation.class_label,
                     validation.obstacle_avoidance_label,
                     validation.lifting_used,
                     validation.comp_time_mean,
                     validation.comp_time_min,
                     validation.comp_time_max,
                     validation.collisions,
                     validation.min_distance]
        table_output.append(table_row)
    t = tabulate(table_output, headers=table_header, tablefmt='orgtbl')
    print(t)
