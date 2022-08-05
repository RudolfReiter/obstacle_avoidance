import random
from copy import deepcopy
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

if __name__ == "__main__":
    # general scenario parameters
    simulator_options = SimulatorOptions()
    simulator_options.n_sim = int(140)
    simulator_options.idx_skip_traj = 2

    n_runs = 10
    n_vehicles = 3

    range_velocity = (20, 40)
    range_lat_acc = (8, 12)

    planner_classes = [VehiclePlannerAcados20221, VehiclePlannerAcados20222]
    planner_classes_labels = ["frenet", "dual"]

    # Create Test vehicle parameters
    model_parameters = []
    for i in range(n_vehicles):
        model_parameters.append(KinematicModelParameters())
        model_parameters[-1].maximum_deceleration_force = 10e3
        model_parameters[-1].maximum_acceleration_force = 15e3
        model_parameters[-1].maximum_velocity = range_velocity[1] - i * (
                range_velocity[1] - range_velocity[0]) / n_vehicles
        model_parameters[-1].maximum_lateral_acc = np.random.uniform(range_lat_acc[0], range_lat_acc[1])
        model_parameters[-1].safety_radius = 3
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
    planner_options.actions_max[0] = 100
    planner_options.actions_min[0] = -100
    planner_options.obstacle_model = VehicleObstacleModel.ELIPSE
    planner_options.contraints_slack_weight_sqr = 5e4

    # Add variants of planner options
    planner_options_list.append(planner_options)
    planner_options_list_label.append("ellipse")

    planner_options = deepcopy(planner_options)
    planner_options.obstacle_model = VehicleObstacleModel.CIRCLES
    planner_options_list.append(planner_options)
    planner_options_list_label.append("circles")

    # Create test road
    road_options = RoadOptions()
    road_options.road_width = 10
    road_options.n_points = 400
    road_options.s_grid = np.arange(0, road_options.n_points * road_options.delta_s, road_options.delta_s)
    road_options.kappa_grid = np.zeros_like(road_options.s_grid) + 1e-6
    road = Road(road_options)

    # Create planner
    planner_sets = []
    planner_sets_labels = []

    for planner_class, planner_class_label in zip(planner_classes, planner_classes_labels):
        for planner_opts_current, planner_opts_labels in zip(planner_options_list, planner_options_list_label):
            vehicle_planner = []
            for i in range(n_vehicles):
                other_parameters = model_parameters[:i] + model_parameters[i + 1:]
                vehicle_planner.append(VehiclePlannerAcados20221(ego_model_params=model_parameters[i],
                                                                 road=road,
                                                                 planner_options=planner_opts_current,
                                                                 opp_model_params=other_parameters))
            planner_sets.append(vehicle_planner)
            planner_sets_labels.append(planner_class_label + "_" + planner_opts_labels)

    for current_planner_set, label in zip(planner_sets, planner_sets_labels):
        t_average, t_max, t_min, collisions_lead, collisions_follow, min_distances = [], [], [], [], [], []
        for i_run in range(n_runs):
            initial_states = []
            for i in range(n_vehicles):
                s = np.random.uniform(1 + i * 20, 10)
                n = np.random.uniform(-road.road_options_.road_width / 4 + road.road_options_.road_width / 4)
                initial_states.append(np.array([s, n, 0, 0, 0]))

            # Set parameters
            simulator = SimpleSimulator(options=simulator_options,
                                        initial_states=initial_states,
                                        vehicle_parameters=model_parameters,
                                        planners=current_planner_set,
                                        road=road)

            # simulate
            simulator.simulate()

            t_average.append(np.mean(simulator.statistics.solver_times) * 1000.)
            t_max.append(np.max(simulator.statistics.solver_times) * 1000.)
            t_min.append(np.min(simulator.statistics.solver_times) * 1000.)

            collisions_lead.append(np.sum(simulator.statistics.collisions_lead))
            collisions_follow.append(np.sum(simulator.statistics.collisions_follow))
            min_distances.append(np.mean(np.min(simulator.statistics.min_distances, axis=1)))

        # print result
        print("-----------------------------")
        print("Results for setting: " + label)
        print("-----------------------------")
        t_min = np.mean(np.array(t_min))
        t_average = np.mean(np.array(t_average))
        t_max = np.mean(np.array(t_max))
        collisions_lead = np.mean(np.array(collisions_lead))
        collisions_follow = np.mean(np.array(collisions_follow))
        min_distances = np.mean(np.array(min_distances))

        t_max = np.mean(np.array(t_max))
        print("Mean computation time (ms) [ave/min/max]: {:4.2f}/ {:4.2f}/ {:4.2f}".format(t_min, t_average, t_max))
        print("Mean lead collisions: {}".format(collisions_lead))
        print("Mean follow collisions: {}".format(collisions_follow))
        print("Mean of minimum opponent distance: {}".format(min_distances))
