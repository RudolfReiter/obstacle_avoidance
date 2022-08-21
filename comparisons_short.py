import pickle
from copy import deepcopy
from dataclasses import dataclass
from typing import List
from tabulate import tabulate
import numpy as np
import pickle
from vehiclegym.automotive_datastructures import CartesianTrajectory
from vehiclegym.trajectory_planner_base import VehicleObstacleModel
from vehiclegym.road import RoadOptions, Road
from vehicle_models.model_kinematic import KinematicModelParameters
from vehiclegym.trajectory_planner_acados_20222 import VehiclePlannerAcados20222
from vehiclegym.trajectory_planner_acados_20222 import PlannerOptions
import time
from vehiclegym.animator import AnimationParameters, Animator, AnimationPlanningColorType
from vehiclegym.simulator_simple import SimulatorOptions, SimpleSimulator

from utils import PlannerOption, ValidationResult

if __name__ == "__main__":
    print("-----------------------")
    print("  Test planner 20222   ")
    print("-----------------------")

    # general scenario parameters
    SAVE = True
    f_name = "save_data_short_safe_3"
    USE_RANDOM = False
    ANIMATE = False
    n_sim = int(80)
    n_runs = 50
    idx_skip_traj = 2
    validations = []
    planner_options = [
        PlannerOption(str_obstacle="ellipse", obstacle=VehicleObstacleModel.ELIPSE, lifting=True, weight_obstacle=1e6,
                      circles=3),
        PlannerOption(str_obstacle="ellipse", obstacle=VehicleObstacleModel.ELIPSE, lifting=False, weight_obstacle=1e6,
                      circles=3),

        PlannerOption(str_obstacle="circle5x5", obstacle=VehicleObstacleModel.CIRCLES, lifting=True,
                      weight_obstacle=1e6, circles=5),
        PlannerOption(str_obstacle="circle5x5", obstacle=VehicleObstacleModel.CIRCLES, lifting=False,
                      weight_obstacle=1e6, circles=5),

        PlannerOption(str_obstacle="circle3x3", obstacle=VehicleObstacleModel.CIRCLES, lifting=True,
                      weight_obstacle=1e6, circles=3),
        PlannerOption(str_obstacle="circle3x3", obstacle=VehicleObstacleModel.CIRCLES, lifting=False,
                      weight_obstacle=1e6, circles=3),

        PlannerOption(str_obstacle="hyperplane1e5", obstacle=VehicleObstacleModel.HYPERPLANE, lifting=True,
                      weight_obstacle=1e5, circles=1),
        PlannerOption(str_obstacle="hyperplane1e5", obstacle=VehicleObstacleModel.HYPERPLANE, lifting=False,
                      weight_obstacle=1e5, circles=1)
    ]
    for options in planner_options:

        # Generate parameter data classes
        ego_model_params = KinematicModelParameters()
        ego_model_params.maximum_deceleration_force = 10e3
        ego_model_params.maximum_acceleration_force = 15e3
        ego_model_params.maximum_velocity = 40
        ego_model_params.maximum_lateral_acc = 18
        ego_model_params.length_rear = 1.7
        ego_model_params.length_front = 1.7
        ego_model_params.safety_radius = 2.5
        ego_model_params.chassis_length = 4

        opp_model_params_0 = KinematicModelParameters()
        opp_model_params_0.maximum_deceleration_force = 10e3
        opp_model_params_0.maximum_acceleration_force = 15e3
        opp_model_params_0.safety_radius = 3
        opp_model_params_0.maximum_velocity = 30
        opp_model_params_0.maximum_lateral_acc = 12
        opp_model_params_0.chassis_length = 5
        opp_model_params_0.chassis_width = 2.8
        opp_model_params_0.length_rear = 2
        opp_model_params_0.length_front = 2

        opp_model_params_1 = deepcopy(opp_model_params_0)

        # Create planner options
        planner_options = PlannerOptions()
        planner_options.n_nodes = 30
        planner_options.time_disc = 0.1
        planner_options.v_max_terminal_set = 15
        planner_options.debug_mode = True
        planner_options.obstacle_model = options.obstacle
        planner_options.increase_opp = 0.
        planner_options.n_circles_opp = options.circles
        planner_options.n_circles_ego = options.circles
        planner_options.auto_size_circles = True
        planner_options.increase_opp = 0.0
        planner_options.use_lifting = options.lifting
        planner_options.panos_logmaxepx = np.sqrt(0.01)

        planner_options.contraints_slack_weight_sqr = options.weight_obstacle  # use 1e4 for hyperplane

        opp_planner_options = deepcopy(planner_options)
        opp_planner_options.deactivate_obstacle_avoidance = True

        # Create Animation Parameter
        animation_parameter = AnimationParameters()
        animation_parameter.animation_frames_per_sec = 10
        animation_parameter.animation_speed = 3.
        animation_parameter.plot_opp_predictions = [0]
        animation_parameter.plot_safety_circles = [0, 1, 2]
        animation_parameter.plot_ego_plans = [0]
        animation_parameter.plot_acceleration_arrows = [0]
        animation_parameter.planning_color_type = AnimationPlanningColorType.UNI
        animation_parameter.fast_animation = True

        # Create test road
        road_options = RoadOptions()
        road_options.road_width = 15
        road_options.n_points = 400
        road_options.random_road_parameters.maximum_kappa = 1 / 900
        road = Road(road_options)
        road.randomize(seed=-1)
        road.set_kappa(kappa_grid=road.kappa_grid_ + (-1 / 50 + np.cumsum(np.ones_like(road.kappa_grid_)) / 15000))

        # Create planner
        vehicle_planner_ego = VehiclePlannerAcados20222(ego_model_params=ego_model_params,
                                                        road=road,
                                                        planner_options=planner_options,
                                                        opp_model_params=[opp_model_params_0, opp_model_params_1])

        vehicle_planner_opp_0 = VehiclePlannerAcados20222(ego_model_params=opp_model_params_0,
                                                          road=road,
                                                          planner_options=opp_planner_options,
                                                          opp_model_params=[ego_model_params, opp_model_params_1])

        vehicle_planner_opp_1 = VehiclePlannerAcados20222(ego_model_params=opp_model_params_1,
                                                          road=road,
                                                          planner_options=opp_planner_options,
                                                          opp_model_params=[ego_model_params, opp_model_params_0])

        speed_opp = 15
        dist = 60.
        ini_dist = 100
        n_pos = -0
        # Set parameters
        initial_state_ego_f = np.array([1, 0., 0., speed_opp - 5, 0])
        initial_state_opp_0_f = np.array([ini_dist, n_pos, 0.0, speed_opp, 0])
        initial_state_opp_1_f = np.array([ini_dist + dist, n_pos, 0.0, speed_opp, 0])

        # Default actions
        action0 = np.array([0., 50., 1e2])
        action1 = np.array([n_pos, speed_opp, 1e5])
        action2 = np.array([n_pos, speed_opp, 1e5])

        # Set parameters
        simulator_options = SimulatorOptions()
        simulator_options.n_sim = int(n_sim)
        simulator_options.idx_skip_traj = idx_skip_traj
        simulator = SimpleSimulator(options=simulator_options,
                                    initial_states=[initial_state_ego_f, initial_state_opp_0_f, initial_state_opp_1_f],
                                    vehicle_parameters=[ego_model_params, opp_model_params_0, opp_model_params_1],
                                    planners=[vehicle_planner_ego, vehicle_planner_opp_0, vehicle_planner_opp_1],
                                    road=road,
                                    default_actions=[action0, action1, action2])

        t_full, t_average, t_max, t_min, collisions_lead, collisions_follow, min_distances = \
            [], [], [], [], [], [], []
        t_qp, t_sim, qp_iter, s_final = [], [], [], []
        for i in range(n_runs):
            # simulate
            simulator.simulate()

            if ANIMATE:
                animator = Animator(animation_parameter=animation_parameter)
                animator.set_data(planning_data_container=simulator.planning_containers,
                                  vehicle_parameter=[ego_model_params, opp_model_params_0, opp_model_params_1],
                                  planner_parameter=[planner_options, opp_planner_options, opp_planner_options],
                                  road=road,
                                  statistics=simulator.statistics)
                animator.animate(save_as_movie=False)

            t_full.append(simulator.statistics.solver_times)

            t_average.append(np.mean(simulator.statistics.solver_times) * 1000.)
            t_max.append(np.max(simulator.statistics.solver_times) * 1000.)
            t_min.append(np.min(simulator.statistics.solver_times) * 1000.)

            collisions_lead.append(np.sum(simulator.statistics.collisions_lead))
            collisions_follow.append(np.sum(simulator.statistics.collisions_follow))
            min_distances.append(np.mean(np.min(simulator.statistics.min_distances, axis=1)))

            t_qp.append(np.mean(simulator.statistics.time_qps))
            t_sim.append(np.mean(simulator.statistics.time_sims))
            qp_iter.append(np.mean(simulator.statistics.qp_iters))
            s_final.append(simulator.current_states[0][0])

            #road.randomize(seed=i)
            np.random.seed(seed=i)
            delta_ss = np.random.uniform(-15, 35, 3)
            if delta_ss[0]+60<delta_ss[1]:
                delta_ss[1]=delta_ss[0]+60

            initial_state_opp_0_f[0] = initial_state_opp_0_f[0] + delta_ss[0]
            initial_state_opp_1_f[0] = initial_state_opp_1_f[0] + delta_ss[1]
            simulator.reset([initial_state_ego_f, initial_state_opp_0_f, initial_state_opp_1_f], road=road)

        collisions = float(np.sum(np.array(collisions_lead + collisions_follow) > 0))
        label_class = "20222"
        validations.append(ValidationResult(obstacle_avoidance_label=options.str_obstacle,
                                            class_label=label_class,
                                            lifting_used=planner_options.use_lifting,
                                            comp_times=t_full,
                                            comp_time_min=np.mean(np.array(t_min)),
                                            comp_time_max=np.mean(np.array(t_max)),
                                            comp_time_mean=np.mean(np.array(t_average)),
                                            min_distance=np.mean(np.array(min_distances)),
                                            collisions=collisions,
                                            qp_iter=np.mean(np.array(qp_iter)),
                                            sim_time=np.mean(np.array(t_sim)),
                                            qp_time=np.mean(np.array(t_qp)),
                                            s_final=np.mean(np.array(s_final)),
                                            s_final_all=s_final))

    table_output = []
    table_header = ["Class", "Obstacle", "Lifting", "t_ave", "t_min", "t_max", "collisions_mean", "s_final", "min_dist",
                    "t_sim", "t_qp", "qp_iter"]
    for validation in validations:
        table_row = [validation.class_label,
                     validation.obstacle_avoidance_label,
                     validation.lifting_used,
                     validation.comp_time_mean,
                     validation.comp_time_min,
                     validation.comp_time_max,
                     validation.collisions,
                     validation.s_final,
                     validation.min_distance,
                     validation.sim_time,
                     validation.qp_time,
                     validation.qp_iter]
        table_output.append(table_row)
    t = tabulate(table_output, headers=table_header, tablefmt='orgtbl')
    print(t)
    if SAVE:
        with open(f_name, "wb") as fp:  # Pickling
            pickle.dump(validations, fp)

