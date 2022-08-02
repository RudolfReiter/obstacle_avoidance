import random
from copy import deepcopy
import numpy as np
from vehicle_models.model_kinematic_frenet import FrenetModelWithObstaclesCircles
from vehiclegym.road import RoadOptions, Road
from vehicle_models.model_kinematic import KinematicModelParameters
from vehiclegym.trajectory_planner_acados_20221 import VehiclePlannerAcados20221
from vehiclegym.trajectory_planner_acados_20221 import PlannerOptionsAcados
from vehiclegym.trajectory_planner_acados_20221 import VehicleObstacleModel
import time
from vehiclegym.animator import AnimationParameters, Animator, AnimationPlanningColorType
from vehiclegym.old_files.trajectory_planner_20221 import PlanningDataContainer

if __name__ == "__main__":
    print("-----------------------")
    print("  Test planner 20221   ")
    print("-----------------------")

    # general scenario parameters
    USE_RANDOM = False
    SAVE_ANIMATION = False
    n_sim = int(100)
    idx_skip_traj = 2

    # Generate parameter data classes
    ego_model_params = KinematicModelParameters()
    ego_model_params.maximum_deceleration_force = 10e3
    ego_model_params.maximum_acceleration_force = 15e3
    ego_model_params.maximum_velocity = 40
    ego_model_params.maximum_lateral_acc = 8
    ego_model_params.safety_radius = 3

    opp_model_params_0 = KinematicModelParameters()
    opp_model_params_0.maximum_deceleration_force = 10e3
    opp_model_params_0.maximum_acceleration_force = 15e3
    opp_model_params_0.safety_radius = 3
    opp_model_params_0.maximum_velocity = 22
    opp_model_params_0.maximum_lateral_acc = 12

    opp_model_params_1 = deepcopy(opp_model_params_0)
    opp_model_params_1.maximum_velocity = 17

    # Create planner options
    planner_options = PlannerOptionsAcados()
    planner_options.n_nodes = 30
    planner_options.prediction_uncertainty_backoff_per_s = 0
    planner_options.n_opponent_prediction_nodes = 30
    planner_options.weight_progress_per_s = 0
    planner_options.n_reward_actions = 15
    planner_options.time_disc = 0.1
    planner_options.v_max_terminal_set = 15
    planner_options.n_circles_opp = 1
    planner_options.n_circles_ego = 1
    planner_options.auto_size_circles = False
    planner_options.debug_mode = True
    planner_options.contraints_slack_weight_sqr = 1e4
    planner_options.obstacle_model = VehicleObstacleModel.ELIPSE

    # Create Animation Parameter
    animation_parameter = AnimationParameters()
    animation_parameter.animation_frames_per_sec = 10
    animation_parameter.animation_speed = 1
    animation_parameter.plot_opp_predictions = []
    animation_parameter.plot_safety_circles = [0]
    animation_parameter.plot_ego_plans = []
    animation_parameter.plot_acceleration_arrows = []
    animation_parameter.planning_color_type = AnimationPlanningColorType.UNI
    animation_parameter.fast_animation = True

    # Create test road
    road_options = RoadOptions()
    road_options.road_width = 10
    road_options.n_points = 400
    road_options.s_grid = np.arange(0, road_options.n_points*road_options.delta_s, road_options.delta_s)
    road_options.kappa_grid = np.zeros_like(road_options.s_grid)+1e-6
    road = Road(road_options)

    # Create planner
    vehicle_planner_ego = VehiclePlannerAcados20221(ego_model_params=ego_model_params,
                                                    road=road,
                                                    planner_options=planner_options,
                                                    opp_model_params=[opp_model_params_0, opp_model_params_1])

    vehicle_planner_opp_0 = VehiclePlannerAcados20221(ego_model_params=opp_model_params_0,
                                                      road=road,
                                                      planner_options=planner_options,
                                                      opp_model_params=[ego_model_params, opp_model_params_1])

    vehicle_planner_opp_1 = VehiclePlannerAcados20221(ego_model_params=opp_model_params_1,
                                                      road=road,
                                                      planner_options=planner_options,
                                                      opp_model_params=[opp_model_params_0, ego_model_params])

    # Set parameters
    initial_state_ego_f = np.array([10, 0, 0, 0, 0])
    initial_state_opp_0_f = np.array([30, 3, 0.0, 4., 0])
    initial_state_opp_1_f = np.array([50, -3, 0.0, 4., 0])
    n_rand = 0
    v_rand = 50
    nw_rand = 5e2
    action = np.array([n_rand, v_rand, nw_rand])
    vehicle_planner_ego.warm_start(states_ego=initial_state_ego_f, actions=action)
    vehicle_planner_opp_0.warm_start(states_ego=initial_state_opp_0_f, actions=action)
    vehicle_planner_opp_1.warm_start(states_ego=initial_state_opp_1_f, actions=action)

    vehicle_planner_ego.set_states(states_ego=initial_state_ego_f, actions=action,
                                   states_opp=[initial_state_opp_0_f, initial_state_opp_1_f])
    vehicle_planner_opp_0.set_states(states_ego=initial_state_opp_0_f, actions=action,
                                     states_opp=[initial_state_ego_f, initial_state_opp_1_f])
    vehicle_planner_opp_1.set_states(states_ego=initial_state_opp_1_f, actions=action,
                                     states_opp=[initial_state_ego_f, initial_state_opp_0_f])

    opti_timings = []

    ego_planning_data_container = PlanningDataContainer()
    opp0_planning_data_container = PlanningDataContainer()
    opp1_planning_data_container = PlanningDataContainer()

    for i in range(n_sim):
        # set simulation time
        t_sim = i * idx_skip_traj * planner_options.time_disc

        # Solve and time vehicles
        t_exec_start_ego = time.time()
        vehicle_planner_ego.solve()
        vehicle_planner_opp_0.solve()
        vehicle_planner_opp_1.solve()
        t_exec_stopp_ego = time.time()

        # Get formatted solution
        ego_planning_data = vehicle_planner_ego.get_formatted_solution(t0=t_sim)
        opp0_planning_data = vehicle_planner_opp_0.get_formatted_solution(t0=t_sim)
        opp1_planning_data = vehicle_planner_opp_1.get_formatted_solution(t0=t_sim)

        # Save formatted solution to container with all data
        ego_planning_data_container.add(ego_planning_data)
        opp0_planning_data_container.add(opp0_planning_data)
        opp1_planning_data_container.add(opp1_planning_data)

        # Save optimal timings
        opti_timings.append(t_exec_stopp_ego - t_exec_start_ego)

        if USE_RANDOM:
            n_rand = random.uniform(planner_options.actions_min[0], planner_options.actions_max[0])
            v_rand = random.uniform(planner_options.actions_min[1], planner_options.actions_max[1])
            nw_rand = random.uniform(planner_options.actions_min[2], planner_options.actions_max[2])
            action = np.array([n_rand, v_rand, nw_rand])

        # Set new initial states
        vehicle_planner_ego.set_states(states_ego=ego_planning_data.x[:, idx_skip_traj], actions=action,
                                       states_opp=[opp0_planning_data.x[:, idx_skip_traj],
                                                   opp1_planning_data.x[:, idx_skip_traj]])
        vehicle_planner_opp_0.set_states(states_ego=opp0_planning_data.x[:, idx_skip_traj], actions=action,
                                         states_opp=[ego_planning_data.x[:, idx_skip_traj],
                                                     opp1_planning_data.x[:, idx_skip_traj]])
        vehicle_planner_opp_1.set_states(states_ego=opp1_planning_data.x[:, idx_skip_traj], actions=action,
                                         states_opp=[ego_planning_data.x[:, idx_skip_traj],
                                                     opp0_planning_data.x[:, idx_skip_traj]])

    animator = Animator(animation_parameter=animation_parameter)
    animator.set_data(planning_data_container=[ego_planning_data_container,
                                               opp0_planning_data_container,
                                               opp1_planning_data_container],
                      vehicle_parameter=[ego_model_params, opp_model_params_0, opp_model_params_1],
                      planner_parameter=[planner_options, planner_options, planner_options],
                      road=road)
    animator.animate(save_as_movie=SAVE_ANIMATION)

    print("Average computation time: {}s".format(np.sum(np.array(opti_timings)) / n_sim))
