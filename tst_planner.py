from copy import deepcopy
from dataclasses import dataclass
from typing import List
import numpy as np
from tabulate import tabulate
from vehiclegym.trajectory_planner_base import VehicleObstacleModel
from vehiclegym.road import RoadOptions, Road
from vehicle_models.model_kinematic import KinematicModelParameters
from vehiclegym.trajectory_planner_acados_20222 import VehiclePlannerAcados20222
from vehiclegym.trajectory_planner_acados_20222 import PlannerOptions
from vehiclegym.animator import AnimationParameters, Animator, AnimationPlanningColorType
from vehiclegym.simulator_simple import SimulatorOptions, SimpleSimulator



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
    qp_iter: float
    sim_time: float
    qp_time: float
    s_final: float


if __name__ == "__main__":
    # general scenario parameters
    simulator_options = SimulatorOptions()
    simulator_options.idx_skip_traj = 2

    ANIMATE = True
    n_runs = 10
    n_vehicles = 3
    simulator_options.n_sim = int(10 + 10 * n_vehicles)*2

    range_velocity = (10, 15)

    planner_classes = [VehiclePlannerAcados20222]
    planner_classes_labels = ["20222"]
    planner_related_lifting = [True]

    # Create Animation Parameter
    animation_parameter = AnimationParameters()
    animation_parameter.animation_frames_per_sec = 10
    animation_parameter.animation_speed = 1.
    animation_parameter.plot_opp_predictions = [0]
    animation_parameter.plot_safety_circles = [0, 1, 2]
    animation_parameter.plot_ego_plans = [0, 1, 2]
    animation_parameter.plot_acceleration_arrows = [0]
    animation_parameter.planning_color_type = AnimationPlanningColorType.UNI
    animation_parameter.fast_animation = False

    validations = []

    # Create Test vehicle parameters
    model_parameters = []
    for i in range(n_vehicles):
        model_parameters.append(KinematicModelParameters())
        model_parameters[-1].maximum_deceleration_force = 10e3
        model_parameters[-1].maximum_acceleration_force = 15e3
        model_parameters[-1].maximum_velocity = 40.
        model_parameters[-1].maximum_lateral_acc = 18.
        model_parameters[-1].length_rear = 2
        model_parameters[-1].length_front = 2
        model_parameters[-1].safety_radius = 2.5
        model_parameters[-1].chassis_length = 4
        model_parameters[-1].chassis_width = 1.8
        if i >= 1:
            model_parameters[-1].chassis_length = 6
            model_parameters[-1].chassis_width = 2.8
            model_parameters[-1].length_front = 3.
            model_parameters[-1].length_rear = 3.
            model_parameters[-1].maximum_lateral_acc = 12
            model_parameters[-1].maximum_velocity = 30
            model_parameters[-1].safety_radius = 1


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
    planner_options.auto_size_circles = True
    planner_options.use_lifting = True
    planner_options.obstacle_model = VehicleObstacleModel.ELIPSE
    planner_options.panos_logmaxepx = np.sqrt(0.01)
    planner_options.increase_opp = 0.0
    planner_options.contraints_slack_weight_sqr = 1e6

    #Add variants of planner options
    planner_options_list.append(planner_options)
    planner_options_list_label.append("ellipse")
    #
    # planner_options = deepcopy(planner_options)
    # planner_options.obstacle_model = VehicleObstacleModel.CIRCLES
    # planner_options.n_circles_opp = 5
    # planner_options.n_circles_ego = 5
    # planner_options_list.append(planner_options)
    # planner_options_list_label.append("circles5x5")
    # #
    # planner_options = deepcopy(planner_options)
    # planner_options.obstacle_model = VehicleObstacleModel.CIRCLES
    # planner_options_list.append(planner_options)
    # planner_options_list_label.append("circles2x2")
    # #
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
    # #
    # # planner_options = deepcopy(planner_options)
    # # planner_options.obstacle_model = VehicleObstacleModel.PANOS
    # # planner_options_list.append(planner_options)
    # # planner_options_list_label.append("leuven")
    # #
    # planner_options = deepcopy(planner_options)
    # planner_options.obstacle_model = VehicleObstacleModel.HYPERPLANE
    # planner_options.contraints_slack_weight_sqr = 1e4
    # planner_options_list.append(planner_options)
    # planner_options_list_label.append("hyperplane")

    # Create test road
    road_options = RoadOptions()
    road_options.road_width = 12
    road_options.n_points = 400
    road_options.random_road_parameters.maximum_kappa = 1 / 100
    road = Road(road_options)
    road.randomize(seed=1)
    road.set_kappa(kappa_grid=road.kappa_grid_)# + (-1 / 50 + np.cumsum(np.ones_like(road.kappa_grid_)) / 15000))

    # Create planner
    planner_sets = []
    list_class_labels = []
    list_obstacle_labels = []

    for planner_class, label_class, do_lift in zip(planner_classes, planner_classes_labels, planner_related_lifting):
        for planner_opts_current_o, label_obstacle in zip(planner_options_list, planner_options_list_label):
            t_full, t_average, t_max, t_min, collisions_lead, collisions_follow, min_distances = \
                [], [], [], [], [], [], []
            t_qp, t_sim, qp_iter, s_final = [], [], [], []
            planner_opts_current = deepcopy(planner_opts_current_o)
            planner_opts_current.use_lifting = do_lift
            current_planner_set = []
            for i in range(n_vehicles):
                other_parameters = model_parameters[:i] + model_parameters[i + 1:]
                planner_opts_current = deepcopy(planner_opts_current)
                if i > 0:
                    planner_opts_current.deactivate_obstacle_avoidance = True
                current_planner_set.append(planner_class(ego_model_params=model_parameters[i],
                                                         road=road,
                                                         planner_options=planner_opts_current,
                                                         opp_model_params=other_parameters))

            for i_run in range(n_runs):
                # randomize road
                add = i_run
                road.randomize(seed=add)
                np.random.seed(seed=add)

                rand_velocity = np.random.uniform(range_velocity[0], range_velocity[1])
                initial_states = []
                actions = []
                # Set parameters
                initial_state_ego_f = np.array([1, 0., 0., rand_velocity - 5, 0])

                # Default actions
                action_ego = np.array([0., 50., 1e2])

                initial_states.append(initial_state_ego_f)
                actions.append(action_ego)
                for i in range(n_vehicles - 1):
                    s = 35. * i + 50.
                    n = -3.0 + float(np.mod(i, 2) * 6.0)
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
                s_final.append(simulator.current_states[0][0])

                if ANIMATE:
                    animator = Animator(animation_parameter=animation_parameter)
                    animator.set_data(planning_data_container=simulator.planning_containers,
                                      vehicle_parameter=model_parameters,
                                      planner_parameter=[planner_opts_current]*n_vehicles,
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
                                                collisions=collisions_lead + collisions_follow,
                                                qp_iter=np.mean(np.array(qp_iter)),
                                                sim_time=np.mean(np.array(t_sim)),
                                                qp_time=np.mean(np.array(t_qp)),
                                                s_final=np.mean(np.array(s_final))))

    table_output = []
    table_header = ["Class", "Obstacle", "Lifting", "t_ave", "t_min", "t_max", "collisions_mean","s_final" ,"min_dist", "t_sim", "t_qp", "qp_iter"]
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
