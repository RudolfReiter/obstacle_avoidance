from dataclasses import dataclass
from typing import List

from vehiclegym.trajectory_planner_base import VehicleObstacleModel


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
    s_final_all: List


@dataclass
class PlannerOption:
    str_obstacle: str
    obstacle: VehicleObstacleModel
    lifting: bool
    weight_obstacle: float
    circles: int