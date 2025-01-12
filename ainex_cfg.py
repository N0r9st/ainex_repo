# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for a simple Cartpole robot."""


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

joint_names = [
    'head_pan',
    'l_hip_yaw',
    'l_sho_pitch',
    'r_hip_yaw',
    'r_sho_pitch',
    'head_tilt',
    'l_hip_roll',
    'l_sho_roll',
    'r_hip_roll',
    'r_sho_roll',
    'l_hip_pitch',
    'l_el_pitch',
    'r_hip_pitch',
    'r_el_pitch',
    'l_knee',
    'l_el_yaw',
    'r_knee',
    'r_el_yaw',
    'l_ank_pitch',
    'l_gripper',
    'r_ank_pitch',
    'r_gripper',
    'l_ank_roll',
    'r_ank_roll'
]

joint_pos = {
    'head_pan': 0.,
    'l_hip_yaw': 0.,
    'l_sho_pitch': 0.,
    'r_hip_yaw': 0.,
    'r_sho_pitch': 0.,
    'head_tilt': 0.,
    'l_hip_roll': 0.,
    'l_sho_roll': 0.,
    'r_hip_roll': 0.,
    'r_sho_roll': 0.,
    'l_hip_pitch': 0.,
    'l_el_pitch': 0.,
    'r_hip_pitch': 0.,
    'r_el_pitch': 0.,
    'l_knee': 0.,
    'l_el_yaw': 0.,
    'r_knee': 0.,
    'r_el_yaw': 0.,
    'l_ank_pitch': 0.,
    'l_gripper': 0.,
    'r_ank_pitch': 0.,
    'r_gripper': 0.,
    'l_ank_roll': 0.,
    'r_ank_roll': 0.,
}

actuators = {}
for elem in joint_names:
    actuators[elem + '_actuator'] = ImplicitActuatorCfg(
            joint_names_expr=[elem],
            effort_limit=100.0,
            velocity_limit=4.0,
            stiffness=40.0,
            damping=10.0,
        )


AINEX_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"./ainex.usd",
        # rigid_props=sim_utils.RigidBodyPropertiesCfg(
        #     rigid_body_enabled=True,
        #     max_linear_velocity=1000.0,
        #     max_angular_velocity=1000.0,
        #     max_depenetration_velocity=100.0,
        #     enable_gyroscopic_forces=True,
        # ),
        # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        #     enabled_self_collisions=False,
        #     solver_position_iteration_count=4,
        #     solver_velocity_iteration_count=0,
        #     sleep_threshold=0.005,
        #     stabilization_threshold=0.001,
        # ),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, .248),
        joint_pos=joint_pos
    ),
    actuators=actuators,
)
"""Configuration for a simple Cartpole robot."""
