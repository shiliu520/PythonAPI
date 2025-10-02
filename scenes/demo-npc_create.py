#!/usr/bin/env python3
#
# Copyright (c) 2019-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import math
import random
from environs import Env
import lgsvl

print("Python API Quickstart #12: Creating NPCs on lanes")
env = Env()

sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))
if sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
    sim.reset()
else:
    sim.load(lgsvl.wise.DefaultAssets.map_borregasave)

spawns = sim.get_spawn()

ego_state = lgsvl.AgentState()
ego_state.transform = spawns[0]
sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_modular), lgsvl.AgentType.EGO, ego_state)

sx = spawns[0].position.x
sy = spawns[0].position.y
sz = spawns[0].position.z

mindist = 10.0
maxdist = 40.0

random.seed(0)

available_npcs = ['Sedan', 'SUV', 'Jeep', 'Hatchback']  # 'SchoolBus', 'DeliveryTruck'
def create_static_npc(sim, ego_state, lon_dist, lat_dist, npc_type = "Sedan"):
    forward = lgsvl.utils.transform_to_forward(ego_state.transform)
    right = lgsvl.utils.transform_to_right(ego_state.transform)
    up = lgsvl.utils.transform_to_up(ego_state.transform)

    obs_state = lgsvl.AgentState()
    # 计算NPC的位置（自车前方30米）
    desired_position = ego_state.transform.position + lon_dist * forward + lat_dist * right  # 右侧车道内

    # 设置碰撞检测层掩码（排除自车层）
    layer_mask = 0
    for bit in [0, 10, 11, 12]:  # 不包含9，避免检测到自车本身
        layer_mask |= 1 << bit

    # 从期望位置向下发射射线检测地面
    hit = sim.raycast(desired_position, -up, layer_mask)

    # 将NPC位置设置为地面接触点
    obs_state.transform.position = hit.point
    obs_state.transform.rotation= ego_state.transform.rotation

    # 设置NPC速度
    obs_state.velocity = 0.0 * forward

    # 添加NPC并设置其行为
    npc = sim.add_agent(npc_type, lgsvl.AgentType.NPC, obs_state)

lon_dist, lat_dist = 20, 3.6
create_static_npc(sim, ego_state, lon_dist, lat_dist, 'Sedan')
create_static_npc(sim, ego_state, 0, lat_dist, 'Jeep')
create_static_npc(sim, ego_state, 0, -lat_dist, 'SchoolBus')

# npc1.follow_closest_lane(False, 0)
sim.run(time_limit = 0)