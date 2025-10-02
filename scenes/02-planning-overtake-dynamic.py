#!/usr/bin/env python3
#
# Copyright (c) 2020 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

from datetime import datetime
from environs import Env
import random
import lgsvl
import sys
import time
from utils import create_npc
import logging
logging.basicConfig(level=logging.INFO)
import math

'''
LGSVL__AUTOPILOT_0_HOST             IP address of the computer running the bridge to connect to
LGSVL__AUTOPILOT_0_PORT             Port that the bridge listens on for messages
LGSVL__AUTOPILOT_0_VEHICLE_CONFIG   Vehicle configuration to be loaded in Dreamview
LGSVL__AUTOPILOT_HD_MAP             HD map to be loaded in Dreamview
LGSVL__MAP                          ID of map to be loaded in Simulator
LGSVL__RANDOM_SEED                  Simulation random seed
LGSVL__SIMULATION_DURATION_SECS     How long to run the simulation for
LGSVL__SIMULATOR_HOST               IP address of computer running simulator
LGSVL__SIMULATOR_PORT               Port that the simulator allows websocket connections over
LGSVL__VEHICLE_0                    ID of EGO vehicle to be loaded in Simulator
'''

env = Env()

# 模拟器和桥接配置
SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "172.27.103.117")  # wsl2 hyper v ip
# BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "192.168.123.241")  # pandavan ip
BRIDGE_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)

# 仿真参数配置
LGSVL__SIMULATION_DURATION_SECS = 60.0
LGSVL__RANDOM_SEED = env.int("LGSVL__RANDOM_SEED", 51472)
OBSTACLE_DISTANCE = 50  # 障碍物在自车前方50m
OBSTACLE_WIDTH = 1.8    # 障碍物宽度（部分占用车道）
DETOUR_COMPLETE_DISTANCE = 30  # 绕行完成后距离障碍物的距离

# 车辆配置
# vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5)        # lgsvl输出lidar点云、radar感知结果
vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_modular)  # lgsvl输出障碍物信息
LGSVL__AUTOPILOT_0_VEHICLE_CONFIG = env.str("LGSVL__AUTOPILOT_0_VEHICLE_CONFIG", 'Lincoln2017MKZ')

# 强制使用straight2lanesame地图（双车道直道）
map_options = [{
    "hd_map": "Highway101GLE",
    "scene_id": "431292c2-f6f6-4f5a-ae62-0964f6018d20"
}]
selected_index = 0
LGSVL__AUTOPILOT_HD_MAP = env.str("LGSVL__AUTOPILOT_HD_MAP", map_options[selected_index]["hd_map"])
scene_name = env.str("LGSVL__MAP", map_options[selected_index]["scene_id"])


# 初始化模拟器
sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
# try:
#     print(f"Loading map {scene_name}...")
#     sim.load(scene_name, LGSVL__RANDOM_SEED)
# except Exception:
if sim.current_scene == scene_name:
    sim.reset()
else:
    sim.load(scene_name)


# 设置时间为白天
sim.set_date_time(datetime(2022, 6, 22, 13, 0, 0, 0), True)

# 获取 spawn 点并确保自车在右侧车道
spawns = sim.get_spawn()
# 对于straight2lanesame地图，选择右侧车道的spawn点（通常索引1为右侧车道）
spawn_index = 1 if len(spawns) > 1 else 0
print(f"Using spawn point {spawn_index} (right lane)")

# 配置自车状态（右侧车道）
ego_state = lgsvl.AgentState()
ego_state.transform = spawns[spawn_index]
print(f"Loading vehicle {vehicle_conf}...")
ego = sim.add_agent(vehicle_conf, lgsvl.AgentType.EGO, ego_state)

# 连接到Apollo桥接
print("Connecting to apollo cyber bridge...")
ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)
# 等待桥接成功
while not ego.bridge_connected:
    time.sleep(1)
print("Connected to bridge successfully!!!")

# 碰撞检测回调
def on_collision(agent1, agent2, contact):
    print(f"\nCollision occurred: {agent1} collided with {agent2}")
    sys.exit(0)

ego.on_collision(on_collision)

# 配置Dreamview
dv = lgsvl.dreamview.Connection(sim, ego, BRIDGE_HOST)
dv.set_hd_map(LGSVL__AUTOPILOT_HD_MAP)
dv.set_vehicle(LGSVL__AUTOPILOT_0_VEHICLE_CONFIG)

# 启动Apollo模块
default_modules = [
    'Localization',
    'Transform',
    'Routing',
    'Prediction',
    'Planning',
    'Control',
    'Recorder'
]

current_pos = dv.ego.state.transform
current_gps = dv.sim.map_to_gps(current_pos)
heading = math.radians(current_gps.orientation)

# Start position should be the position of the GPS
# Unity returns the position of the center of the vehicle so adjustment is required
northing_adjustment = (
    math.sin(heading) * dv.gps_offset.z - math.cos(heading) * dv.gps_offset.x
)
easting_adjustment = (
    math.cos(heading) * dv.gps_offset.z + math.sin(heading) * dv.gps_offset.x
)

dv.disable_apollo()
# 设置远处的目的地，确保自车直行
forward = lgsvl.utils.transform_to_forward(ego_state.transform)
right = lgsvl.utils.transform_to_right(ego.transform)
destination = ego_state.position + 2000 * forward  # 自车前方200m作为目的地
# dv.setup_apollo(current_gps.easting + easting_adjustment, current_gps.northing + northing_adjustment, default_modules, default_timeout=600.0)
dv.setup_apollo(destination.x, destination.z, default_modules, default_timeout=600.0) # 目的地坐标必须是地图坐标系，已经设置了终点，但是发请求的时候，routing还没好

# time.sleep(5)  # 等待apollo模块启动
dv.set_destination(ego_state.position.x, ego_state.position.z)
dv.set_destination(destination.x, destination.z)

# 在车道添加障碍物
print("Adding obstacle ...")
obstacle1 = create_npc(sim, ego_state, 50, -5.5, 'Sedan', 0.0)

'''
让障碍物动态行驶的两种方法:
1. 使用follow_closest_lane方法, 设置速度和是否跟随车道线, 理论上有碰撞检测
2. 使用follow_waypoints方法, 设置一系列目标点和速度, 没有碰撞检测
'''
obstacle1.follow_closest_lane(True, 3.0, False) # 3.0是npc的车速，False是指不变道

waypoints = []
layer_mask = 0
z_delta = 40
layer_mask |= 1 << 0  # 0 is the layer for the road (default)
obstacle2 = create_npc(sim, ego_state, 25, -3.6, 'SUV', 0.0)
for i in range(1, 20):
    speed = 4.0
    pz = (i + 1) * z_delta
    # Waypoint angles are input as Euler angles (roll, pitch, yaw)
    angle = spawns[0].rotation
    # Raycast the points onto the ground because BorregasAve is not flat
    hit = sim.raycast(
        obstacle2.transform.position + (i != 1) * pz * forward, lgsvl.Vector(0, -1, 0), layer_mask
    )

    # NPC will wait for 0 second at each waypoint
    # print(f"Waypoint {i}: pz={pz}, hit={hit}")
    if hit is None:
        continue  # 跳过无效的点
    wp = lgsvl.DriveWaypoint(hit.point, speed, angle=angle, idle=0)
    waypoints.append(wp)
obstacle2.follow(waypoints, True, waypoints_path_type = 'Linear')  # 3.0是npc的车速，False是指不变道
obstacle1.on_collision(on_collision)
obstacle2.on_collision(on_collision)

# 运行仿真（小步推进，支持Ctrl+C退出）
print("Simulation running...")
duration = LGSVL__SIMULATION_DURATION_SECS
t0 = 0.0
dt = 0.5  # 每次推进0.5秒
detour_completed = False

while t0 < duration:
    sim.run(dt)
    t0 += dt

    if not detour_completed:
        ego_pos = ego.state.position
        obstacle_pos = obstacle1.state.position
        distance = (ego_pos - obstacle_pos).magnitude()
        if distance > DETOUR_COMPLETE_DISTANCE:
            detour_completed = True
else:
    print("Simulation completed.")