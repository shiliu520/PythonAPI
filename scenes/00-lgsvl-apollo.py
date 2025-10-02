#!/usr/bin/env python3
#
# Copyright (c) 2020 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

from datetime import datetime
from environs import Env
import random
import lgsvl, sys
import logging
logging.basicConfig(level=logging.INFO)


'''
LGSVL__AUTOPILOT_0_HOST             IP address of the computer running the bridge to connect to
LGSVL__AUTOPILOT_0_PORT             Port that the bridge listens on for messages
LGSVL__AUTOPILOT_0_VEHICLE_CONFIG   Vehicle configuration to be loaded in Dreamview (Capitalization and spacing must match the dropdown in Dreamview)
LGSVL__AUTOPILOT_HD_MAP             HD map to be loaded in Dreamview (Capitalization and spacing must match the dropdown in Dreamview)
LGSVL__MAP                          ID of map to be loaded in Simulator
LGSVL__RANDOM_SEED                  Simulation random seed
LGSVL__SIMULATION_DURATION_SECS     How long to run the simulation for
LGSVL__SIMULATOR_HOST               IP address of computer running simulator (Master node if a cluster)
LGSVL__SIMULATOR_PORT               Port that the simulator allows websocket connections over
LGSVL__VEHICLE_0                    ID of EGO vehicle to be loaded in Simulator
'''

env = Env()

# lgsvl模拟器端口为8181，与apollo桥接端口为9090
SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
# BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "172.27.103.117") # wsl2 hyper v ip
# BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "192.168.123.241")  # pandavan ip
BRIDGE_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)

# 选取BorregasAve地图，车辆Lincoln2017MKZ选择Lincoln2017MKZ，仿真时间为1800s，种子为51472(随机天气)
LGSVL__SIMULATION_DURATION_SECS = 60.0
LGSVL__RANDOM_SEED = env.int("LGSVL__RANDOM_SEED", 51472)

# 读取车辆配置，加载仿真环境
vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_modular)
LGSVL__AUTOPILOT_0_VEHICLE_CONFIG = env.str("LGSVL__AUTOPILOT_0_VEHICLE_CONFIG", 'Lincoln2017MKZ')

# 地图选择
# 定义数字对应的地图配置列表
# 顺序对应: 1-BorregasAve, 2-Shalun, 3-SanFrancisco, 4-AutonomouStuff
map_options = [
    {
        "hd_map": "BorregasAve",
        "scene_id": lgsvl.wise.DefaultAssets.map_borregasave
    },
    {
        "hd_map": "Shalun",
        "scene_id": "97128028-33c7-4411-b1ec-d693ed35071f"
    },
    {
        "hd_map": "SanFrancisco",
        "scene_id": "12da60a7-2fc9-474d-a62a-5cc08cb97fe8"
    },
    {
        "hd_map": "AutonomouStuff",
        "scene_id": "2aae5d39-a11c-4516-87c4-cdc9ca784551"
    },
    {
        "hd_map": "Highway101GLE",
        "scene_id": "431292c2-f6f6-4f5a-ae62-0964f6018d20"
    },
    {
        "hd_map": "WideFlatMap",
        "scene_id": "e6dc21da-0105-4534-a30d-c1939a8a4ff6"
    },
    {
        "hd_map": "GoMentum",
        "scene_id": "979dd7f3-b25b-47f0-ab10-a6effb370138"
    },
    {
        "hd_map": "GoMentumDTL",
        "scene_id": "d7b4b7b4-ce9d-4670-8c41-917eb7bc15d6"
    },
    {
        'hd_map': "IndianapolisMotorSpeedway",
        'scene_id': "62765742-57bf-4ccd-85e5-db8295d34ead"
    }
]

# 2. 设置切换标志（1-4），对应上面的地图顺序
selected_flag = 1  # 这里修改1-4之间的数字即可切换地图

# 验证flag有效性并处理
if not (1 <= selected_flag <= len(map_options)):
    print(f"无效的flag值 {selected_flag}，将使用默认地图（1-BorregasAve）")
    selected_flag = 1

# 应用选中的地图配置（注意列表索引从0开始，需要-1转换）
selected_index = selected_flag - 1
LGSVL__AUTOPILOT_HD_MAP = env.str("LGSVL__AUTOPILOT_HD_MAP", map_options[selected_index]["hd_map"])
scene_name = env.str("LGSVL__MAP", map_options[selected_index]["scene_id"])

sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
try:
    print("Loading map {}...".format(scene_name))
    sim.load(scene_name, LGSVL__RANDOM_SEED) # laod map with random seed
except Exception:
    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)


# 重设时间
sim.set_date_time(datetime(2022, 6, 22, 13, 0, 0, 0), True)

# 获取车辆位置spawns
spawns = sim.get_spawn()
spawn_index = LGSVL__RANDOM_SEED % len(spawns)

# 加载自车
state = lgsvl.AgentState()
state.transform = spawns[spawn_index]  # TODO some sort of Env Variable so that user/wise can select from list
print("Loading vehicle {}...".format(vehicle_conf))
ego = sim.add_agent(vehicle_conf, lgsvl.AgentType.EGO, state)

# 桥接
print("Connecting to apollo cyber bridge...")
# The EGO is now looking for a bridge at the specified IP and port
ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)
print("Connecting to bridge successed!!!")

# 禁用红绿灯和 stop sign
print("Disabling traffic lights and stop signs...")
print("Total signals: {}".format(len(sim.get_controllables("signal"))))
# for signal in sim.get_controllables("signal"):
#     # signal.control("green")   # 永远绿灯
#     control_policy = "trigger=50;green=30;yellow=2;red=15;loop"
#     # Control this traffic light with a new control policy
#     signal.control(control_policy)

# for stop in sim.get_controllables("stop_sign"):
#     stop.control("off")       # 关闭 StopSign

# 碰撞检测
def on_collision(agent1, agent2, contact):
    raise Exception("{} collided with {}".format(agent1, agent2))
    sys.exit(1)

ego.on_collision(on_collision)

# 仿真器连接dreamview，设置高精度地图，设置车辆
dv = lgsvl.dreamview.Connection(sim, ego, BRIDGE_HOST)
dv.set_hd_map(LGSVL__AUTOPILOT_HD_MAP)
dv.set_vehicle(LGSVL__AUTOPILOT_0_VEHICLE_CONFIG)

# 默认apollo打开模块
default_modules = [
    'Localization',
    'Transform',
    'Routing',
    'Prediction',
    'Planning',
    'Control',
    'Recorder'
]

dv.disable_apollo()
# 设置终点
if len(spawns[spawn_index].destinations) > 0:
    destination_index = LGSVL__RANDOM_SEED % len(spawns[spawn_index].destinations)
    destination = spawns[spawn_index].destinations[destination_index] # TODO some sort of Env Variable so that user/wise can select from list
    dv.setup_apollo(destination.position.x, destination.position.z, default_modules, default_timeout=600.0)

# 设置远处的目的地，确保自车直行
forward = lgsvl.utils.transform_to_forward(state.transform)
right = lgsvl.utils.transform_to_right(ego.transform)
destination = state.position + 200 * forward  # 自车前方200m作为目的地
dv.set_destination(state.position.x, state.position.z)
dv.set_destination(destination.x, destination.z)

# 设置随机NPC和行人
print("adding npcs")
sim.add_random_agents(lgsvl.AgentType.NPC)
# sim.add_random_agents(lgsvl.AgentType.PEDESTRIAN)

# 运行模拟器
# 用小步推进仿真，支持 Ctrl+C 退出
print("Simulation running...")
duration = LGSVL__SIMULATION_DURATION_SECS
t0 = 0.0
dt = 0.5  # 每次推进 0.5s，可以改大或改小

try:
    while t0 < duration:
        sim.run(dt)
        t0 += dt
except KeyboardInterrupt:
    print("\nSimulation stopped by user")
    sys.exit(0)
