import lgsvl

def create_npc(sim, ego_state, lon_dist, lat_dist, npc_type = "Sedan", vel = 0.0):
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
    obs_state.velocity = lgsvl.Vector(vel, 0, 0)
    # obs_state.velocity = 0.0 * forward
    # print(type(obs_state.velocity), obs_state.velocity)
    # print(type(lgsvl.Vector(vel, 0, 0)), lgsvl.Vector(vel, 0, 0))

    # 添加NPC并设置其行为
    npc = sim.add_agent(npc_type, lgsvl.AgentType.NPC, obs_state)
    return npc