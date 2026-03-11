import numpy as np
import math

def calcDanger(lidar, maxRange):
    lidar = np.array(lidar)
    a = maxRange
    b = 1
    m = np.where(lidar >= maxRange + 0.5, 0, a - lidar * b)
    return m

def calc_h(m, sector_size):
    m = np.array(m)
    sectors = len(m) // sector_size
    h = np.zeros(sectors)
    for i in range(sectors):
        start = i * sector_size
        block = m[start:start + sector_size]
        h[i] = np.sum(block) / sector_size
    return h

def calc_hp(h, L):
    h = np.array(h)
    h_length = len(h)
    h_padded = np.concatenate((np.zeros(L), h, np.zeros(L)))
    hp_sum = np.zeros(h_length)
    weightArray = np.concatenate((np.arange(1, L+1), np.arange(L, 0, -1)))
    div = np.ones(h_length) * (2 * L + 1)
    zeros_length = h_length - (2 * L)
    zeros_length = zeros_length if zeros_length > 0 else 0
    div_mod = np.concatenate((np.arange(L, 0, -1), np.zeros(zeros_length), np.arange(1, L+1)))
    div = div - div_mod
    for i in range(L, h_length + L):
        block = h_padded[i - L : i + L]
        hp_sum[i - L] = np.sum(block * weightArray)
    hp = hp_sum / div
    return hp

def calc_Hb(hp, threshold):
    Hb = hp > (threshold + 2)
    return Hb

def find_valleys(Hb, threshold, robotW):
    clearance_angle = 2 * np.arcsin(robotW / (2 * (threshold))) * 180 / np.pi
    sectToClear = int(np.floor(clearance_angle / 3))
    diff_Hb = np.diff(Hb.astype(int))
    b1 = np.where(diff_Hb == -1)[0] + 2
    b2 = np.where(diff_Hb == 1)[0] + 1
    if Hb[0] == 0:
        b1 = np.insert(b1, 0, 0)
    if Hb[-1] == 0:
        b2 = np.append(b2, len(Hb))
    passable = np.vstack((b1, b2))
    valley_width = passable[1, :] - passable[0, :] + 1
    valid = valley_width >= sectToClear
    passable = passable[:, valid]
    passable = passable.T
    return passable, sectToClear

def calc_Target(targetPos, currentPos, currentHeading):
    tX = targetPos[0]
    tY = targetPos[1]
    cX = currentPos[0]
    cY = currentPos[1]
    currentHeading = (currentHeading + 360) % 360
    dX = tX - cX
    dY = tY - cY
    if dX == 0:
        theta = 90.0
    else:
        theta = abs(math.degrees(math.atan(dY / dX)))
    if dX >= 0 and dY >= 0:
        Th = currentHeading - theta
        if Th > 180:
            Th = Th - 360
    elif dX < 0 and dY >= 0:
        Th = currentHeading - (180 - theta)
        if Th > 180:
            Th = Th - 360
    elif dY < 0 and dX <= 0:
        Th = currentHeading - (180 + theta)
        if Th < -180:
            Th = Th + 360
    elif dY < 0 and dX >= 0:
        Th = -(360 - currentHeading - theta)
        if Th <= -180:
            Th = Th + 360
    else:
        Th = currentHeading
    if Th < -135:
        Th = -135
    elif Th > 135:
        Th = 135
    Th_s = round(((Th + 135) * 89 / 270) + 1)
    return Th_s

def pick_valley(valleyArs, T_heading):
    if valleyArs is None or len(valleyArs) == 0:
        return None
    dist = np.abs(valleyArs - T_heading)
    min_dist = np.min(dist)
    rows, _ = np.where(dist == min_dist)
    candidate_rows = np.unique(rows)
    if candidate_rows.size == 0:
        return None
    chosen_valleys = valleyArs[candidate_rows, :]
    if chosen_valleys.shape[0] == 1:
        return chosen_valleys[0, :]
    if np.all(chosen_valleys == chosen_valleys[0, :]):
        return chosen_valleys[0, :]
    widths = chosen_valleys[:, 1] - chosen_valleys[:, 0]
    idx = np.argmax(widths)
    return chosen_valleys[idx, :]

def pick_heading(Th, chosenValley, wideValleyMin, sectToClear):
    buffer = math.ceil(wideValleyMin / 2) + 1
    chosenValley = np.array(chosenValley, dtype=float)
    valley_width = chosenValley[1] - chosenValley[0]
    if valley_width <= wideValleyMin:
        heading = round(np.mean(chosenValley))
    else:
        adjusted_valley = chosenValley.copy()
        adjusted_valley[0] += buffer
        adjusted_valley[1] -= buffer
        if Th < adjusted_valley[0]:
            heading = adjusted_valley[0]
        elif Th > adjusted_valley[1]:
            heading = adjusted_valley[1]
        else:
            heading = Th
        heading = round(heading)
    return heading

def project_trajectory(currentPos, currentHeading, candidateHeading, ds):
    """
    Projects the robot's position forward by distance ds.
    This simplified model assumes a short motion toward candidateHeading.
    """
    candidate_rad = math.radians(candidateHeading)
    new_x = currentPos[0] + ds * math.cos(candidate_rad)
    new_y = currentPos[1] + ds * math.sin(candidate_rad)
    new_heading = candidateHeading  # simplified assumption
    return (new_x, new_y), new_heading

def cost_trajectory(currentPos, projectedPos, target_sector, candidate_heading, prev_heading=None):
    """
    Computes a cost for the projected trajectory.
    Combines the progress cost (distance moved), a penalty for heading deviation from target_sector,
    and, if provided, a switching penalty for deviating from the previous heading.
    """
    progress_cost = math.hypot(projectedPos[0] - currentPos[0], projectedPos[1] - currentPos[1])
    heading_deviation = abs(candidate_heading - target_sector)
    total_cost = progress_cost + 0.1 * heading_deviation
    if prev_heading is not None:
        switching_penalty = 0.2 * abs(candidate_heading - prev_heading)
        total_cost += switching_penalty
    return total_cost

class Node:
    def __init__(self, position, heading, g_cost, depth, action):
        self.position = position
        self.heading = heading
        self.g_cost = g_cost
        self.depth = depth
        self.action = action  # the candidate heading taken at the root
        self.f_cost = 0
    def __lt__(self, other):
        return self.f_cost < other.f_cost

def generate_candidate_headings(hb, heading_sector, threshold, robotDim, wideValleyMin):
    valleys, sectToClear = find_valleys(hb, threshold, robotDim)
    candidates = []
    if valleys.size == 0:
        return [heading_sector]
    for valley in valleys:
        cand = pick_heading(heading_sector, valley, wideValleyMin, sectToClear)
        candidates.append(cand)
    return list(set(candidates))

def vfh_star_full(currentPos, currentHeading, heading_sector, ds, ng, hb, threshold, robotDim, wideValleyMin, prev_heading):
    """
    Full VFH* search: expands candidate trajectories up to depth ng.
    The cost function now includes a switching penalty based on the difference
    between the candidate heading and prev_heading.
    Returns the candidate heading from the root that leads to the lowest total cost.
    """
    import heapq
    open_list = []
    root = Node(currentPos, currentHeading, 0, 0, action=None)
    root.f_cost = 0
    heapq.heappush(open_list, (root.f_cost, root))
    best_node = None

    while open_list:
        f, node = heapq.heappop(open_list)
        if node.depth == ng:
            if best_node is None or node.g_cost < best_node.g_cost:
                best_node = node
            continue
        candidates = generate_candidate_headings(hb, heading_sector, threshold, robotDim, wideValleyMin)
        for cand in candidates:
            proj_pos, proj_heading = project_trajectory(node.position, node.heading, cand, ds)
            cost = cost_trajectory(node.position, proj_pos, heading_sector, cand, prev_heading)
            new_g = node.g_cost + cost
            new_depth = node.depth + 1
            if node.depth == 0:
                action = cand
            else:
                action = node.action
            h_cost = (ng - new_depth) * ds + 0.1 * abs(cand - heading_sector)
            new_f = new_g + h_cost
            new_node = Node(proj_pos, proj_heading, new_g, new_depth, action)
            new_node.f_cost = new_f
            heapq.heappush(open_list, (new_node.f_cost, new_node))
    if best_node is not None:
        return best_node.action
    else:
        return heading_sector


