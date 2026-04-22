"""
VFH / VFH* 局部导航辅助函数。

本文件中的函数在 `aaaVFH.py` 中按如下顺序被调用，用于把激光雷达数据转成可执行的局部转向决策：

1. `calcDanger`:
   把激光距离转换为危险度，距离越近危险度越高。
2. `calc_h`:
   将逐束激光的危险度按固定扇区做聚合，得到扇区直方图。
3. `calc_hp`:
   对扇区直方图做平滑，减弱单个测距噪声带来的抖动。
4. `calc_Hb`:
   将连续直方图二值化，标记哪些扇区被视为障碍。
5. `find_valleys` / `pick_heading`:
   在二值直方图中寻找可通行波谷，并选出候选航向。
6. `vfh_star_full`:
   基于候选航向做简化的 VFH* 前瞻搜索，返回根节点应采取的转向。

这里保留了原始实现的计算方式，当前任务仅补充注释，不改变行为。
"""

import numpy as np
import math


def calcDanger(lidar, maxRange):
    """
    将激光雷达距离映射为危险度数组。

    在 `aaaVFH.py` 中，这一步是 VFH 处理链的入口。距离越近，返回的危险度越大；
    超出感知上限较多的测量值会被视为无障碍，对应危险度置 0。

    Args:
        lidar: 激光雷达距离序列。
        maxRange: 认为有效的最大测距距离。

    Returns:
        numpy.ndarray: 与输入长度一致的危险度数组。
    """
    lidar = np.array(lidar)
    a = maxRange
    b = 1
    m = np.where(lidar >= maxRange + 0.5, 0, a - lidar * b)
    return m


def calc_h(m, sector_size):
    """
    按扇区聚合危险度，生成一维扇区直方图。

    `aaaVFH.py` 中将连续激光束分组，每 `sector_size` 个测距点组成一个扇区，
    用该扇区的平均危险度表示当前方向上的障碍强度。

    Args:
        m: `calcDanger` 输出的危险度数组。
        sector_size: 每个扇区包含的激光束数量。

    Returns:
        numpy.ndarray: 扇区危险度直方图。
    """
    m = np.array(m)
    sectors = len(m) // sector_size
    h = np.zeros(sectors)
    for i in range(sectors):
        start = i * sector_size
        block = m[start:start + sector_size]
        h[i] = np.sum(block) / sector_size
    return h


def calc_hp(h, L):
    """
    对扇区直方图做局部加权平滑。

    这一步对应经典 VFH 中的平滑直方图，用邻域扇区的加权和降低噪声影响，
    使后续的障碍/可通行区域判定更稳定。

    Args:
        h: 原始扇区直方图。
        L: 平滑窗口半宽。

    Returns:
        numpy.ndarray: 平滑后的扇区直方图。
    """
    h = np.array(h)
    h_length = len(h)
    h_padded = np.concatenate((np.zeros(L), h, np.zeros(L)))
    hp_sum = np.zeros(h_length)
    # 构造对称权重，中心附近权重大，两侧逐步减小。
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
    """
    将连续直方图二值化为障碍直方图。

    返回值中 `True` 表示该扇区被判定为障碍，`False` 表示该扇区可以作为候选通行区域。
    在 `aaaVFH.py` 中，调用方会把该结果继续传给 `find_valleys` 搜索波谷。

    Args:
        hp: 需要阈值化的扇区直方图，通常是平滑结果，也可以由调用方传入其他直方图。
        threshold: 基础阈值。

    Returns:
        numpy.ndarray: 布尔数组形式的二值障碍直方图。
    """
    Hb = hp > (threshold + 2)
    return Hb


def find_valleys(Hb, threshold, robotW):
    """
    在二值障碍直方图中寻找满足车体宽度要求的可通行波谷。

    波谷可以理解为连续的“非障碍扇区”区间。函数会先根据机器人宽度和阈值
    估算至少需要多少个连续扇区才能安全通过，再过滤掉过窄的波谷。

    Args:
        Hb: 二值障碍直方图，`True` 表示障碍，`False` 表示空闲。
        threshold: 用于估算安全通行角度的距离阈值。
        robotW: 机器人宽度。

    Returns:
        tuple:
        - passable: 形状为 `(N, 2)` 的数组，每行表示一个可通行波谷的起止扇区。
        - sectToClear: 至少需要连续空闲的扇区数。
    """
    clearance_angle = 2 * np.arcsin(robotW / (2 * (threshold))) * 180 / np.pi
    sectToClear = int(np.floor(clearance_angle / 3))
    diff_Hb = np.diff(Hb.astype(int))
    # `b1` 为波谷起点，`b2` 为波谷终点。
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
    """
    将全局目标点映射为局部 VFH 扇区编号。

    `aaaVFH.py` 中先通过路径跟踪得到一个前视目标点，再调用本函数把该点相对机器人
    当前位姿的方向，转换成 VFH 使用的扇区编号。最终返回值被后续波谷选择和 VFH* 搜索复用。

    Args:
        targetPos: 目标点世界坐标 `(x, y)`。
        currentPos: 机器人当前世界坐标 `(x, y)`。
        currentHeading: 当前航向角，单位为度。

    Returns:
        int: 映射后的目标扇区编号，范围约为 1 到 90。
    """
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
    """
    选择与目标方向最接近的可通行波谷。

    如果有多个波谷到目标扇区的距离相同，则优先返回更宽的波谷。
    这个函数体现了 VFH 中“先找可通行区域，再选最接近目标的区域”的思想。

    Args:
        valleyArs: 可通行波谷数组，每行是 `[start_sector, end_sector]`。
        T_heading: 目标扇区编号。

    Returns:
        numpy.ndarray | None: 选中的波谷区间；若没有可选波谷则返回 `None`。
    """
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
    """
    在选定波谷内部生成最终航向扇区。

    对窄波谷，直接取中间扇区以保证穿越稳定性；对宽波谷，允许在留出缓冲区后，
    尽量选择更接近目标方向 `Th` 的扇区，从而兼顾避障与朝向目标。

    Args:
        Th: 目标扇区编号。
        chosenValley: 当前波谷的起止扇区。
        wideValleyMin: 判定为“宽波谷”的最小宽度。
        sectToClear: 机器人安全通过所需的最小连续扇区数。

    Returns:
        int: 选出的航向扇区编号。
    """
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
    依据候选航向做一步简化前向投影。

    这是 `vfh_star_full` 中的状态扩展函数。当前实现采用非常轻量的运动学近似：
    从当前位置沿 `candidateHeading` 方向前进 `ds` 距离，并把该候选航向视为新的朝向。

    注意：这里延续原实现，直接把 `candidateHeading` 当作角度值参与投影，
    用于快速比较候选轨迹之间的相对优劣，而不是精确的车辆运动模型。

    Args:
        currentPos: 当前坐标 `(x, y)`。
        currentHeading: 当前朝向，保留为接口参数，便于后续替换成更完整模型。
        candidateHeading: 候选航向。
        ds: 单步投影距离。

    Returns:
        tuple:
        - projected_pos: 投影后的坐标 `(x, y)`。
        - new_heading: 投影后的朝向。
    """
    candidate_rad = math.radians(candidateHeading)
    new_x = currentPos[0] + ds * math.cos(candidate_rad)
    new_y = currentPos[1] + ds * math.sin(candidate_rad)
    new_heading = candidateHeading  # simplified assumption
    return (new_x, new_y), new_heading


def cost_trajectory(currentPos, projectedPos, target_sector, candidate_heading, prev_heading=None):
    """
    计算候选轨迹的代价，用于 VFH* 搜索排序。

    代价由三部分组成：
    1. 前进一步的运动代价；
    2. 当前候选航向偏离目标扇区的惩罚；
    3. 与上一时刻航向差异带来的切换惩罚（可选）。

    在 `aaaVFH.py` 中，这个切换项用于抑制转向抖动，让航向变化更平滑。

    Args:
        currentPos: 当前坐标。
        projectedPos: 候选投影后的坐标。
        target_sector: 目标扇区编号。
        candidate_heading: 当前候选航向。
        prev_heading: 上一时刻选中的航向，用于增加切换惩罚。

    Returns:
        float: 该候选轨迹的累计步进代价。
    """
    progress_cost = math.hypot(projectedPos[0] - currentPos[0], projectedPos[1] - currentPos[1])
    heading_deviation = abs(candidate_heading - target_sector)
    total_cost = progress_cost + 0.1 * heading_deviation
    if prev_heading is not None:
        switching_penalty = 0.2 * abs(candidate_heading - prev_heading)
        total_cost += switching_penalty
    return total_cost


class Node:
    """
    VFH* 搜索树中的节点。

    Attributes:
        position: 当前投影位置。
        heading: 当前节点对应的航向。
        g_cost: 从根节点累计到当前节点的真实代价。
        depth: 当前搜索深度。
        action: 根节点展开时选择的首个候选航向。
        f_cost: A* 使用的综合代价，通常为 `g + h`。
    """

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
    """
    根据当前二值直方图生成一组候选航向。

    函数先调用 `find_valleys` 找到所有可通行波谷，再对每个波谷调用 `pick_heading`
    生成代表性的候选扇区。如果没有任何可通行波谷，则退化为直接朝目标扇区尝试。

    Args:
        hb: 二值障碍直方图。
        heading_sector: 目标扇区编号。
        threshold: 安全距离阈值。
        robotDim: 机器人宽度。
        wideValleyMin: 宽波谷判定阈值。

    Returns:
        list[int]: 候选航向扇区列表。
    """
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
    执行完整的简化版 VFH* 前瞻搜索，返回根节点应采取的航向。

    搜索流程如下：
    1. 以当前位姿作为根节点；
    2. 在每一层根据当前直方图生成候选航向；
    3. 用 `project_trajectory` 做一步前向投影；
    4. 用 `cost_trajectory` 和启发式代价对节点排序；
    5. 搜索到深度 `ng` 后，选择累计代价最低的叶节点，其根动作即为最终输出。

    `aaaVFH.py` 中通过该函数把单步 VFH 的“局部可行航向”扩展为带前瞻的航向选择。

    Args:
        currentPos: 当前坐标。
        currentHeading: 当前朝向。
        heading_sector: 目标扇区编号。
        ds: 每次前向投影的步长。
        ng: 前瞻搜索深度。
        hb: 二值障碍直方图。
        threshold: 安全距离阈值。
        robotDim: 机器人宽度。
        wideValleyMin: 宽波谷判定阈值。
        prev_heading: 上一时刻航向，用于切换惩罚。

    Returns:
        int: 代价最低的根节点候选航向；若搜索失败，则退回目标扇区。
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
            # 启发式项鼓励在剩余步数内继续向目标方向靠近。
            h_cost = (ng - new_depth) * ds + 0.1 * abs(cand - heading_sector)
            new_f = new_g + h_cost
            new_node = Node(proj_pos, proj_heading, new_g, new_depth, action)
            new_node.f_cost = new_f
            heapq.heappush(open_list, (new_node.f_cost, new_node))
    if best_node is not None:
        return best_node.action
    else:
        return heading_sector

