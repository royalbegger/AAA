from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment
from navigation_pkg.kalman_filter import KalmanTrackerCA


@dataclass
class TrackerUpdateResult:
    """保存单帧多目标跟踪更新后的匹配结果。"""

    detection_to_track: Dict[int, int] = field(default_factory=dict)
    matched_track_ids: Set[int] = field(default_factory=set)
    new_track_ids: Set[int] = field(default_factory=set)
    unmatched_track_ids: Set[int] = field(default_factory=set)
    removed_track_ids: Set[int] = field(default_factory=set)
    tracker_states: Dict[int, np.ndarray] = field(default_factory=dict)


class MultiObjectTracker:

    def __init__(self):

        self.trackers = {}
        self.next_id = 0

        self.max_missed = 10
        self.match_threshold = 1.5
        self.last_update_result = TrackerUpdateResult()

    def update(self, detections):
        """使用当前帧检测结果更新全部轨迹，并返回匹配信息。"""
        detections = [np.asarray(det, dtype=float) for det in detections]
        removed_track_ids = set()

        # 1. 预测所有 tracker
        for tracker in self.trackers.values():
            tracker.predict()

        if len(self.trackers) == 0:
            detection_to_track = {}
            new_track_ids = set()
            for det_idx, det in enumerate(detections):
                track_id = self._create_tracker(det)
                detection_to_track[det_idx] = track_id
                new_track_ids.add(track_id)
            return self._finalize_update_result(
                detection_to_track=detection_to_track,
                matched_track_ids=set(),
                new_track_ids=new_track_ids,
                unmatched_track_ids=set(),
                removed_track_ids=removed_track_ids,
            )

        # 2. 构建 cost matrix
        tracker_ids = list(self.trackers.keys())
        cost = np.zeros((len(tracker_ids), len(detections)))

        for i, tid in enumerate(tracker_ids):
            state = self.trackers[tid].get_state()
            for j, det in enumerate(detections):
                cost[i,j] = np.linalg.norm(state[:2] - det)

        # 3. 匈牙利匹配
        row_ind, col_ind = linear_sum_assignment(cost)

        matched_track_ids = set()
        assigned_dets = set()
        detection_to_track = {}

        # 4. 更新匹配到的
        for r, c in zip(row_ind, col_ind):

            if cost[r,c] < self.match_threshold:

                tid = tracker_ids[r]
                self.trackers[tid].update(detections[c])

                matched_track_ids.add(tid)
                assigned_dets.add(c)
                detection_to_track[c] = tid

        # 5. 未匹配检测 → 新建
        new_track_ids = set()
        for i, det in enumerate(detections):
            if i not in assigned_dets:
                track_id = self._create_tracker(det)
                detection_to_track[i] = track_id
                new_track_ids.add(track_id)

        # 6. 删除长时间未匹配 tracker
        remove_ids = []
        for tid, tracker in self.trackers.items():
            if tracker.missed > self.max_missed:
                remove_ids.append(tid)

        for tid in remove_ids:
            del self.trackers[tid]
            removed_track_ids.add(tid)

        unmatched_track_ids = set(tracker_ids) - matched_track_ids - removed_track_ids
        return self._finalize_update_result(
            detection_to_track=detection_to_track,
            matched_track_ids=matched_track_ids,
            new_track_ids=new_track_ids,
            unmatched_track_ids=unmatched_track_ids,
            removed_track_ids=removed_track_ids,
        )

    def _create_tracker(self, det):
        tracker = KalmanTrackerCA(det)
        track_id = self.next_id
        self.trackers[track_id] = tracker
        self.next_id += 1
        return track_id

    def _finalize_update_result(
        self,
        detection_to_track,
        matched_track_ids,
        new_track_ids,
        unmatched_track_ids,
        removed_track_ids,
    ):
        """生成当前帧的匹配结果并缓存。"""
        tracker_states = {
            tid: tracker.get_state()
            for tid, tracker in self.trackers.items()
        }
        self.last_update_result = TrackerUpdateResult(
            detection_to_track=dict(detection_to_track),
            matched_track_ids=set(matched_track_ids),
            new_track_ids=set(new_track_ids),
            unmatched_track_ids=set(unmatched_track_ids),
            removed_track_ids=set(removed_track_ids),
            tracker_states=tracker_states,
        )
        return self.last_update_result

    def get_tracks(self):
        outputs = []
        for tid, tracker in self.trackers.items():
            state = tracker.get_state()
            outputs.append((tid, state))
        return outputs

    def get_last_update_result(self):
        """返回最近一帧跟踪更新后的匹配结果。"""
        return self.last_update_result
