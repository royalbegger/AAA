# coding=utf-8
import numpy as np


class gauss_smooth_3d:

    def __init__(self, kernel_size, num_scale):
        self.kernel_size = kernel_size
        self.num_scale = num_scale

    def generate_gaussian_kernel(self, sigma, id):
        if self.kernel_size % 2 == 0:
            self.kernel_size += 1
        if self.kernel_size > 101:
            self.kernel_size = 13

        half = self.kernel_size // 2
        kernel = np.zeros(self.kernel_size)
        s = 0.0

        idx = float(id)
        for i in range(self.kernel_size):
            offset = i - half - idx / self.num_scale
            kernel[i] = np.exp(-0.5 * (offset / sigma) ** 2)
            s += kernel[i]

        kernel /= s
        return kernel

    def smooth_3d_trajectory(self, traj_pts, sigma=1.0):
        if traj_pts.ndim < 1 or traj_pts.shape[1] < 2:
            raise ValueError("dimension of traj_pts must be at least 2")

        dimension = traj_pts.shape[1]

        padded = np.vstack([
            np.repeat(traj_pts[:1], self.kernel_size, axis=0),
            traj_pts,
            np.repeat(traj_pts[-1:], self.kernel_size, axis=0)
        ])

        n_pts = len(padded)

        kernels = [
            self.generate_gaussian_kernel(sigma, i)
            for i in range(self.num_scale)
        ]

        smoothed = []

        for i in range(self.num_scale * n_pts):
            id = i % self.num_scale
            base_idx = i // self.num_scale
            kernel = kernels[id]

            acc = np.zeros(dimension)
            for k in range(self.kernel_size):
                idx = base_idx + k
                idx = max(0, min(idx, n_pts - 1))
                acc += kernel[k] * padded[idx][:dimension]

            smoothed.append(acc)

        return np.array(smoothed)
