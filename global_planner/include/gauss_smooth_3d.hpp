#ifndef GAUSS_SMOOTH_3D_HPP
#define GAUSS_SMOOTH_3D_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace astar_planner
{

class GaussSmooth3D
{
public:
    using Point = std::array<double, 3>;

    GaussSmooth3D(int kernel_size, int num_scale)
        : kernel_size_(normalizeKernelSize(kernel_size)),
          num_scale_(std::max(1, num_scale))
    {
    }

    std::vector<double> generateGaussianKernel(double sigma, int phase) const
    {
        if (sigma <= 0.0) {
            throw std::invalid_argument("sigma must be positive");
        }

        const int half = kernel_size_ / 2;
        std::vector<double> kernel(kernel_size_, 0.0);
        double sum = 0.0;

        const double phase_offset = static_cast<double>(phase) / static_cast<double>(num_scale_);
        for (int i = 0; i < kernel_size_; ++i) {
            const double offset = static_cast<double>(i - half) - phase_offset;
            kernel[i] = std::exp(-0.5 * std::pow(offset / sigma, 2.0));
            sum += kernel[i];
        }

        if (sum <= 0.0) {
            throw std::runtime_error("gaussian kernel normalization failed");
        }

        for (double& value : kernel) {
            value /= sum;
        }

        return kernel;
    }

    std::vector<Point> smoothTrajectory(const std::vector<Point>& trajectory,
                                        double sigma = 1.0) const
    {
        if (trajectory.size() < 2) {
            return trajectory;
        }

        std::vector<Point> padded;
        padded.reserve(trajectory.size() + 2 * static_cast<size_t>(kernel_size_));

        for (int i = 0; i < kernel_size_; ++i) {
            padded.push_back(trajectory.front());
        }
        padded.insert(padded.end(), trajectory.begin(), trajectory.end());
        for (int i = 0; i < kernel_size_; ++i) {
            padded.push_back(trajectory.back());
        }

        std::vector<std::vector<double>> kernels;
        kernels.reserve(static_cast<size_t>(num_scale_));
        for (int phase = 0; phase < num_scale_; ++phase) {
            kernels.push_back(generateGaussianKernel(sigma, phase));
        }

        std::vector<Point> smoothed;
        smoothed.reserve(padded.size() * static_cast<size_t>(num_scale_));

        for (size_t i = 0; i < padded.size() * static_cast<size_t>(num_scale_); ++i) {
            const int phase = static_cast<int>(i % static_cast<size_t>(num_scale_));
            const size_t base_idx = i / static_cast<size_t>(num_scale_);
            const auto& kernel = kernels[phase];

            Point acc{{0.0, 0.0, 0.0}};
            for (int k = 0; k < kernel_size_; ++k) {
                const size_t idx = std::min(base_idx + static_cast<size_t>(k), padded.size() - 1);
                for (size_t dim = 0; dim < acc.size(); ++dim) {
                    acc[dim] += kernel[static_cast<size_t>(k)] * padded[idx][dim];
                }
            }

            smoothed.push_back(acc);
        }

        return smoothed;
    }

    int kernelSize() const
    {
        return kernel_size_;
    }

    int numScale() const
    {
        return num_scale_;
    }

private:
    static int normalizeKernelSize(int kernel_size)
    {
        if (kernel_size < 1) {
            kernel_size = 1;
        }
        if (kernel_size % 2 == 0) {
            ++kernel_size;
        }
        if (kernel_size > 101) {
            kernel_size = 13;
        }
        return kernel_size;
    }

    int kernel_size_;
    int num_scale_;
};

} // namespace astar_planner

#endif // GAUSS_SMOOTH_3D_HPP
