#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>
#include <limits>

class ESDF2D
{
public:
    ESDF2D()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        // 未知区域衰减参数
        pnh.param<double>("unknown_decay_factor", unknown_decay_factor_, 0.8);
        pnh.param<int>("unknown_decay_offset", unknown_decay_offset_, 0);
        pnh.param<double>("obstacle_inflation_factor", obstacle_inflation_factor_, 1);
        // 确保衰减因子在 (0,1] 范围内
        if (unknown_decay_factor_ <= 0.0) unknown_decay_factor_ = 0.1;
        if (unknown_decay_factor_ > 1.0) unknown_decay_factor_ = 1.0;
        if (obstacle_inflation_factor_ < 0.0) obstacle_inflation_factor_ = 0.0;

        map_sub_ = nh.subscribe("/map", 1, &ESDF2D::mapCallback, this);
        esdf_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/esdf_map", 1);
    }

private:
    ros::Subscriber map_sub_;
    ros::Publisher esdf_pub_;
    const float INF = 1e8;
    double unknown_decay_factor_;
    double obstacle_inflation_factor_;
    int unknown_decay_offset_;

    // ----------------- 1D Squared Distance Transform -----------------
    void distanceTransform1D(
        const std::vector<float>& f,
        std::vector<float>& d,
        int n)
    {
        std::vector<int> v(n);
        std::vector<float> z(n + 1);

        int k = 0;
        v[0] = 0;
        z[0] = -INF;
        z[1] = INF;

        for (int q = 1; q < n; q++)
        {
            float s;
            do {
                s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * (q - v[k]));
                if (s <= z[k]) k--;
            } while (s <= z[k]);

            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = INF;
        }

        k = 0;
        for (int q = 0; q < n; q++)
        {
            while (z[k + 1] < q) k++;
            float dx = q - v[k];
            d[q] = dx * dx + f[v[k]];
        }
    }

    void inflateObstacleGrid(std::vector<float>& grid, int W, int H)
    {
        if (obstacle_inflation_factor_ <= 0.0) {
            return;
        }

        const int radius_cells = static_cast<int>(std::ceil(obstacle_inflation_factor_));
        const double radius_sq = obstacle_inflation_factor_ * obstacle_inflation_factor_;
        std::vector<int> obstacle_indices;
        obstacle_indices.reserve(grid.size());

        for (int idx = 0; idx < static_cast<int>(grid.size()); ++idx)
        {
            if (grid[idx] == 0.0f)
            {
                obstacle_indices.push_back(idx);
            }
        }

        for (const int idx : obstacle_indices)
        {
            const int ox = idx % W;
            const int oy = idx / W;
            for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            {
                for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                {
                    if (dx * dx + dy * dy > radius_sq)
                    {
                        continue;
                    }

                    const int nx = ox + dx;
                    const int ny = oy + dy;
                    if (nx < 0 || nx >= W || ny < 0 || ny >= H)
                    {
                        continue;
                    }

                    grid[ny * W + nx] = 0.0f;
                }
            }
        }
    }

    // ----------------- Map Callback -----------------
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
        int W = map->info.width;
        int H = map->info.height;
        float res = map->info.resolution;

        // 初始化所有栅格为极大值（INF = 1e8）
        std::vector<float> grid(W * H, INF);

        // Step 1: 初始化障碍物（保留原代码中的边界裁剪逻辑，但可能只适用于特定场景）
        int Changing_Flag = 0;
        int value = 100;
        int x1 = 0;
        int x2 = 0;
        // while (!(x1 < 96 && x2 > 96))
        // {
        //     for (int x = 0; x < W; x++)
        //     {
        //         int idx = value * W + x;
        //         int occ = map->data[idx];
        //         if(occ > 0 && Changing_Flag == 0)
        //         {
        //             x1 = x;
        //             Changing_Flag = 1;
        //         }
        //         else if(occ < 0 && Changing_Flag == 1)
        //         {
        //             x2 = x;
        //             Changing_Flag = 0;
        //         }
        //     }
        //     value++ ;
        // }
        
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                int idx = y * W + x;
                int occ = map->data[idx];
                // if (x < x1 || x > x2)
                //     grid[idx] = 0.0f;   
                // else 
                if(occ == 100)
                    grid[idx] = 0.0f;              
            }
        }

        // Step 1.5: 对原始占用栅格进行障碍膨胀，膨胀系数默认 1.2 个栅格。
        inflateObstacleGrid(grid, W, H);

        // Step 2: X 方向 EDT
        std::vector<float> temp(W * H, INF);
        for (int y = 0; y < H; y++)
        {
            std::vector<float> f(W), d(W);
            for (int x = 0; x < W; x++)
                f[x] = grid[y * W + x];
            distanceTransform1D(f, d, W);
            for (int x = 0; x < W; x++)
                temp[y * W + x] = d[x];
        }

        // Step 3: Y 方向 EDT
        std::vector<float> dist(W * H, INF);
        for (int x = 0; x < W; x++)
        {
            std::vector<float> f(H), d(H);
            for (int y = 0; y < H; y++)
                f[y] = temp[y * W + x];
            distanceTransform1D(f, d, H);
            for (int y = 0; y < H; y++)
                dist[y * W + x] = std::sqrt(d[y]) * res;
        }

        // Step 4: 输出 ESDF OccupancyGrid，并对未知区域进行衰减
        nav_msgs::OccupancyGrid esdf;
        esdf.header = map->header;
        esdf.info = map->info;
        esdf.data.resize(W * H);
        const float D_MAX = 1.3; // 最大有效距离
        for (int i = 0; i < W * H; i++)
        {
            int8_t value;
            // 障碍物区域（grid[i]==0.0f 表示障碍物或边界裁剪区域）
            if (grid[i] == 0.0f)
            {
                value = 100;
            }
            else
            {
                float d = std::min(dist[i], D_MAX);
                value = static_cast<int8_t>((1 - d / D_MAX) * 100);
            }

            // 如果是未知区域，应用衰减
            if (map->data[i] == -1)
            {
                // 先衰减（乘法）
                double decayed = static_cast<double>(value) * unknown_decay_factor_;
                // 再叠加偏移（可选）
                decayed += unknown_decay_offset_;
                // 限制在 0~100 之间
                if (decayed > 100.0) decayed = 100.0;
                if (decayed < 0.0) decayed = 0.0;
                value = static_cast<int8_t>(decayed);
            }

            esdf.data[i] = value;
        }

        esdf_pub_.publish(esdf);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "esdf_2d_node");
    ESDF2D node;
    ros::spin();
    return 0;
}
