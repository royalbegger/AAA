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
        map_sub_ = nh.subscribe("/map", 1, &ESDF2D::mapCallback, this);
        esdf_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/esdf_map", 1);
    }

private:
    ros::Subscriber map_sub_;
    ros::Publisher esdf_pub_;
    const float INF = 1e8;

    // ----------------- 1D Squared Distance Transform -----------------
    //实现了高效的Felzenszwalb算法（线性时间算法）     
    //输入：一维数组f，其中f[i]是原始距离（障碍物为0，其他为INF）
    //输出：一维数组d，其中d[i]是到最近障碍物的平方距离
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

    // ----------------- Map Callback -----------------
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
        int W = map->info.width;
        int H = map->info.height;
        float res = map->info.resolution;

        // 初始化所有栅格为极大值（INF = 1e8）
        std::vector<float> grid(W * H, INF);

        //Step 1: 初始化障碍
        int Changing_Flag = 0;
        int value = 100;
        int x1 = 0;
        int x2 = 0;
        while (!(x1 < 96 && x2 > 96))
        {
            for (int x = 0; x < W; x++)
            {
                int idx = value * W + x;
                int occ = map->data[idx];
                if(occ > 0 && Changing_Flag == 0)
                {
                    x1 = x;
                    Changing_Flag = 1;
                }
                else if(occ < 0 && Changing_Flag == 1)
                {
                    x2 = x;
                    Changing_Flag = 0;
                }
            }
            value++ ;
        }
        
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                int idx = y * W + x;
                int occ = map->data[idx];
                if (x < x1 || x > x2)
                    grid[idx] = 0.0f;   
                else 
                    if(occ == 100 )
                        // 障碍物位置距离设为0
                        grid[idx] = 0.0f;              
            }
        }

        // Step 2: X 方向 EDT X方向距离变换
        std::vector<float> temp(W * H, INF);
        for (int y = 0; y < H; y++)
        {
            // 提取当前行的距离值
            std::vector<float> f(W), d(W);
            for (int x = 0; x < W; x++)
                f[x] = grid[y * W + x];

            // 执行一维距离变换
            distanceTransform1D(f, d, W);

            for (int x = 0; x < W; x++)
                temp[y * W + x] = d[x];
        }

        // Step 3: Y 方向 EDT
        std::vector<float> dist(W * H, INF);
        for (int x = 0; x < W; x++)
        {
            // 提取当前列的距离值（来自X方向变换结果）
            std::vector<float> f(H), d(H);
            for (int y = 0; y < H; y++)
                f[y] = temp[y * W + x];

            // 执行一维距离变换
            distanceTransform1D(f, d, H);

            for (int y = 0; y < H; y++)
                // 计算实际欧几里得距离 = sqrt(distance²) * 分辨率
                dist[y * W + x] = std::sqrt(d[y]) * res;
        }

        // Step 4: 输出 ESDF OccupancyGrid
        nav_msgs::OccupancyGrid esdf;
        esdf.header = map->header;
        esdf.info = map->info;
        esdf.data.resize(W * H);
        const float D_MAX = 1.0; // 最大有效距离
        for (int i = 0; i < W * H; i++)
        {                // obstacle
            if(grid[i] == 0.0f)
                esdf.data[i] = 100;
            else
            {
                float d = std::min(dist[i], D_MAX); // 截断
                if (d <= D_MAX)
                {
                    esdf.data[i] = static_cast<int8_t>( (1 - d / D_MAX) * 100 );
                }
                else 
                {
                    esdf.data[i] = 0;
                }

            }
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
