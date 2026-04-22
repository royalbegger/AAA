#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

struct CellKey
{
    int x = 0;
    int y = 0;

    bool operator==(const CellKey& other) const
    {
        return x == other.x && y == other.y;
    }
};

struct CellKeyHash
{
    std::size_t operator()(const CellKey& key) const
    {
        return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1);
    }
};

struct CellState
{
    double log_odds = 0.0;
    ros::Time last_update;
};

struct PointRecord
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    CellKey cell;
};

double clampValue(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

double logit(double probability)
{
    const double clamped = clampValue(probability, 1e-4, 1.0 - 1e-4);
    return std::log(clamped / (1.0 - clamped));
}

double logistic(double log_odds)
{
    return 1.0 / (1.0 + std::exp(-log_odds));
}

tf2::Transform transformMsgToTf(const geometry_msgs::Transform& transform_msg)
{
    tf2::Transform transform;
    tf2::Quaternion rotation(transform_msg.rotation.x,
                             transform_msg.rotation.y,
                             transform_msg.rotation.z,
                             transform_msg.rotation.w);
    transform.setOrigin(
        tf2::Vector3(transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z));
    transform.setRotation(rotation);
    return transform;
}

}  // namespace

class DynamicPointCloudFilter
{
public:
    DynamicPointCloudFilter()
        : nh_()
        , pnh_("~")
        , tf_buffer_()
        , tf_listener_(tf_buffer_)
    {
        pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/points_raw");
        pnh_.param<std::string>("output_cloud_topic", output_cloud_topic_, "/points_static_filtered");
        pnh_.param<std::string>("dynamic_cloud_topic", dynamic_cloud_topic_, "/points_dynamic_filtered");
        pnh_.param<std::string>("global_frame", global_frame_, "map");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");

        pnh_.param("grid_resolution", grid_resolution_, 0.15);
        pnh_.param("min_height", min_height_, -0.30);
        pnh_.param("max_height", max_height_, 1.80);
        pnh_.param("max_range", max_range_, 20.0);
        pnh_.param("association_radius_cells", association_radius_cells_, 1);
        pnh_.param("cell_timeout", cell_timeout_, 6.0);
        pnh_.param("max_tracking_distance", max_tracking_distance_, 30.0);
        pnh_.param("tf_lookup_timeout", tf_lookup_timeout_, 0.05);
        pnh_.param("static_hit_probability", static_hit_probability_, 0.72);
        pnh_.param("static_miss_probability", static_miss_probability_, 0.35);
        pnh_.param("static_probability_threshold", static_probability_threshold_, 0.60);
        pnh_.param("publish_dynamic_cloud", publish_dynamic_cloud_, true);

        hit_log_odds_delta_ = logit(static_hit_probability_);
        miss_log_odds_delta_ = logit(static_miss_probability_);
        min_log_odds_ = logit(0.05);
        max_log_odds_ = logit(0.95);

        cloud_sub_ = nh_.subscribe(input_cloud_topic_, 5, &DynamicPointCloudFilter::cloudCallback, this);

        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
        if (publish_dynamic_cloud_) {
            dynamic_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_cloud_topic_, 1);
        }

        ROS_INFO("DynamicPointCloudFilter initialized");
        ROS_INFO("  Input cloud: %s", input_cloud_topic_.c_str());
        ROS_INFO("  Output static cloud: %s", output_cloud_topic_.c_str());
        if (publish_dynamic_cloud_) {
            ROS_INFO("  Output dynamic cloud: %s", dynamic_cloud_topic_.c_str());
        }
        ROS_INFO("  Robot pose via TF: %s -> %s", global_frame_.c_str(), base_frame_.c_str());
        ROS_INFO("  Base frame: %s", base_frame_.c_str());
        ROS_INFO("  Grid resolution: %.3f m", grid_resolution_);
        ROS_INFO("  Static threshold: %.2f", static_probability_threshold_);
    }

private:
    using CellSet = std::unordered_set<CellKey, CellKeyHash>;
    using CellMap = std::unordered_map<CellKey, CellState, CellKeyHash>;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        tf2::Transform global_to_base;
        if (!lookupRobotPose(global_to_base)) {
            return;
        }

        tf2::Transform base_to_cloud;
        if (!lookupBaseToCloudTransform(cloud_msg->header, base_to_cloud)) {
            return;
        }

        std::vector<PointRecord> valid_points;
        CellSet current_cells;
        current_cells.reserve(cloud_msg->width * cloud_msg->height);

        extractCurrentFrame(*cloud_msg, global_to_base, base_to_cloud, valid_points, current_cells);
        updateStaticConfidence(current_cells,
                               global_to_base.getOrigin().x(),
                               global_to_base.getOrigin().y(),
                               cloud_msg->header.stamp);
        publishFilteredClouds(*cloud_msg, valid_points);
        previous_cells_ = current_cells;
    }

    bool lookupRobotPose(tf2::Transform& global_to_base)
    {
        try {
            const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                global_frame_, base_frame_, ros::Time(0), ros::Duration(tf_lookup_timeout_));
            global_to_base = transformMsgToTf(transform.transform);
            return true;
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Failed to lookup robot pose %s -> %s: %s",
                              global_frame_.c_str(), base_frame_.c_str(), ex.what());
            return false;
        }
    }

    bool lookupBaseToCloudTransform(const std_msgs::Header& cloud_header, tf2::Transform& base_to_cloud)
    {
        if (cloud_header.frame_id.empty() || cloud_header.frame_id == base_frame_) {
            base_to_cloud.setIdentity();
            return true;
        }

        try {
            const ros::Time query_time = cloud_header.stamp.isZero() ? ros::Time(0) : cloud_header.stamp;
            const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                base_frame_, cloud_header.frame_id, query_time, ros::Duration(tf_lookup_timeout_));
            base_to_cloud = transformMsgToTf(transform.transform);
            return true;
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Failed to lookup transform %s <- %s: %s",
                              base_frame_.c_str(), cloud_header.frame_id.c_str(), ex.what());
            return false;
        }
    }

    void extractCurrentFrame(const sensor_msgs::PointCloud2& cloud_msg,
                             const tf2::Transform& global_to_base,
                             const tf2::Transform& base_to_cloud,
                             std::vector<PointRecord>& valid_points,
                             CellSet& current_cells)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");

        valid_points.reserve(cloud_msg.width * cloud_msg.height);

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            const tf2::Vector3 point_in_cloud(x, y, z);
            const tf2::Vector3 point_in_base = base_to_cloud * point_in_cloud;
            if (point_in_base.z() < min_height_ || point_in_base.z() > max_height_) {
                continue;
            }

            if (std::hypot(point_in_base.x(), point_in_base.y()) > max_range_) {
                continue;
            }

            const tf2::Vector3 point_in_global = global_to_base * point_in_base;
            const CellKey cell = worldToCell(point_in_global.x(), point_in_global.y());
            current_cells.insert(cell);

            PointRecord record;
            record.x = x;
            record.y = y;
            record.z = z;
            record.cell = cell;
            valid_points.push_back(record);
        }
    }

    CellKey worldToCell(double x, double y) const
    {
        CellKey cell;
        cell.x = static_cast<int>(std::floor(x / grid_resolution_));
        cell.y = static_cast<int>(std::floor(y / grid_resolution_));
        return cell;
    }

    bool hasAssociation(const CellSet& cells, const CellKey& query) const
    {
        for (int dx = -association_radius_cells_; dx <= association_radius_cells_; ++dx) {
            for (int dy = -association_radius_cells_; dy <= association_radius_cells_; ++dy) {
                CellKey candidate;
                candidate.x = query.x + dx;
                candidate.y = query.y + dy;
                if (cells.find(candidate) != cells.end()) {
                    return true;
                }
            }
        }

        return false;
    }

    void updateStaticConfidence(const CellSet& current_cells,
                                double robot_x,
                                double robot_y,
                                const ros::Time& stamp)
    {
        pruneGrid(robot_x, robot_y, stamp);

        for (const CellKey& cell : current_cells) {
            const bool matched_last_frame = hasAssociation(previous_cells_, cell);
            addEvidence(cell, matched_last_frame ? hit_log_odds_delta_ : miss_log_odds_delta_, stamp);
        }

        for (const CellKey& cell : previous_cells_) {
            if (!hasAssociation(current_cells, cell)) {
                addEvidence(cell, miss_log_odds_delta_, stamp);
            }
        }
    }

    void addEvidence(const CellKey& cell, double delta_log_odds, const ros::Time& stamp)
    {
        CellState& state = cell_states_[cell];
        state.log_odds = clampValue(state.log_odds + delta_log_odds, min_log_odds_, max_log_odds_);
        state.last_update = stamp;
    }

    void pruneGrid(double robot_x, double robot_y, const ros::Time& stamp)
    {
        if (cell_states_.empty()) {
            return;
        }

        for (auto it = cell_states_.begin(); it != cell_states_.end();) {
            const double cell_world_x = (static_cast<double>(it->first.x) + 0.5) * grid_resolution_;
            const double cell_world_y = (static_cast<double>(it->first.y) + 0.5) * grid_resolution_;
            const double distance = std::hypot(cell_world_x - robot_x, cell_world_y - robot_y);
            const double age = (stamp - it->second.last_update).toSec();

            if (distance > max_tracking_distance_ || age > cell_timeout_) {
                it = cell_states_.erase(it);
            } else {
                ++it;
            }
        }
    }

    void publishFilteredClouds(const sensor_msgs::PointCloud2& input_cloud,
                               const std::vector<PointRecord>& valid_points)
    {
        std::vector<PointRecord> static_points;
        std::vector<PointRecord> dynamic_points;
        static_points.reserve(valid_points.size());
        dynamic_points.reserve(valid_points.size());

        for (const PointRecord& point : valid_points) {
            const double confidence = getStaticProbability(point.cell);
            if (confidence >= static_probability_threshold_) {
                static_points.push_back(point);
            } else {
                dynamic_points.push_back(point);
            }
        }

        sensor_msgs::PointCloud2 static_cloud;
        buildCloudMessage(input_cloud.header, static_points, static_cloud);
        filtered_cloud_pub_.publish(static_cloud);

        if (publish_dynamic_cloud_) {
            sensor_msgs::PointCloud2 dynamic_cloud;
            buildCloudMessage(input_cloud.header, dynamic_points, dynamic_cloud);
            dynamic_cloud_pub_.publish(dynamic_cloud);
        }
    }

    double getStaticProbability(const CellKey& cell) const
    {
        const auto it = cell_states_.find(cell);
        if (it == cell_states_.end()) {
            return 0.5;
        }

        return logistic(it->second.log_odds);
    }

    void buildCloudMessage(const std_msgs::Header& header,
                           const std::vector<PointRecord>& points,
                           sensor_msgs::PointCloud2& cloud_msg) const
    {
        cloud_msg.header = header;
        cloud_msg.height = 1;
        cloud_msg.width = static_cast<uint32_t>(points.size());
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const PointRecord& point : points) {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher dynamic_cloud_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    CellMap cell_states_;
    CellSet previous_cells_;

    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string dynamic_cloud_topic_;
    std::string global_frame_;
    std::string base_frame_;

    double grid_resolution_ = 0.15;
    double min_height_ = -0.30;
    double max_height_ = 1.80;
    double max_range_ = 20.0;
    double cell_timeout_ = 6.0;
    double max_tracking_distance_ = 30.0;
    double tf_lookup_timeout_ = 0.05;
    double static_hit_probability_ = 0.72;
    double static_miss_probability_ = 0.35;
    double static_probability_threshold_ = 0.60;
    double hit_log_odds_delta_ = 0.0;
    double miss_log_odds_delta_ = 0.0;
    double min_log_odds_ = 0.0;
    double max_log_odds_ = 0.0;
    int association_radius_cells_ = 1;
    bool publish_dynamic_cloud_ = true;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_pointcloud_filter_node");
    DynamicPointCloudFilter node;
    ros::spin();
    return 0;
}
