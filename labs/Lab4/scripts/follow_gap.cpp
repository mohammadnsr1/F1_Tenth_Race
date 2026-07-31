#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <vector>
#include <algorithm>
#include <numeric>

class ReactiveFollowGap {
public:
    ReactiveFollowGap() {
        ros::NodeHandle nh;
        lidar_sub = nh.subscribe("/scan", 10, &ReactiveFollowGap::lidarCallback, this);
        drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
        bubble_radius = 0.5; // meters
    }

private:
    ros::Subscriber lidar_sub;
    ros::Publisher drive_pub;
    double bubble_radius;

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        std::vector<float> proc_ranges = preprocessLidar(scan->ranges);

        // Finding the minimum range to calculate the bubble index
        auto bubble_idx = std::distance(proc_ranges.begin(), std::min_element(proc_ranges.begin(), proc_ranges.end()));
        double nominal_distance = std::accumulate(proc_ranges.begin(), proc_ranges.end(), 0.0) / proc_ranges.size();
        double theta = bubble_radius / nominal_distance;
        int num_points_in_bubble = static_cast<int>(theta / scan->angle_increment);

        // Zero out the points within the bubble radius
        int start_idx = std::max(0, static_cast<int>(bubble_idx - num_points_in_bubble));
	int end_idx = std::min(static_cast<int>(proc_ranges.size()), static_cast<int>(bubble_idx + num_points_in_bubble));
        std::fill(proc_ranges.begin() + start_idx, proc_ranges.begin() + end_idx, 0.0);

        // Finding the max gap
        int max_start, max_end;
        findMaxGap(proc_ranges, max_start, max_end);

        // Finding the best point in the max gap
        int best_point_idx = findBestPoint(proc_ranges, max_start, max_end);

        // Calculate the angle to the best point
        double angle = scan->angle_min + best_point_idx * scan->angle_increment;

        // Publish the drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = 1.0; // Set a constant speed or adjust based on context
        drive_pub.publish(drive_msg);
    }

    std::vector<float> preprocessLidar(const std::vector<float>& ranges) {
        std::vector<float> proc_ranges(ranges.size(), 0);
        // Simple moving average with window size of 5
        for (size_t i = 2; i < ranges.size() - 2; ++i) {
            proc_ranges[i] = (ranges[i-2] + ranges[i-1] + ranges[i] + ranges[i+1] + ranges[i+2]) / 5.0;
        }
        // Thresholding values greater than 3 meters
        for (auto& range : proc_ranges) {
            if (range > 3) range = 0;
        }
        return proc_ranges;
    }

    void findMaxGap(const std::vector<float>& ranges, int& max_start, int& max_end) {
        int max_gap = 0, current_start = -1;
        max_start = 0; max_end = 0;
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > 0 && current_start < 0) current_start = i; // Start of a new gap
            else if (ranges[i] == 0 && current_start >= 0) { // End of the current gap
                int current_gap = i - current_start;
                if (current_gap > max_gap) {
                    max_gap = current_gap;
                    max_start = current_start;
                    max_end = i - 1;
                }
                current_start = -1;
            }
        }
        // Check if the last gap reaches the end of the range array
        if (current_start >= 0) {
            int current_gap = ranges.size() - current_start;
            if (current_gap > max_gap) {
                max_gap = current_gap;
                max_start = current_start;
                max_end = ranges.size() - 1;
            }
        }
    }

    int findBestPoint(const std::vector<float>& ranges, int start_idx, int end_idx) {
        return std::distance(ranges.begin(), std::max_element(ranges.begin() + start_idx, ranges.begin() + end_idx + 1));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "follow_gap_node");
    ReactiveFollowGap reactive_follow_gap;
    ros::spin();
    return 0;
}

