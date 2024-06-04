#include <ros/ros.h>
#include <vision_seg/aruco_center.h>
#include <vision_seg/seg_center.h>
#include <geometry_msgs/TransformStamped.h>
#include <vision_seg/find_tf_service.h>

class Arucofinder {
public:
    void arucoXYCallback(const vision_seg::aruco_center::ConstPtr& msg) {
        aruco_all_pose.clear();
        if (msg->names.size() > 0) {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string marker_id = msg->names[i];
                float x = msg->x_points[i];
                float y = msg->y_points[i];
                aruco_all_pose.push_back({marker_id, x, y});
            }
        }
    }

    void segXYCallback(const vision_seg::seg_center::ConstPtr& msg) {
        // ROS_INFO("Seg marker detected");
        seg_pose.push_back({msg->names, msg->x_points, msg->y_points});
    }
    void startServer() {
        aruco_xy_sub = nh.subscribe("aruco_cam_xy", 10, &Arucofinder::arucoXYCallback, this);
        seg_xy_sub = nh.subscribe("seg_cam_xy", 10, &Arucofinder::segXYCallback, this);
        ros::ServiceServer service = nh.advertiseService("recent_tf_service", &Arucofinder::callback, this);
        ros::spin();
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber aruco_xy_sub;
    ros::Subscriber seg_xy_sub;
    std::vector<std::tuple<std::string, float, float>> aruco_all_pose;
    std::vector<std::tuple<std::string, float, float>> seg_pose;
    bool server_state;

    bool callback(vision_seg::find_tf_service::Request &req, 
    vision_seg::find_tf_service::Response &res) {
        std::string recent_id =findRecentAruco();
        ROS_INFO("recent_id: %s", req.class_name.c_str());
        if (!recent_id.empty()) {
            res.return_name = recent_id;
            ROS_INFO("i return recent_id : %s", res.return_name.c_str());
            return true;
        }else {
            ROS_WARN("no aruco marker detected");
            return false;
        }
    }

    std::string findRecentAruco() {
        float min_val = std::numeric_limits<float>::max();
        std::string recent_id;
        ROS_INFO("aruco_all_pose size: %d", aruco_all_pose.size());
        ROS_INFO("seg_pose size: %d", seg_pose.size());
        for (const auto& pose : aruco_all_pose) {
            for (const auto& seg : seg_pose) {
                float y_diff = std::get<1>(pose) - std::get<1>(seg); // y 좌표 차이 계산
                float tmp_val = std::abs(y_diff);
                if (tmp_val < min_val) {
                    min_val = tmp_val;
                    recent_id = std::get<0>(pose);
                }
            }
        }
        return recent_id;
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "findNearAruco");
    Arucofinder arucofinder ;
    arucofinder.startServer();
    return 0;
}