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
                float marker_id = msg->names[i];
                float x = msg->x_points[i];
                float y = msg->y_points[i];
                aruco_all_pose.push_back({marker_id, x, y});
                ROS_INFO("Aruco marker detected");
            }
        }
    }

    void segXYCallback(const vision_seg::seg_center::ConstPtr& msg) {
        ROS_INFO("Seg marker detected");
        seg_pose.push_back(msg->names);
        seg_pose.push_back(msg->x_points);
        seg_pose.push_back(msg->y_points);
    }
    void startServer() {
        aruco_xy_sub = nh.subscribe("arco_cam_xy", 10, &Arucofinder::arucoXYCallback, this);
        seg_xy_sub = nh.subscribe("seg_cam_xy", 10, &Arucofinder::segXYCallback, this);
        ros::ServiceServer service = nh.advertiseService("aruco_seg_start", &Arucofinder::callback, this);
        ros::spin();
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber aruco_xy_sub;
    ros::Subscriber seg_xy_sub;
    std::vector<std::vector<float>> aruco_all_pose;
    std::vector<float> seg_pose;
    bool server_state;

    bool callback(vision_seg::find_tf_service::Request &req, 
    vision_seg::find_tf_service::Response &res) {
        float recent_id =findRecentAruco();
        if (recent_id != -1) {
            res.return_name = recent_id;
            ROS_INFO("i return recent_id %f", recent_id);
            return true;
        }
        else {
            ROS_WARN("no aruco marker detected");
            return false;
        }
    }

    int findRecentAruco(){
        float min_val = std::numeric_limits<float>::max();
        float recent_id = -1;
        for(const auto& pose : aruco_all_pose){
            float y_diff = pose[1] - seg_pose[1];
            float tmp_val = abs(y_diff);
            if(tmp_val < min_val){
                min_val = tmp_val;
                recent_id = pose[0];
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