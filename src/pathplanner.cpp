#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <classification/bbox.h>

class PathPlanner{
    public:
        PathPlanner(ros::NodeHandle nh) : nh_(nh) {
            sub_ = nh.subscribe<classification::bbox> ("/feature/bbox", 1, &PathPlanner::bboxCB, this);
            sub_step_height = nh.subscribe<std_msgs::Float32> ("/feature/step", 1, &PathPlanner::stepCB, this);
            sub_path_type = nh.subscribe<std_msgs::Bool> ("/feature/path", 1, &PathPlanner::pathTypeCB, this);
            pub_ = nh.advertise<nav_msgs::Path> ("/feature/plan", 1);
        };

        void spin() {

            if(progress == state::IN_PROGRESS && pplan.poses.size() != 0){

                auto p = pplan.poses.at(curr_pos).pose.position;

                double distPrev = sqrt(pow(agent.x + p.x, 2) +
                                    pow(agent.y + p.y, 2) +
                                    pow(agent.z + p.z, 2));

                // If within 1m of next target position
                if(distPrev < 1) {

                    if(pplan.poses.rend()->header.seq == curr_pos){
                        progress == state::COMPLETED;
                        auto diff = ros::Time::now().toSec() - time.toSec();

                        ROS_INFO("Path Plan Completed - Total Time: %lf", diff);
                    }else{
                        curr_pos++;
                        ROS_INFO("Point Reached! Moving to next location");
                    }

                }

            }

        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        ros::Subscriber sub_step_height;
        ros::Subscriber sub_path_type;

        nav_msgs::Path pplan;
        ros::Time time;

        geometry_msgs::Point agent;

        geometry_msgs::Pose origin;
        float height;
        float width;
        float step = 1;

        bool path_type;
        int curr_pos = 0;

        enum state{
            IN_PROGRESS,
            COMPLETED
        } progress;


        void stepCB(const std_msgs::Float32::ConstPtr& msg) {
            step = msg->data;
            ROS_INFO("Step height changed to : %f", msg->data);
            genPathPlan();
        };

        void bboxCB(const classification::bbox::ConstPtr& msg) {
            double distPrev = sqrt(pow(origin.position.x + msg->origin.position.x, 2) +
                                    pow(origin.position.y + msg->origin.position.y, 2) +
                                    pow(origin.position.z + msg->origin.position.z, 2));
            origin = msg->origin;
            height = msg->height.data;
            width = msg->width.data;

            ROS_INFO("BBOX model updated, ORIGIN - {%f, %f, %f}    HEIGHT: %f    WIDTH: %f",
                        origin.position.x,
                        origin.position.y,
                        origin.position.z,
                        height,
                        width);

            if(distPrev>width/2){
                curr_pos = 0;
                time = ros::Time::now();
                progress = state::IN_PROGRESS;
            }
            genPathPlan();
        };

        void pathTypeCB(const std_msgs::Bool::ConstPtr& msg){
            path_type = msg->data;
            genPathPlan();
        }



        void genPathPlan(){
            geometry_msgs::Quaternion q = origin.orientation;
            double rotation = asin(-2.0 * (q.x * q.z - q.w * q.y));

            geometry_msgs::Point front; front.x = cos(rotation); front.z = sin(rotation);
            geometry_msgs::Point right; right.x = sin(rotation); right.z = cos(rotation);

            ROS_INFO("FRONT - {%f, %f, %f}     FRONT - {%f, %f, %f}",
                        front.x,
                        front.y,
                        front.z,
                        right.x,
                        right.y,
                        right.z);

            nav_msgs::Path pplan_msg;

            if(!path_type){
                for(int i = 1; i*step < height; i++){
                    geometry_msgs::PoseStamped newp;
                    newp.header.seq = i;
                    newp.pose.position = origin.position;
                    switch(i % 4){
                        case 0:
                            newp.pose.position.x += front.x * width/2;
                            newp.pose.position.z += front.z * width/2;
                            break;
                        case 1:
                            newp.pose.position.x += right.x * width/2;
                            newp.pose.position.z += right.z * width/2;
                            break;
                        case 2:
                            newp.pose.position.x -= front.x * width/2;
                            newp.pose.position.z -= front.z * width/2;
                            break;
                        case 3:
                            newp.pose.position.x -= right.x * width/2;
                            newp.pose.position.z -= right.z * width/2;
                            break;
                    }
                    newp.pose.position.y += (float)i*step;
                    pplan_msg.poses.push_back(newp);
                }
            }else{
                for(int i = 0; i < 8; i++){
                    geometry_msgs::PoseStamped newp;
                    newp.header.seq = i;
                    newp.pose.position = origin.position;
                    switch(i % 4){
                        case 0:
                            newp.pose.position.x += front.x * width/2;
                            newp.pose.position.z += front.z * width/2;
                            break;
                        case 1:
                            newp.pose.position.x += right.x * width/2;
                            newp.pose.position.z += right.z * width/2;
                            break;
                        case 2:
                            newp.pose.position.x -= front.x * width/2;
                            newp.pose.position.z -= front.z * width/2;
                            break;
                        case 3:
                            newp.pose.position.x -= right.x * width/2;
                            newp.pose.position.z -= right.z * width/2;
                            break;
                    }
                    if(i==1 || i==2 || i==5 || i==6)
                        newp.pose.position.y += height;
                    else
                        newp.pose.position.y += step;
                    pplan_msg.poses.push_back(newp);
                }
            }
            pplan = pplan_msg;
            pub_.publish(pplan_msg);

        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_filter");
    ros::NodeHandle nh;

    auto pp = PathPlanner(nh);

    while(ros::ok()){
        ros::spinOnce();
        pp.spin();
    }
    return 0;
}
