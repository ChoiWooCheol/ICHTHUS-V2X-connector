#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<autoware_msgs/LaneArray.h>
#include<autoware_msgs/Lane.h>
#include<autoware_msgs/Waypoint.h>
#include<geometry_msgs/TwistStamped.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/tf.h>
#include<cmath>
#include<fstream>
#include<string>
#include<cstdlib>
#include<iostream>
#include<alpha_city_msgs/connected_signal.h>
#include<alpha_city_msgs/t_signal.h>
#include<alpha_city_msgs/Intersection.h>

#define CURRENT_POSE_FREQUENCY 0.01

enum SIG{
    GO,
    LEFT,
    GOLEFT,
    NONE
};

class TrafficInfo{
public:
    TrafficInfo()
    {}

    TrafficInfo(int sid, int cid, int tid, int type, double x, double y, double z)
    : sid_(sid)
    , cid_(cid)
    , tid_(tid)
    , type_(type)
    , x_(x)
    , y_(y)
    , z_(z)
    {}
    int unique_id;
    int sid_; // stopline id (csv)
    int cid_; // crossroad id (csv)
    int tid_; // traffic light id (csv)
    int type_; // go, left, goleft, none (csv)
    double x_, y_, z_; // traffic light pose (csv)
    int time_ = -1; // (v2x)
    bool signal_ = true; // (v2x)
    autoware_msgs::Waypoint wp_;
    int wp_idx;
    int lane_idx;
    double distance = 0.0;
};

class TrafficSignalConnector{
public:
    TrafficSignalConnector()
    : private_nh("~")
    {
        if(!private_nh.getParam("csv_path", path)) throw std::runtime_error("set csv_path");
        if(!private_nh.getParam("test", test)) throw std::runtime_error("set test_param");
        sub_GlobalPaths = nh.subscribe("/lane_waypoints_array", 1, &TrafficSignalConnector::callbackLaneWaypointsArray, this);
        sub_CurrentPose = nh.subscribe("/current_pose", 1, &TrafficSignalConnector::callbackCurrentPose, this);
        sub_CurrentVelocity = nh.subscribe("/current_velocity", 1, &TrafficSignalConnector::callbackCurrentVelocity, this);
        sub_Trajectory_Cost = nh.subscribe("/local_trajectory_cost", 1, &TrafficSignalConnector::callbackGetLocalTrajectoryCost, this);

        if(test)
            sub_TrafficLightsInfo = nh.subscribe("/v2x_test_info", 1, &TrafficSignalConnector::callbackTrafficLightsInfo, this);
        else
            sub_TrafficLightsInfo = nh.subscribe("/v2x_signal_info", 1, &TrafficSignalConnector::callbackTrafficLightsInfo, this);
    
        pub_ConnectedTrafficSignal = nh.advertise<alpha_city_msgs::connected_signal>("/connected_traffic_signal", 1);
        pub_StopLineRviz = nh.advertise<visualization_msgs::MarkerArray>("stopline_array_rviz", 1, true);
        initCSV();
    }

    ~TrafficSignalConnector()
    {

    }

    void callbackLaneWaypointsArray(const autoware_msgs::LaneArray::ConstPtr& msg)
    {
        // global path를 받아오고 connectTrafficSignal 함수 호출.
        m_global_path = *msg;
        int lane_size = msg->lanes.size();
        m_prev_gid = msg->lanes.at(lane_size-1).waypoints[0].gid;
        if(!globalpath_ok)
        {
            globalpath_ok = true;
            ROS_INFO("globalpath_ok");
        }
        connect();
    }

    void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_current_pose = *msg;
        if(!current_pose_ok)
        {
            current_pose_ok = true;
            ROS_INFO("current_pose_ok");
        } 
        if(m_connected_traffic_signals.size() != 0)
        {
            for(auto& ti : m_connected_traffic_signals)
            {
                if(ti.time_ > 0.0) ti.time_ = ti.time_ - CURRENT_POSE_FREQUENCY;
            }
        }
        debug();
    }

    void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        m_current_speed = msg->twist.linear.x;
        if(!current_vel_ok){
            current_vel_ok = true;
            ROS_INFO("current_vel_ok");
        } 
    }

    void callbackGetLocalTrajectoryCost(const autoware_msgs::Lane::ConstPtr& msg)
    {
        if(!local_trajectory_ok){
            local_trajectory_ok = true;
            ROS_INFO("local_trajectory_ok");
        } 
        m_Current_TrajectoryIndex = msg->lane_id;
        if(m_Current_TrajectoryIndex == m_Prev_TrajectoryIndex)
        {
            connect_ok = true;
        }
        else
        {
            connect();
        }
        m_Prev_TrajectoryIndex = m_Current_TrajectoryIndex;
    }   

    void callbackTrafficLightsInfo(const alpha_city_msgs::Intersection::ConstPtr& msg){
        if(connected_ok())
        {
            for(auto& ti : m_connected_traffic_signals)
            {
                if(msg->id == ti.cid_)
                {
                    for(auto& pole : msg->poles)
                    {
                        for(auto& detail : pole.detail)
                        {
                            if(detail.type_id == ti.tid_)
                            {
                                ti.signal_ = detail.current_event;
                                ti.time_ = detail.min_endtime;
                            }
                        }
                    }
                }
            }
            v2x_ok = true;
        }
    }

    bool connected_ok()
    {
        return (connect_ok && init_ok && local_trajectory_ok && globalpath_ok && current_vel_ok && current_pose_ok);
    }

    void connect()
    {
        connect_ok = false;
        m_connected_traffic_signals.clear();
        m_connected_traffic_signals.resize(0);
        if(local_trajectory_ok) connectTrafficSignal(m_global_path);
    }

    void initCSV()
    {
        m_csv_info.clear();
        m_csv_info.resize(0);
        std::ifstream f(path.c_str());
        if(f.fail())
        {
            ROS_ERROR("CSV loading... failed");
            exit(1);
        }
        
        std::string s;
        int cnt = 0;
        TrafficInfo ti;
        getline(f,s);
        while(!f.eof()){
            getline(f,s);
            char* tok = strtok(s.c_str(),",");
            cnt = 0;
            while(tok!=NULL){
                if(cnt == 0) ti.sid_ = atoi(tok);
                else if (cnt == 1) ti.cid_ = atoi(tok);
                else if (cnt == 2) ti.tid_ = atoi(tok);
                else if (cnt == 3) ti.type_ = atoi(tok);
                else if (cnt == 4) ti.x_ = atof(tok);
                else if (cnt == 5) ti.y_ = atof(tok);
                else if (cnt == 6) 
                {
                    ti.z_ = atof(tok);
                    m_csv_info.emplace_back(ti);
                    cnt = -1;
                }
                cnt++;
                tok = strtok(NULL,",");
            }
        }
        f.close();
        ROS_INFO("CSV init_ok");
        init_ok = true;
    }    

    void debug()
    {
        if(connected_ok()) publishConnectedSignals();
    }

    void publishConnectedSignals()
    {
        std::vector<TrafficInfo> possible_signals = isPossible(m_global_path);
        alpha_city_msgs::connected_signal cs;
        for(auto& ti : possible_signals)
        {
            alpha_city_msgs::t_signal s;
            s.sid        = ti.sid_;
            s.cid        = ti.cid_;
            s.tid        = ti.tid_;
            s.type       = ti.type_;
            s.x          = ti.x_;
            s.y          = ti.y_;
            s.z          = ti.z_;
            s.alive_time = ti.time_;
            s.sig        = ti.signal_;
            s.distance   = ti.distance;
            s.waypoint   = ti.wp_;
            cs.signals.emplace_back(s);
            // std::cout << "dist : " << s.distance << std::endl;
        }
        pub_ConnectedTrafficSignal.publish(cs);
    }

    void setID(TrafficInfo& ti_)
    {
        ti_.unique_id = ti_.cid_ * 100 + ti_.tid_;
    }

    int getCurrentTrajectoryIndex()
    {
        int idx = m_global_path.lanes.size() - 1;
        if(m_prev_gid == m_global_path.lanes.at(idx).waypoints[0].gid)
            return m_Current_TrajectoryIndex;
        else -1;
    }

    void connectTrafficSignal(autoware_msgs::LaneArray& lanes)
    {
        // csv 파일 중 global path 위에 있는 stop line을 vector에 저장
        // connect_ok : true
        int lane_id = 0;
        int wp_idx = 0;
        
        int idx = getCurrentTrajectoryIndex();
        if(idx < 0) return;
        int max_lane_idx = lanes.lanes.size() - 1;
        idx = std::min(idx, max_lane_idx);
        for(auto& wp : lanes.lanes.at(idx).waypoints)
        {
            for(auto& csv : m_csv_info)
            {
                if(sqrt(pow(wp.pose.pose.position.x - csv.x_, 2) + pow(wp.pose.pose.position.y - csv.y_, 2)) < 0.5)
                {
                    TrafficInfo ti(csv.sid_, csv.cid_, csv.tid_, csv.type_, csv.x_, csv.y_, csv.z_);
                    ti.wp_ = wp;
                    ti.lane_idx = idx;
                    ti.wp_idx = wp_idx;
                    m_connected_traffic_signals.emplace_back(ti);
                    break;
                }
            }
            wp_idx++;
        }
        visualization_msgs::MarkerArray branch_array;

        for(unsigned int i=0; i< m_connected_traffic_signals.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "HMI_Destinations";
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            //marker.scale.x = 3.25;
            //marker.scale.y = 3.25;
            marker.scale.z = 3.25;
            marker.color.a = 0.9;
            marker.id = i;
            
            marker.color.r = 0.8;
            marker.color.g = 0.2;
            marker.color.b = 0.2;

            marker.pose.position.x = m_connected_traffic_signals.at(i).wp_.pose.pose.position.x;
            marker.pose.position.y = m_connected_traffic_signals.at(i).wp_.pose.pose.position.y;
            marker.pose.position.z = m_connected_traffic_signals.at(i).wp_.pose.pose.position.z;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(m_connected_traffic_signals.at(i).wp_.pose.pose.orientation.z);

            std::ostringstream str_out;
            str_out << "S";
            marker.text = str_out.str();

            branch_array.markers.push_back(marker);
        }
        pub_StopLineRviz.publish(branch_array);

        if(m_connected_traffic_signals.size() != 0){
            ROS_INFO("CSV & Global Path connecting... ok");
            ROS_INFO("========== STOP LINE INFO ==========");
            for(auto& mct : m_connected_traffic_signals)
            {
                ROS_INFO("sid : %d, cid : %d, tid : %d, type : %d, cost : %f", mct.sid_, mct.cid_, mct.tid_, mct.type_, mct.wp_.cost);
            }
            connect_ok = true;
            ROS_INFO("connect_ok");
        } 
    }

    int getWaypointIndex(double x, double y, autoware_msgs::LaneArray& lanes)
    {
        m_wp_idx = 0;
        int idx = getCurrentTrajectoryIndex();
        if(idx < 0) return 0;
        int max_lane_idx = lanes.lanes.size() - 1;
        idx = std::min(idx, max_lane_idx);
        autoware_msgs::Lane lane = lanes.lanes.at(idx);
        for(int i = 0; i < lane.waypoints.size(); ++i)
        {
            if(sqrt(pow(x - lane.waypoints[i].pose.pose.position.x, 2) + pow(y - lane.waypoints[i].pose.pose.position.y, 2)) < 1.0)
            {
                return i;
            } 
        }

        return 0;
    }

    double getStoplineDistance(TrafficInfo& ti, const int& wp_idx, autoware_msgs::LaneArray& lanes)
    {
        double d = 0.0;
        double d2 = 0.0;
        int idx = getCurrentTrajectoryIndex();
        if(idx < 0) return 200.0;
        int max_lane_idx = lanes.lanes.size() - 1;
        idx = std::min(idx, max_lane_idx);
        autoware_msgs::Lane lane = lanes.lanes.at(idx);
        int max_wp_index = lane.waypoints.size() - 1;
        ti.wp_idx = std::min(ti.wp_idx, max_wp_index);
        for(int i = wp_idx; i < ti.wp_idx - 1; ++i)
        {
            d2 = hypot((lane.waypoints.at(i).pose.pose.position.x - lane.waypoints.at(i+1).pose.pose.position.x),
                        (lane.waypoints.at(i).pose.pose.position.y - lane.waypoints.at(i+1).pose.pose.position.y));
            d = d + d2;
        }
        return d;
    }
    
    std::vector<TrafficInfo> isPossible(autoware_msgs::LaneArray& lanes)
    {
        double dist;
        std::vector<TrafficInfo> temp;
        int current_wp_idx = getWaypointIndex(m_current_pose.pose.position.x, m_current_pose.pose.position.y, lanes);
        for(auto& ti : m_connected_traffic_signals)
        {
            dist = getStoplineDistance(ti, current_wp_idx, lanes);
            ti.distance = dist;
            if(m_current_speed < 3.0) m_current_speed = 3.0;
            if(ti.time_ < (dist - 3.0 / m_current_speed)) ti.signal_ = false;
            if(dist > 0.1 && dist < getFollowDistance()) temp.emplace_back(ti);
        }
        return temp;
    }

    double getFollowDistance()
    {
        double min_limit_distance = 30.0;
        double m_kph = m_current_speed * 3.6;
        double ichthus_latency_margin = 0.25 * m_current_speed * m_current_speed + 0.1 * m_current_speed; //(10m/s : 26m, 15m/s : 55m)
        double brake_distance = 0.005 * m_kph * m_kph + 0.2 * m_kph;								   // origin car braking distance (10m/s => 13.68m)
        double free_running_distance = m_current_speed * 3.0;
        double newFollowDistance = brake_distance + free_running_distance + ichthus_latency_margin;
        newFollowDistance = std::max(min_limit_distance, newFollowDistance);
        return newFollowDistance;
    }

    void testloop()
    {
        std::string cmd;
        bool cmd_;
        if(connected_ok())
        {
            std::cin >> cmd;
            if(cmd == "true")
                cmd_ = true;
            else
                cmd_ = false;
        }
    }


    bool test = false;
private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber sub_GlobalPaths;
    ros::Subscriber sub_CurrentPose;
    ros::Subscriber sub_CurrentVelocity;
    ros::Subscriber sub_Trajectory_Cost;
    ros::Subscriber sub_TrafficLightsInfo;

    ros::Publisher pub_ConnectedTrafficSignal;
    ros::Publisher pub_StopLineRviz;
    std::vector<TrafficInfo> m_connected_traffic_signals;
    std::vector<TrafficInfo> m_csv_info;
    double                     m_current_speed;
    geometry_msgs::PoseStamped m_current_pose;
    autoware_msgs::LaneArray   m_global_path;

    int m_curr_gid;
    int m_prev_gid = -1;
    int m_Current_TrajectoryIndex;
    int m_Prev_TrajectoryIndex = -1;

    bool v2x_ok = false;
    bool init_ok = false;
    bool connect_ok = false;
    bool globalpath_ok = false;
    bool current_vel_ok = false;
    bool current_pose_ok = false;
    bool local_trajectory_ok = false;

    int m_wp_idx;
    std::string path;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "traffic_signal_connector_node");
    TrafficSignalConnector tsc;
    /*
    if(tsc.test)
    {
        while (ros::ok())
        {
            ros::spinOnce();
            tsc.testloop();
        }
        
    }
    else
    {
        ros::spin();    
    }
    */
   ros::spin();
    
    return 0;
}

/*
1. global path 받음
2. csv에서 global path에 해당하는 stop line을 얻어옴 -> vector에 저장
3. connect ok : true;
4. v2x 정보에 vector에 저장되어있는 stop line들중 해당하는 stop line이 있다면 값 변경.
5. publish
*/