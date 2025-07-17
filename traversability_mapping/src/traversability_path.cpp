#include "utility.h"

class TraversabilityPath : public rclcpp::Node {

private:
    std::mutex mtx;

    rclcpp::Subscription<elevation_msgs::msg::OccupancyElevation>::SharedPtr subElevationMap;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;

    elevation_msgs::msg::OccupancyElevation elevationMap;

    float map_min[3];
    float map_max[3];

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPathCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPathLibraryValid;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPathLibraryOrigin;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGlobalPath;

    tf2_ros::Buffer tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::msg::TransformStamped transform;

    const int pathDepth = 4;
    bool planningFlag;

    const float angularVelocityMax = 7.0 / 180.0 * M_PI;
    const float angularVelocityRes = 0.5 / 180.0 * M_PI;
    const float angularVelocityMax2 = 0.5 / 180.0 * M_PI;
    const float angularVelocityRes2 = 0.25 / 180.0 * M_PI;
    const float forwardVelocity = 0.1;
    const float deltaTime = 1;
    const int simTime = 30;

    int stateListSize;
    std::vector<state_t*> stateList;
    std::vector<state_t*> pathList;

    PointType goalPoint;
    nav_msgs::msg::Path globalPath;

    pcl::PointCloud<PointType>::Ptr pathCloudLocal;
    pcl::PointCloud<PointType>::Ptr pathCloudGlobal;
    pcl::PointCloud<PointType>::Ptr pathCloudValid;
    pcl::PointCloud<PointType>::Ptr pathCloud;

    pcl::KdTreeFLANN<PointType>::Ptr kdTreeFromCloud;

    state_t *rootState;
    state_t *goalState;
    // yanzhijie
public:
    TraversabilityPath() : 
        Node("traversability_path"),
        tf_buffer(this->get_clock()),
        planningFlag(false)
    {
        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer, this);

        pubGlobalPath = this->create_publisher<nav_msgs::msg::Path>("/global_path", 5);
        pubPathCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/path_trajectory", 5);
        pubPathLibraryValid = this->create_publisher<sensor_msgs::msg::PointCloud2>("/path_library_valid", 5);
        pubPathLibraryOrigin = this->create_publisher<sensor_msgs::msg::PointCloud2>("/path_library_origin", 5);

        subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/prm_goal", 5, std::bind(&TraversabilityPath::goalPosHandler, this, std::placeholders::_1));
        subElevationMap = this->create_subscription<elevation_msgs::msg::OccupancyElevation>(
            "/occupancy_map_local_height", 5, std::bind(&TraversabilityPath::elevationMapHandler, this, std::placeholders::_1));

        pathCloud.reset(new pcl::PointCloud<PointType>());
        pathCloudLocal.reset(new pcl::PointCloud<PointType>());
        pathCloudGlobal.reset(new pcl::PointCloud<PointType>());
        pathCloudValid.reset(new pcl::PointCloud<PointType>());

        kdTreeFromCloud.reset(new pcl::KdTreeFLANN<PointType>());

        rootState = new state_t;
        goalState = new state_t;

        createPathLibrary();
    }

    void createPathLibrary(){
        // add root node
        rootState->x[0] = 0;
        rootState->x[1] = 0;
        rootState->x[2] = 0;
        rootState->theta = 0;
        rootState->stateId = stateList.size();
        rootState->cost = 0;

        stateList.push_back(rootState);
        // add nodes recursively
        createPathLibrary(rootState, 0);
        // extract nodes for visualization
        stateListSize = stateList.size();

        for (int i = 0; i < stateListSize; ++i){
            PointType p;
            p.x = stateList[i]->x[0];
            p.y = stateList[i]->x[1];
            p.z = stateList[i]->x[2];
            p.intensity = stateList[i]->stateId;

            pathCloudLocal->push_back(p);
        }
    }

    void createPathLibrary(state_t* parentState, int previousDepth){

        int thisDepth = previousDepth + 1;
        if (thisDepth > pathDepth)
            return;

        float currentAngVelMax = angularVelocityMax;
        float currentAngVelRes = angularVelocityRes;

        if (thisDepth >= 2){
            currentAngVelMax = angularVelocityMax2;
            currentAngVelRes = angularVelocityRes2;
        }

        for (float vTheta = -currentAngVelMax; vTheta <= currentAngVelMax + 1e-6; vTheta += currentAngVelRes){

            state_t *previousState = parentState;

            for (int i = 0; i < simTime; ++i){

                state_t *newState = new state_t;

                newState->x[0] = previousState->x[0] + (forwardVelocity * cos(previousState->theta)) * deltaTime;
                newState->x[1] = previousState->x[1] + (forwardVelocity * sin(previousState->theta)) * deltaTime;
                newState->x[2] = previousState->x[2];
                newState->theta = previousState->theta + vTheta * deltaTime;
                newState->cost = parentState->cost + distance(newState->x, parentState->x);
                newState->stateId = stateList.size();
                newState->parentState = previousState;

                previousState->childList.push_back(newState);

                stateList.push_back(newState);

                previousState = newState;
            }

            createPathLibrary(previousState, thisDepth);
        }        
    }

    void elevationMapHandler(const elevation_msgs::msg::OccupancyElevation::SharedPtr mapMsg){

        std::lock_guard<std::mutex> lock(mtx);

        elevationMap = *mapMsg;

        updateCostMap();

        updatePathLibrary();

        if (planningFlag == false)
            return;

        updateTrajectory();

        publishTrajectory();
    }

    void goalPosHandler(const geometry_msgs::msg::PoseStamped::SharedPtr goal){
        
        goalPoint.x = goal->pose.position.x;
        goalPoint.y = goal->pose.position.y;
        goalPoint.z = goal->pose.position.z;

        goalState->x[0] = goalPoint.x;
        goalState->x[1] = goalPoint.y;
        goalState->x[2] = goalPoint.z;
        
        // start planning
        planningFlag = true;
    }

    void updateCostMap(){

        map_min[0] = elevationMap.occupancy.info.origin.position.x; 
        map_min[1] = elevationMap.occupancy.info.origin.position.y;
        map_min[2] = elevationMap.occupancy.info.origin.position.z;
        map_max[0] = elevationMap.occupancy.info.origin.position.x + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.width; 
        map_max[1] = elevationMap.occupancy.info.origin.position.y + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.height; 
        map_max[2] = elevationMap.occupancy.info.origin.position.z;

        int sizeMap = elevationMap.occupancy.data.size();
        int inflationSize = int(costmapInflationRadius / elevationMap.occupancy.info.resolution);
        for (int i = 0; i < sizeMap; ++i) {
            int idX = int(i % elevationMap.occupancy.info.width);
            int idY = int(i / elevationMap.occupancy.info.width);
            if (elevationMap.occupancy.data[i] > 0){
                for (int m = -inflationSize; m <= inflationSize; ++m) {
                    for (int n = -inflationSize; n <= inflationSize; ++n) {
                        int newIdX = idX + m;
                        int newIdY = idY + n;
                        if (newIdX < 0 || newIdX >= elevationMap.occupancy.info.width || newIdY < 0 || newIdY >= elevationMap.occupancy.info.height)
                            continue;
                        int index = newIdX + newIdY * elevationMap.occupancy.info.width;
                        elevationMap.cost_map[index] = std::max(elevationMap.cost_map[index], std::sqrt(float(m*m+n*n)));
                    }
                }
            }
        }
    }

    void updatePathLibrary(){

        // 1. Reset valid flag
        for (int i = 0; i < stateListSize; ++i){
            stateList[i]->validFlag = true;
        }

        // 2. Transform local paths to global paths
        try{transform = tf_buffer.lookupTransform("map","base_link", rclcpp::Time(0)); } 
        catch (tf2::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return; }

        pathCloudLocal->header.frame_id = "base_link";
        pathCloudLocal->header.stamp = 0; // don't use the latest time, we don't have that transform in the queue yet

        // Transform point cloud from base_link to map frame
        pcl_ros::transformPointCloud("map", *pathCloudLocal, *pathCloudGlobal, tf_buffer);

        // 3. Collision check
        state_t *state = new state_t;
        for (int i = 0; i < stateListSize; ++i){

            if (stateList[i]->validFlag == false)
                continue;
            state->x[0] = pathCloudGlobal->points[i].x;
            state->x[1] = pathCloudGlobal->points[i].y;
            state->x[2] = pathCloudGlobal->points[i].z;

            if (isIncollision(state) == true){
                markInvalidState(stateList[i]);
            }
        }

        delete state;

        // 4. extract valid states
        pathCloudValid->clear();
        for (int i = 0; i < stateListSize; ++i){
            if (stateList[i]->validFlag == false)
                continue;
            // updateHeight(&pathCloudGlobal->points[i]);
            pathCloudValid->push_back(pathCloudGlobal->points[i]);
        }

        // 5. Visualize valid states (or paths)
        if (pubPathLibraryValid->get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*pathCloudValid, laserCloudTemp);
            laserCloudTemp.header.stamp = this->now();
            laserCloudTemp.header.frame_id = "map";
            pubPathLibraryValid->publish(laserCloudTemp);
        }

        // 6. Visualize all library states (or paths)
        if (pubPathLibraryOrigin->get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*pathCloudLocal, laserCloudTemp);
            laserCloudTemp.header.stamp = this->now();
            laserCloudTemp.header.frame_id = "base_link";
            pubPathLibraryOrigin->publish(laserCloudTemp);
        }
    }

    void updateHeight(PointType *p){
        if (p->x <= map_min[0] || p->x >= map_max[0] || p->y <= map_min[1] || p->y >= map_max[1])
            return;
        int rounded_x = (int)((p->x - map_min[0]) / mapResolution);
        int rounded_y = (int)((p->y - map_min[1]) / mapResolution);
        int index = rounded_x + rounded_y * elevationMap.occupancy.info.width;
        if (index < 0 || index >= elevationMap.occupancy.info.width * elevationMap.occupancy.info.height)
            return;
        p->z = elevationMap.height[index] + 0.2;
    }

    void updateTrajectory(){

        pathList.clear();
        
        if (pathCloudValid->size() == 0)
            return;

        // Find near valid states
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdTreeFromCloud->setInputCloud(pathCloudValid);

        double radius = 0.3; // search radius
        kdTreeFromCloud->radiusSearch(goalPoint, radius, pointSearchInd, pointSearchSqDis, 0);

        if (pointSearchInd.size() == 0)
            kdTreeFromCloud->nearestKSearch(goalPoint, 5, pointSearchInd, pointSearchSqDis);

        // Find the state with the minimum cost
        float minCost = FLT_MAX;
        state_t *minState = NULL;

        for (int i = 0; i < pointSearchInd.size(); ++i){
            int id = pathCloudValid->points[pointSearchInd[i]].intensity;
            if (stateList[id]->cost < minCost){
                minCost = stateList[id]->cost;
                minState = stateList[id];
            }
        }

        if (minState == NULL) // no path to the nearestGoalState is found
            return;

        // Extract path
        state_t *thisState = minState;
        while (thisState->parentState != NULL){
            pathList.insert(pathList.begin(), thisState);
            thisState = thisState->parentState;
        }
        pathList.insert(pathList.begin(), stateList[0]); // add current robot state
    }

    void publishTrajectory(){

        if (pubPathCloud->get_subscription_count() != 0){
            pathCloud->clear();
            for (int i = 0; i < pathList.size(); ++i){
                PointType p = pathCloudGlobal->points[pathList[i]->stateId];
                p.z += 0.1;
                p.intensity = pathList[i]->cost;
                pathCloud->push_back(p);
            }
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*pathCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = this->now();
            laserCloudTemp.header.frame_id = "map";
            pubPathCloud->publish(laserCloudTemp);
        }

        // even no feasible path is found, publish an empty path
        globalPath.poses.clear();
        for (int i = 0; i < pathList.size(); i++){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = pathCloudGlobal->points[pathList[i]->stateId].x;
            pose.pose.position.y = pathCloudGlobal->points[pathList[i]->stateId].y;
            pose.pose.position.z = pathCloudGlobal->points[pathList[i]->stateId].z;
            tf2::Quaternion q;
            q.setRPY(0, 0, 0); // yaw = 0
            pose.pose.orientation = tf2::toMsg(q);
            globalPath.poses.push_back(pose);
        }

        // publish path
        globalPath.header.frame_id = "map";
        globalPath.header.stamp = this->now();
        pubGlobalPath->publish(globalPath);

        planningFlag = false;
    }


    void markInvalidState(state_t* state){
        state->validFlag = false;
        int childListSize = state->childList.size();
        for (int i = 0; i < childListSize; ++i){
            markInvalidState(state->childList[i]);
        }
    }

    // Collision check (using state for input)
    bool isIncollision(state_t* stateIn){
        // if the state is outside the map, discard this state
        if (stateIn->x[0] <= map_min[0] || stateIn->x[0] >= map_max[0] 
            || stateIn->x[1] <= map_min[1] || stateIn->x[1] >= map_max[1])
            return false;
        // if the distance to the nearest obstacle is less than xxx, in collision
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        int index = rounded_x + rounded_y * elevationMap.occupancy.info.width;

        // close to obstacles within ... m
        if (elevationMap.cost_map[index] > 0)
            return true;
        
        return false;
    }

    float distance(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) + 
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]) +
                    (state_to[2]-state_from[2])*(state_to[2]-state_from[2]));
    }

    void publishPath(){
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*pathCloudValid, laserCloudTemp);
        laserCloudTemp.header.stamp = this->now();
        laserCloudTemp.header.frame_id = "base_link";
        pubPathLibraryValid->publish(laserCloudTemp);
    }
};


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    
    auto tPath = std::make_shared<TraversabilityPath>();

    RCLCPP_INFO(tPath->get_logger(), "\033[1;32m---->\033[0m Traversability Planner Started.");

    rclcpp::spin(tPath);

    return 0;
}