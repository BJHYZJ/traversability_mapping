#include "utility.h"

class TraversabilityPRM : public rclcpp::Node, public std::enable_shared_from_this<TraversabilityPRM> {
private:
    // tf2
    tf2_ros::Buffer tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::msg::TransformStamped transform;

    // ROS2 publishers/subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPRMGraph;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPRMPath;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGlobalPath;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubSingleSourcePaths;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudPRMNodes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudPRMGraph;
    
    rclcpp::Subscription<elevation_msgs::msg::OccupancyElevation>::SharedPtr subElevationMap;

    elevation_msgs::msg::OccupancyElevation elevationMap;
    float map_min[3];
    float map_max[3];
    ///////////// Planner ////////////
    vector<state_t*> nodeList;
    vector<state_t*> pathList;
    nav_msgs::msg::Path globalPath;
    nav_msgs::msg::Path displayGlobalPath;
    double start_time;
    double finish_time;
    bool planningFlag;
    state_t *robotState;
    state_t *goalState;
    state_t *nearestGoalState;
    state_t *mapCenter;
    kdtree_t *kdtree;
    bool costUpdateFlag[NUM_COSTS];
    std::mutex mtx;

public:
    TraversabilityPRM() : 
        Node("traversability_prm"), 
        tf_buffer(this->get_clock()), 
        planningFlag(false) 
    {
        robotState = new state_t;
        goalState = new state_t;
        mapCenter = new state_t;
        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer, this);

        subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/prm_goal", rclcpp::QoS(5),
            std::bind(&TraversabilityPRM::goalPosHandler, this, std::placeholders::_1));
            
        subElevationMap = this->create_subscription<elevation_msgs::msg::OccupancyElevation>(
            "/occupancy_map_local_height", 5,
            std::bind(&TraversabilityPRM::elevationMapHandler, this, std::placeholders::_1));

        pubPRMGraph = this->create_publisher<visualization_msgs::msg::MarkerArray>("/prm_graph", rclcpp::QoS(5));
        pubPRMPath = this->create_publisher<visualization_msgs::msg::MarkerArray>("/prm_path", rclcpp::QoS(5));
        pubSingleSourcePaths = this->create_publisher<visualization_msgs::msg::MarkerArray>("/prm_single_source_paths", 5);
        pubCloudPRMNodes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/prm_cloud_nodes", rclcpp::QoS(5));
        pubCloudPRMGraph = this->create_publisher<sensor_msgs::msg::PointCloud2>("/prm_cloud_graph", rclcpp::QoS(5));
        pubGlobalPath = this->create_publisher<nav_msgs::msg::Path>("/global_path", rclcpp::QoS(5));

        allocateMemory();
    }

    ~TraversabilityPRM() {}

    void allocateMemory() {
        kdtree = kd_create(3);
        for (int i = 0; i < NUM_COSTS; ++i)
            costUpdateFlag[i] = false;
        for (int i = 0; i < costHierarchy.size(); ++i)
            costUpdateFlag[costHierarchy[i]] = true;
    }

    void elevationMapHandler(const elevation_msgs::msg::OccupancyElevation::SharedPtr mapMsg) {
        std::lock_guard<std::mutex> lock(mtx);
        elevationMap = *mapMsg;
        updateMapBoundary();
        updateCostMap();
        buildRoadMap();
    }

    void updateMapBoundary() {
        map_min[0] = elevationMap.occupancy.info.origin.position.x;
        map_min[1] = elevationMap.occupancy.info.origin.position.y;
        map_min[2] = elevationMap.occupancy.info.origin.position.z;
        map_max[0] = elevationMap.occupancy.info.origin.position.x + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.width;
        map_max[1] = elevationMap.occupancy.info.origin.position.y + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.height;
        map_max[2] = elevationMap.occupancy.info.origin.position.z;
    }

    void updateCostMap() {
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

    void buildRoadMap() {
        // 1. generate samples
        generateSamples();
        // 2. Add edges and update state height if map is changed
        updateStatesAndEdges();
        // 3. Planning
        bfsSearch();
        // 4. Visualize Roadmap
        publishPRM();
        // 5. Publish path to move_base and stop planning
        publishPathStop();
        // 6. Convert PRM Graph into point cloud for external usage
        publishRoadmap2Cloud();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Planner /////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void goalPosHandler(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        goalState->x[0] = goal->pose.position.x;
        goalState->x[1] = goal->pose.position.y;
        goalState->x[2] = goal->pose.position.z;
        planningFlag = true;
    }

    bool bfsSearch() {
        pathList.clear();
        globalPath.poses.clear();
        // 0. Planning or not
        if (planningFlag == false)
            return true;
        // 1. reset costs and parents
        for (int i = 0; i < nodeList.size(); ++i){
            for (int j = 0; j < NUM_COSTS; ++j)
                nodeList[i]->costsToRoot[j] = FLT_MAX;
            nodeList[i]->parentState = NULL;
        }
        // 2. find the state that is the closest to the robot
        state_t *startState = NULL;

        vector<state_t*> nearRobotStates;
        getNearStates(robotState, nearRobotStates, 2);
        if (nearRobotStates.size() == 0)
            return false;

        float nearRobotDist = FLT_MAX;
        for (int i = 0; i < nearRobotStates.size(); ++i){
            float dist = distance(nearRobotStates[i]->x, robotState->x);
            if (dist < nearRobotDist && nearRobotStates[i]->neighborList.size() != 0){
                nearRobotDist = dist;
                startState = nearRobotStates[i];
            }
        }

        if (startState == NULL || startState->neighborList.size() == 0)
            return false;

        for (int i = 0; i < NUM_COSTS; ++i)
            startState->costsToRoot[i] = 0;
        // 3. BFS search
        float thisCost;
        vector<state_t*> Queue;
        Queue.push_back(startState);

        while(Queue.size() > 0 && rclcpp::ok()){
            // find the state that can offer lowest cost in this depth and remove it from Queue
            state_t *fromState = minCostStateInQueue(Queue);
            Queue.erase(remove(Queue.begin(), Queue.end(), fromState), Queue.end());
            // stop searching if minimum cost path to goal is found
            // if (fromState == nearestGoalState)
            //     break;
            // lopp through all neighbors of this state
            for (int i = 0; i < fromState->neighborList.size(); ++i){
                state_t *toState = fromState->neighborList[i].neighbor;
                // Loop through cost hierarchy
                for (vector<int>::const_iterator iter = costHierarchy.begin(); iter != costHierarchy.end(); iter++){
                    int costIndex = *iter;

                    thisCost = fromState->costsToRoot[costIndex] + fromState->neighborList[i].edgeCosts[costIndex];
                    // If cost can be improved, update this ndoe with new costs
                    if (thisCost < toState->costsToRoot[costIndex]){
                        updateCosts(fromState, toState, i); // update toState's costToRoot
                        toState->parentState = fromState; // change parent for toState
                        Queue.push_back(toState);
                    }
                    // If cost is same, go to compare secondary cost
                    else if (thisCost == toState->costsToRoot[costIndex]){
                        continue;
                    }
                    // If cost becomes higher, abort this propagation
                    else
                        break;
                }
            }
        }

        // 4. Find valid nearestGoalState in PRM
        nearestGoalState = getNearestState(goalState);
        // find near states to nearestGoalState
        vector<state_t*> nearGoalStates;
        getNearStates(nearestGoalState, nearGoalStates, 20);
        if (nearGoalStates.size() > 0){
            float nearRobotDist = FLT_MAX;
            for (int i = 0; i < nearGoalStates.size(); ++i){
                float dist = distance(nearGoalStates[i]->x, goalState->x);
                if (dist < nearRobotDist && nearGoalStates[i]->parentState != NULL){
                    nearRobotDist = dist;
                    nearestGoalState = nearGoalStates[i];
                }
            }
        }
        // the nearest goal state is invalid
        if (nearestGoalState == NULL || nearestGoalState->parentState == NULL) // no path to the nearestGoalState is found
            return false;

        // 4. Extract path
        state_t *thisState = nearestGoalState;
        while (thisState->parentState != NULL){
            pathList.insert(pathList.begin(), thisState);
            thisState = thisState->parentState;
        }
        pathList.insert(pathList.begin(), robotState); // add current robot state
        // pathList.push_back(goalState); // add goal state

        // 5. Smooth path
        smoothPath();

        return true;
    }

    state_t* minCostStateInQueue(vector<state_t*> Queue){
        // Loop through cost hierarchy
        for (vector<int>::const_iterator iter1 = costHierarchy.begin(); iter1 != costHierarchy.end(); ++iter1){
            int costIndex = *iter1;
            vector<state_t*> tempQueue;
            float minCost = FLT_MAX;
            // loop through nodes saved in Queue
            for (vector<state_t*>::const_iterator iter2 = Queue.begin(); iter2 != Queue.end(); ++iter2){
                state_t* thisState = *iter2;
                // if cost is lower, we put it in tempQueue in case other nodes can offer same cost
                if (thisState->costsToRoot[costIndex] < minCost){
                    minCost = thisState->costsToRoot[costIndex];
                    tempQueue.clear();
                    tempQueue.push_back(thisState);
                }
                // same cost can be offered by other nodes, we save them to tempQueue for next loop (secondary cost)
                else if (thisState->costsToRoot[costIndex] == minCost)
                    tempQueue.push_back(thisState);
            }
            // Queue is used again for next loop
            Queue.clear();
            Queue = tempQueue;
        }
        // If cost hierarchy is selected correctly, there will be only one element left in Queue (no other ties)
        return Queue[0];
    }

    void updateCosts(state_t* fromState, state_t* toState, int neighborInd){
        for (int i = 0; i < NUM_COSTS; ++i)
            toState->costsToRoot[i] = fromState->costsToRoot[i] + fromState->neighborList[neighborInd].edgeCosts[i];
    }

    void smoothPath() {
        if (pathList.size() <= 1)
            return;
        // Cubic Spline
        nav_msgs::msg::Path originPath;
        nav_msgs::msg::Path splinePath;

        originPath.header.frame_id = "map";
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";

        originPath.poses.clear();

        for (int i = 0; i < pathList.size(); i++){
            pose.pose.position.x = pathList[i]->x[0];
            pose.pose.position.y = pathList[i]->x[1];
            pose.pose.position.z = pathList[i]->x[2];
            tf2::Quaternion q;
            q.setRPY(0, 0, 0); // yaw = 0
            pose.pose.orientation = tf2::toMsg(q);
            originPath.poses.push_back(pose);
        }

        path_smoothing::CubicSplineInterpolator csi("smooth");
        csi.interpolatePath(originPath, splinePath);

        globalPath = splinePath;
        displayGlobalPath = globalPath; // displayGlobalPath is only changed during planning
    }

    void generateSamples() {
        double sampling_start_time = rclcpp::Clock().now().seconds();
        while (rclcpp::Clock().now().seconds() - sampling_start_time < 0.002 && rclcpp::ok()){

            state_t* newState = new state_t;

            if (sampleState(newState)){
                // 1.1 Too close discard
                if (nodeList.size() != 0 && stateTooClose(newState) == true){
                    delete newState;
                    continue;
                }
                // 1.2 Save new state and insert to KD-tree
                newState->stateId = nodeList.size(); // mark state ID
                nodeList.push_back(newState);
                insertIntoKdtree(newState);
            }
            else
                delete newState;
        }
    }

    bool stateTooClose(state_t *stateIn){
        // restrict the nearest sample that can be inserted to the roadmap
        state_t *nearestState = getNearestState(stateIn);
        if (nearestState == NULL)
            return false;

        if (distance(stateIn->x, nearestState->x) > neighborSampleRadius
            && abs(stateIn->x[2] - nearestState->x[2]) <= neighborConnectHeight)
            return false;

        return true;
    }

    void updateStatesAndEdges() {
        getRobotState();
        // 0. find local map center
        mapCenter->x[0] = (map_min[0] + map_max[0]) / 2;
        mapCenter->x[1] = (map_min[1] + map_max[1]) / 2;
        mapCenter->x[2] = robotState->x[2];
        // 1. add edges for the nodes that are within a certain radius of mapCenter
        vector<state_t*> nearStates;
        getNearStates(mapCenter, nearStates, neighborSearchRadius);
        if (nearStates.size() == 0)
            return;
        // 3. update states height values (because the map is changing all the time)
        for (int i = 0; i < nearStates.size(); ++i){
            float thisHeight = getStateHeight(nearStates[i]);
            if (thisHeight != -FLT_MAX) // new height can be -FLT_MAX since the map is shifted a bit every time (round error)
                nearStates[i]->x[2]  = thisHeight;
        }
        // 4. loop through all neighbors
        float edgeCosts[NUM_COSTS];
        neighbor_t thisNeighbor;
        for (int i = 0; i < nearStates.size(); ++i){
            for (int j = i+1; j < nearStates.size(); ++j){
                // 4.1 height difference larger than threshold, too steep to connect
                if (abs(nearStates[i]->x[2] - nearStates[j]->x[2]) > neighborConnectHeight){
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }
                // 4.2 distance larger than x, too far to connect
                float distanceBetween = distance(nearStates[i]->x, nearStates[j]->x);
                if (distanceBetween > neighborConnectRadius || distanceBetween < 0.3){
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }
                // 4.3 this edge is connectable
                if(edgePropagation(nearStates[i], nearStates[j], edgeCosts) == true){
                    // even if edge already exists, we still need to update costs (casuse elevation may change)
                    deleteEdge(nearStates[i], nearStates[j]);
                    for (int k = 0; k < NUM_COSTS; ++k)
                        thisNeighbor.edgeCosts[k] = edgeCosts[k];

                    thisNeighbor.neighbor = nearStates[j];
                    nearStates[i]->neighborList.push_back(thisNeighbor);
                    thisNeighbor.neighbor = nearStates[i];
                    nearStates[j]->neighborList.push_back(thisNeighbor);
                }else{ // edge is not connectable, delete old edge if it exists
                    deleteEdge(nearStates[i], nearStates[j]);
                }
            }
        }
    }

    void deleteEdge(state_t* stateA, state_t* stateB) {
        // "remove" compacts the elements that differ from the value to be removed (state_in) in the beginning of the vector
        // and returns the iterator to the first element after that range. Then "erase" removes these elements (who's value is unspecified).
        compareState = stateB;
        stateA->neighborList.erase(std::remove_if(stateA->neighborList.begin(), stateA->neighborList.end(), isStateExsiting), stateA->neighborList.end());
        compareState = stateA;
        stateB->neighborList.erase(std::remove_if(stateB->neighborList.begin(), stateB->neighborList.end(), isStateExsiting), stateB->neighborList.end());
    }

    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]) {
        // 0. initialize edgeCosts
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = 0;
        // 1. segment the edge for collision checking
        int steps = floor(distance(state_from->x, state_to->x) / (mapResolution));
        float stepX = (state_to->x[0]-state_from->x[0]) / steps;
        float stepY = (state_to->x[1]-state_from->x[1]) / steps;
        float stepZ = (state_to->x[2]-state_from->x[2]) / steps;
        // 2. allocate memory for a state, this state must be deleted after collision checking
        state_t *stateCurr = new state_t;;
        stateCurr->x[0] = state_from->x[0];
        stateCurr->x[1] = state_from->x[1];
        stateCurr->x[2] = state_from->x[2];

        int rounded_x, rounded_y, indexInLocalMap;

        // 3. collision checking loop
        for (int stepCount = 0; stepCount < steps; ++stepCount){
            stateCurr->x[0] += stepX;
            stateCurr->x[1] += stepY;
            stateCurr->x[2] += stepZ;

            rounded_x = (int)((stateCurr->x[0] - map_min[0]) / mapResolution);
            rounded_y = (int)((stateCurr->x[1] - map_min[1]) / mapResolution);
            indexInLocalMap = rounded_x + rounded_y * elevationMap.occupancy.info.width;

            if (isIncollision(rounded_x, rounded_y, indexInLocalMap)){
                delete stateCurr;
                return false;
            }

            // costs propagation
            if (costUpdateFlag[2])
                edgeCosts[2] = edgeCosts[2] + mapResolution; // distance cost
        }
        delete stateCurr;
        return true;
    }

    // Collision check (using rounded index for input)
    bool isIncollision(int rounded_x, int rounded_y, int index) {
        if (rounded_x < 0 || rounded_x >= localMapArrayLength ||
            rounded_y < 0 || rounded_y >= localMapArrayLength )
            return true;

        // close to obstacles within ... m
        if (elevationMap.cost_map[index] > 0)
                return true;

        if (planningUnknown == false){
            // stateIn->x is on an unknown grid
            if (elevationMap.height[index] == -FLT_MAX)
                return true;
        }

        return false;
    }

    void getNearStates(state_t *stateIn, vector<state_t*>& vectorNearStatesOut, double radius) {
        kdres_t *kdres = kd_nearest_range (kdtree, stateIn->x, radius);
        vectorNearStatesOut.clear();
        // Create the vector data structure for storing the results
        int numNearVertices = kd_res_size (kdres);
        if (numNearVertices == 0) {
            kd_res_free (kdres);
            return;
        }
        // Place pointers to the near vertices into the vector
        kd_res_rewind (kdres);
        while (!kd_res_end(kdres)) {
            state_t *stateCurr = (state_t *) kd_res_item_data (kdres);
            vectorNearStatesOut.push_back(stateCurr);
            kd_res_next(kdres);
        }
        // Free temporary memory
        kd_res_free (kdres);
    }

    bool sampleState(state_t *stateCurr) {
        // random x and y
        for (int i = 0; i < 2; ++i)
            stateCurr->x[i] = (double)rand()/(RAND_MAX + 1.0)*(map_max[i] - map_min[i])
                - (map_max[i] - map_min[i])/2.0 + (map_max[i] + map_min[i])/2.0;
        // random heading
        stateCurr->theta = (double)rand()/(RAND_MAX + 1.0) * 2 * M_PI - M_PI;
        // collision checking before getting height info
        if (isIncollision(stateCurr))
            return false;
        // z is not random since the robot is constrained to move on the ground
        stateCurr->x[2] = getStateHeight(stateCurr);
        if (stateCurr->x[2] == -FLT_MAX)
            return false;

        return true;
    }

    double getStateHeight(state_t* stateIn) {
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        return elevationMap.height[rounded_x + rounded_y * elevationMap.occupancy.info.width];
    }

    // Collision check (using state for input)
    bool isIncollision(state_t* stateIn) {
        // if the state is outside the map, discard this state
        if (stateIn->x[0] <= map_min[0] || stateIn->x[0] >= map_max[0]
            || stateIn->x[1] <= map_min[1] || stateIn->x[1] >= map_max[1])
            return true;
        // if the distance to the nearest obstacle is less than xxx, in collision
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        int index = rounded_x + rounded_y * elevationMap.occupancy.info.width;

        // close to obstacles within ... m
        if (elevationMap.cost_map[index] > 0)
            return true;

        return false;
    }

    void insertIntoKdtree(state_t *stateCurr) {
        kd_insert(kdtree, stateCurr->x, stateCurr);
    }

    state_t* getNearestState(state_t *stateIn) {
        kdres_t *kdres = kd_nearest(kdtree, stateIn->x);
        if (kd_res_end (kdres)){
            kd_res_free (kdres);
            return NULL;
        }
        state_t* nearestState = (state_t*) kd_res_item_data(kdres);
        kd_res_free (kdres);
        return nearestState;
    }

    // Calculate the Euclidean distance between two samples (2D)
    float distance(double state_from[3], double state_to[3]) {
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) +
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]) +
                    (state_to[2]-state_from[2])*(state_to[2]-state_from[2]));
    }

    void publishPRM() {
        if (pubPRMPath->get_subscription_count() != 0){

            visualization_msgs::msg::MarkerArray markerArray;
            geometry_msgs::msg::Point p;

            // path visualization
            visualization_msgs::msg::Marker markerPath;
            markerPath.header.frame_id = "map";
            markerPath.header.stamp = rclcpp::Clock().now();
            markerPath.action = visualization_msgs::msg::Marker::ADD;
            markerPath.type = visualization_msgs::msg::Marker::LINE_STRIP;
            markerPath.ns = "path";
            markerPath.id = 0;

            markerPath.scale.x = 0.2;
            markerPath.scale.y = 0.2;
            markerPath.scale.z = 0.2;

            markerPath.color.r = 0.0;
            markerPath.color.g = 0;
            markerPath.color.b = 1.0;
            markerPath.color.a = 1.0;

            markerPath.pose.orientation.x = 0.0;
            markerPath.pose.orientation.y = 0.0;
            markerPath.pose.orientation.z = 0.0;
            markerPath.pose.orientation.w = 1.0;

            for (const auto &stamped : displayGlobalPath.poses) {
                p.x = stamped.pose.position.x;
                p.y = stamped.pose.position.y;
                p.z = stamped.pose.position.z + 0.3;
                markerPath.points.push_back(p);
            }

            if (markerPath.points.size() >= 2) {
                markerArray.markers.push_back(markerPath);
            } else {
                visualization_msgs::msg::Marker del;
                del.header.frame_id = "map";
                del.header.stamp    = rclcpp::Clock().now();
                del.ns              = "path";
                del.id              = 0;
                del.action          = visualization_msgs::msg::Marker::DELETE;
                markerArray.markers.push_back(del);
            }

            // goal point visualization
            visualization_msgs::msg::Marker markerGoal;
            markerGoal.header.frame_id = "map";
            markerGoal.header.stamp = rclcpp::Clock().now();
            markerGoal.action = visualization_msgs::msg::Marker::ADD;
            markerGoal.type= visualization_msgs::msg::Marker::SPHERE_LIST;
            markerGoal.ns = "goal";
            markerGoal.id = 1;

            markerGoal.scale.x = 0.5;
            markerGoal.scale.y = 0.5;
            markerGoal.scale.z = 0.5;

            markerGoal.color.r = 0.0;
            markerGoal.color.g = 0.0;
            markerGoal.color.b = 1.0;
            markerGoal.color.a = 1.0;

            markerGoal.pose.orientation.x = 0.0;
            markerGoal.pose.orientation.y = 0.0;
            markerGoal.pose.orientation.z = 0.0;
            markerGoal.pose.orientation.w = 1.0;

            if (!displayGlobalPath.poses.empty()) {
                const auto &back = displayGlobalPath.poses.back().pose.position;
                p.x = back.x;
                p.y = back.y;
                p.z = back.z + 0.3;
                markerGoal.points.push_back(p);
            }


            if (!markerGoal.points.empty()) {
                markerArray.markers.push_back(markerGoal);
            } else {
                visualization_msgs::msg::Marker del;
                del.header.frame_id = "map";
                del.header.stamp    = rclcpp::Clock().now();
                del.ns              = "goal";
                del.id              = 1;
                del.action          = visualization_msgs::msg::Marker::DELETE;
                markerArray.markers.push_back(del);
            }

            pubPRMPath->publish(markerArray);
        }

        if (pubPRMGraph->get_subscription_count() != 0){

            visualization_msgs::msg::MarkerArray markerArray;
            geometry_msgs::msg::Point p;

            // PRM nodes visualization
            visualization_msgs::msg::Marker markerNode;
            markerNode.header.frame_id = "map";
            markerNode.header.stamp = rclcpp::Clock().now();
            markerNode.action = visualization_msgs::msg::Marker::ADD;
            markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            markerNode.ns = "nodes";
            markerNode.id = 2;
            markerNode.scale.x = 0.2;
            markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
            markerNode.color.a = 1;

            for (int i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.13;
                markerNode.points.push_back(p);
            }

            // PRM edge visualization
            visualization_msgs::msg::Marker markerEdge;
            markerEdge.header.frame_id = "map";
            markerEdge.header.stamp = rclcpp::Clock().now();
            markerEdge.action = visualization_msgs::msg::Marker::ADD;
            markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
            markerEdge.ns = "edges";
            markerEdge.id = 3;
            markerEdge.scale.x = 0.05;
            markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
            markerEdge.color.a = 1;

            for (int i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;
                int numNeighbors = nodeList[i]->neighborList.size();
                for (int j = 0; j < numNeighbors; ++j){
                    p.x = nodeList[i]->x[0];
                    p.y = nodeList[i]->x[1];
                    p.z = nodeList[i]->x[2] + 0.1;
                    markerEdge.points.push_back(p);
                    p.x = nodeList[i]->neighborList[j].neighbor->x[0];
                    p.y = nodeList[i]->neighborList[j].neighbor->x[1];
                    p.z = nodeList[i]->neighborList[j].neighbor->x[2] + 0.1;
                    markerEdge.points.push_back(p);
                }
            }

            // push to markerarray and publish
            markerArray.markers.push_back(markerNode);
            markerArray.markers.push_back(markerEdge);
            pubPRMGraph->publish(markerArray);

        }

        // 4. Single Source Shortest Paths
        if (pubSingleSourcePaths->get_subscription_count() != 0){

            visualization_msgs::msg::MarkerArray markerArray;
            geometry_msgs::msg::Point p;

             // publish empty single-source paths
            if (planningFlag == false){
                pubSingleSourcePaths->publish(markerArray);
                return;
            }

            // single source path visualization
            visualization_msgs::msg::Marker markersPath;
            markersPath.header.frame_id = "map";
            markersPath.header.stamp = rclcpp::Clock().now();
            markersPath.action = visualization_msgs::msg::Marker::ADD;
            markersPath.type = visualization_msgs::msg::Marker::LINE_LIST;
            markersPath.ns = "path";
            markersPath.id = 4;
            markersPath.scale.x = 0.05;
            markersPath.color.r = 0.3; markersPath.color.g = 0; markersPath.color.b = 1.0;
            markersPath.color.a = 1.0;

            for (int i = 0; i < nodeList.size(); ++i){
                if (nodeList[i]->parentState == NULL)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.2;
                markersPath.points.push_back(p);
                p.x = nodeList[i]->parentState->x[0];
                p.y = nodeList[i]->parentState->x[1];
                p.z = nodeList[i]->parentState->x[2]+0.2;
                markersPath.points.push_back(p);
            }
            // push to markerarray and publish
            markerArray.markers.push_back(markersPath);
            pubSingleSourcePaths->publish(markerArray);
        }
    }

    void publishPathStop() {
        // even no feasible path is found, publish an empty path
        globalPath.header.frame_id = "map";
        globalPath.header.stamp = rclcpp::Clock().now();
        // publish path
        pubGlobalPath->publish(globalPath);
        // stop planning
        planningFlag = false;
    }

    void publishRoadmap2Cloud() {
        if (pubCloudPRMNodes->get_subscription_count() == 0 && pubCloudPRMGraph->get_subscription_count() == 0)
            return;

        int sizeCloud = nodeList.size();
        // PRM nodes to point cloud
        PointType thisPoint;
        pcl::PointCloud<PointType> nodeCloud; nodeCloud.resize(sizeCloud);
        pcl::PointCloud<PointType> adjacencyCloud; adjacencyCloud.resize(sizeCloud*sizeCloud);
        for (int i = 0; i < sizeCloud; ++i){
            // save node
            thisPoint.x = nodeList[i]->x[0];
            thisPoint.y = nodeList[i]->x[1];
            thisPoint.z = nodeList[i]->x[2]+0.15;
            thisPoint.intensity = i;
            nodeCloud.points[i] = thisPoint;
            // extract adjacency matrix into cloud
            int numNeighbors = nodeList[i]->neighborList.size();
            for (int j = 0; j < numNeighbors; ++j){
                int index = nodeList[i]->stateId + sizeCloud * nodeList[i]->neighborList[j].neighbor->stateId;
                adjacencyCloud.points[index].intensity = 1;
            }
        }
        // Publish
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(nodeCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = rclcpp::Clock().now();
        pubCloudPRMNodes->publish(laserCloudTemp);
        pcl::toROSMsg(adjacencyCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = rclcpp::Clock().now();
        pubCloudPRMGraph->publish(laserCloudTemp);
    }

    void getRobotState() {
        try {
            transform = tf_buffer.lookupTransform("map", "base_link", rclcpp::Time(0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform Failure: %s", ex.what());
            return;
        }
        robotState->x[0] = transform.transform.translation.x;
        robotState->x[1] = transform.transform.translation.y;
        robotState->x[2] = transform.transform.translation.z;

        double roll, pitch, yaw;
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        robotState->theta = yaw + M_PI; // change from -PI~PI to 0~1*PI
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TraversabilityPRM>();
    RCLCPP_INFO(node->get_logger(), "----> Traversability Planner Started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
