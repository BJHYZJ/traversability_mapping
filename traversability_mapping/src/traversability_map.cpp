#include "utility.h"

using std::placeholders::_1;

class TraversabilityMapping : public rclcpp::Node
{

private:
    // Mutex Memory Lock
    std::mutex mtx;
    // Transform Listener
    // tf::TransformListener listener;
    // tf::StampedTransform transform;

    tf2_ros::Buffer tf_buffer;
    // tf2_ros::TransformListener tf_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::msg::TransformStamped transform;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subFilteredGroundCloud;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancyMapLocal;
    rclcpp::Publisher<elevation_msgs::msg::OccupancyElevation>::SharedPtr pubOccupancyMapLocalHeight;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubElevationCloud;

    pcl::PointCloud<PointType>::Ptr laserCloud;
    pcl::PointCloud<PointType>::Ptr laserCloudElevation;

    nav_msgs::msg::OccupancyGrid occupancyMap2D;
    elevation_msgs::msg::OccupancyElevation occupancyMap2DHeight;

    int pubCount;

    // Map Arrays
    int mapArrayCount;
    std::vector<std::vector<int>> mapArrayInd;
    std::vector<std::vector<int>> predictionArrayFlag;
    std::vector<std::unique_ptr<childMap_t>> mapArray;

    // Local Map Extraction
    PointType robotPoint;
    PointType localMapOriginPoint;
    grid_t localMapOriginGrid;

    // Global Variables for Traversability Calculation
    cv::Mat matCov, matEig, matVec;

    // Lists for New Scan
    std::vector<mapCell_t *> observingList1;
    std::vector<mapCell_t *> observingList2;

    rclcpp::TimerBase::SharedPtr timer_;

public:
    TraversabilityMapping() : Node("traversability_mapping"),
                              tf_buffer(this->get_clock()),
                              tf_listener(std::make_unique<tf2_ros::TransformListener>(tf_buffer, this)), // [AI修改1]
                              pubCount(1), // [AI修改2]
                              mapArrayCount(0) // [AI修改2]
    {
        // subscribe to traversability filter

        subFilteredGroundCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/filtered_pointcloud",
            rclcpp::QoS(5),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
              this->cloudHandler(msg);
            }
        );

        pubOccupancyMapLocal = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/occupancy_map_local", rclcpp::QoS(5));

        pubOccupancyMapLocalHeight = this->create_publisher<elevation_msgs::msg::OccupancyElevation>(
            "/occupancy_map_local_height", rclcpp::QoS(5));

        pubElevationCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/elevation_pointcloud", rclcpp::QoS(5));

        allocateMemory();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TraversabilityMapping::TraversabilityThread, this)
        );

        RCLCPP_INFO(this->get_logger(), "Traversability Mapping Started.");
    }

    ~TraversabilityMapping()
    {
    }

    void allocateMemory()
    {
        // allocate memory for point cloud
        laserCloud.reset(new pcl::PointCloud<PointType>());
        laserCloudElevation.reset(new pcl::PointCloud<PointType>());

        mapArrayInd.assign(mapArrayLength, std::vector<int>(mapArrayLength, -1));
        predictionArrayFlag.assign(mapArrayLength, std::vector<int>(mapArrayLength, 0));
        mapArray.clear();

        // Matrix Initialization
        matCov = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
        matEig = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
        matVec = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

        initializeLocalOccupancyMap();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// Register Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &laserCloudMsg)
    {
        // ===================== [AI修改5] 共享数据加锁 =====================
        std::lock_guard<std::mutex> lock(mtx); // [AI修改5]
        // Get Robot Position
        if (!getRobotPosition())
            return;
        // Convert Point Cloud
        pcl::fromROSMsg(*laserCloudMsg, *laserCloud);
        // Register New Scan
        updateElevationMap();
        // publish local occupancy grid map
        publishMap();
    }

    void updateElevationMap()
    {
        // int cloudSize = laserCloud->points.size();
        int cloudSize = static_cast<int>(laserCloud->points.size());
        RCLCPP_INFO(this->get_logger(), "[DEBUG] cloudSize: %d", cloudSize);
        for (int i = 0; i < cloudSize; ++i)
        {
            laserCloud->points[i].z -= 0.2f; // for visualization
            updateElevationMap(&laserCloud->points[i]);
        }
    }

    void updateElevationMap(PointType *point)
    {
        // Find point index in global map
        grid_t thisGrid;
        if (!findPointGridInMap(&thisGrid, point))
            return;
        // Get current cell pointer
        mapCell_t *thisCell = grid2Cell(&thisGrid);
        // update elevation
        updateCellElevation(thisCell, point);
        // update occupancy
        updateCellOccupancy(thisCell, point);
        // update observation time
        updateCellObservationTime(thisCell);
    }

    void updateCellObservationTime(mapCell_t *thisCell)
    {
        ++thisCell->observeTimes;
        if (thisCell->observeTimes >= traversabilityObserveTimeTh)
            observingList1.push_back(thisCell);
    }

    void updateCellOccupancy(mapCell_t *thisCell, PointType *point)
    {
        // Update log_odds
        float p; // Probability of being occupied knowing current measurement.
        if (point->intensity == 100)
            p = p_occupied_when_laser;
        else
            p = p_occupied_when_no_laser;
        thisCell->log_odds += std::log(p / (1 - p));

        if (thisCell->log_odds < -large_log_odds)
            thisCell->log_odds = -large_log_odds;
        else if (thisCell->log_odds > large_log_odds)
            thisCell->log_odds = large_log_odds;
        // Update occupancy
        float occupancy;
        if (thisCell->log_odds < -max_log_odds_for_belief)
            occupancy = 0;
        else if (thisCell->log_odds > max_log_odds_for_belief)
            occupancy = 100;
        else
            occupancy = (int)(lround((1 - 1 / (1 + std::exp(thisCell->log_odds))) * 100));
        // update cell
        thisCell->updateOccupancy(occupancy);
    }

    void updateCellElevation(mapCell_t *thisCell, PointType *point)
    {
        // Kalman Filter: update cell elevation using Kalman filter
        // https://www.cs.cornell.edu/courses/cs4758/2012sp/materials/MI63slides.pdf

        // cell is observed for the first time, no need to use Kalman filter
        if (thisCell->elevation == -FLT_MAX)
        {
            thisCell->elevation = point->z;
            thisCell->elevationVar = pointDistance(robotPoint, *point);
            return;
        }

        // Predict:
        float x_pred = thisCell->elevation;           // x = F * x + B * u
        float P_pred = thisCell->elevationVar + 0.01; // P = F*P*F + Q
        // Update:
        float R_factor = (thisCell->observeTimes > 20) ? 10 : 1;
        float R = pointDistance(robotPoint, *point) * R_factor; // measurement noise: R, scale it with dist and observed times
        float K = P_pred / (P_pred + R);                        // Gain: K  = P * H^T * (HPH + R)^-1
        float y = point->z;                                     // measurement: y
        float x_final = x_pred + K * (y - x_pred);              // x_final = x_pred + K * (y - H * x_pred)
        float P_final = (1 - K) * P_pred;                       // P_final = (I - K * H) * P_pred
        // Update cell
        thisCell->updateElevation(x_final, P_final);
    }

    mapCell_t *grid2Cell(grid_t *thisGrid)
    {
        return mapArray[mapArrayInd[thisGrid->cubeX][thisGrid->cubeY]]->cellArray[thisGrid->gridX][thisGrid->gridY];
    }

    bool findPointGridInMap(grid_t *gridOut, PointType *point)
    {
        // Calculate the cube index that this point belongs to. (Array dimension: mapArrayLength * mapArrayLength)
        grid_t thisGrid;
        getPointCubeIndex(&thisGrid.cubeX, &thisGrid.cubeY, point);
        // Decide whether a point is out of pre-allocated map
        if (thisGrid.cubeX >= 0 && thisGrid.cubeX < mapArrayLength &&
            thisGrid.cubeY >= 0 && thisGrid.cubeY < mapArrayLength)
        {
            // Point is in the boundary, but this sub-map is not allocated before
            // Allocate new memory for this sub-map and save it to mapArray
            if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1)
            {

                mapArray.push_back(
                    std::make_unique<childMap_t>(mapArrayCount, thisGrid.cubeX, thisGrid.cubeY));
                mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] = mapArrayCount;
                ++mapArrayCount;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                "Point cloud is out of elevation map boundary. Increase mapArrayLength!");
            return false;
        }
        // sub-map id
        thisGrid.mapID = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];

        // Find the index for this point in this sub-map (grid index)
        float originX = mapArray[thisGrid.mapID]->originX;
        float originY = mapArray[thisGrid.mapID]->originY;
        thisGrid.gridX = int((point->x - originX) / mapResolution);
        thisGrid.gridY = int((point->y - originY) / mapResolution);

        if (thisGrid.gridX < 0 || thisGrid.gridY < 0 || 
            thisGrid.gridX >= mapCubeArrayLength || thisGrid.gridY >= mapCubeArrayLength)
        {
            return false;
        }
            
        *gridOut = thisGrid;
        return true;
    }

    void getPointCubeIndex(int *cubeX, int *cubeY, PointType *point)
    {
        *cubeX = int((point->x + mapCubeLength / 2.0) / mapCubeLength) + rootCubeIndex;
        *cubeY = int((point->y + mapCubeLength / 2.0) / mapCubeLength) + rootCubeIndex;

        if (point->x + mapCubeLength / 2.0 < 0)
            --*cubeX;
        if (point->y + mapCubeLength / 2.0 < 0)
            --*cubeY;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Traversability Calculation ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void TraversabilityThread()
    {
        std::lock_guard<std::mutex> lock(mtx);
        traversabilityMapCalculation();
    }

    void traversabilityMapCalculation()
    {

        // no new scan, return
        if (observingList1.size() == 0)
            return;

        observingList2 = observingList1;
        observingList1.clear();

        int listSize = observingList2.size();

        for (int i = 0; i < listSize; ++i)
        {

            mapCell_t *thisCell = observingList2[i];
            // convert this cell to a point for convenience
            PointType thisPoint;
            thisPoint.x = thisCell->xyz->x;
            thisPoint.y = thisCell->xyz->y;
            thisPoint.z = thisCell->xyz->z;
            // too far, not accurate
            if (pointDistance(thisPoint, robotPoint) >= traversabilityCalculatingDistance)
                continue;
            // Find neighbor cells of this center cell
            vector<float> xyzVector = findNeighborElevations(thisCell);

            if (xyzVector.size() <= 2)
                continue;

            Eigen::MatrixXf matPoints = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xyzVector.data(), xyzVector.size() / 3, 3);

            // min and max elevation
            float minElevation = matPoints.col(2).minCoeff();
            float maxElevation = matPoints.col(2).maxCoeff();
            float maxDifference = maxElevation - minElevation;

            if (maxDifference > filterHeightLimit)
            {
                thisPoint.intensity = 100;
                updateCellOccupancy(thisCell, &thisPoint);
                continue;
            }

            // find slope
            Eigen::MatrixXf centered = matPoints.rowwise() - matPoints.colwise().mean();
            Eigen::MatrixXf cov = (centered.adjoint() * centered);
            cv::eigen2cv(cov, matCov);         // copy data from eigen to cv::Mat
            cv::eigen(matCov, matEig, matVec); // find eigenvalues and eigenvectors for the covariance matrix

            updateCellOccupancy(thisCell, &thisPoint);
        }
    }

    vector<float> findNeighborElevations(mapCell_t *centerCell)
    {

        vector<float> xyzVector;

        grid_t centerGrid = centerCell->grid;
        grid_t thisGrid;

        int footprintRadiusLength = int(robotRadius / mapResolution);

        for (int k = -footprintRadiusLength; k <= footprintRadiusLength; ++k)
        {
            for (int l = -footprintRadiusLength; l <= footprintRadiusLength; ++l)
            {
                // skip grids too far
                if (std::sqrt(float(k * k + l * l)) * mapResolution > robotRadius)
                    continue;
                // the neighbor grid
                thisGrid.cubeX = centerGrid.cubeX;
                thisGrid.cubeY = centerGrid.cubeY;
                thisGrid.gridX = centerGrid.gridX + k;
                thisGrid.gridY = centerGrid.gridY + l;
                // If the checked grid is in another sub-map, update it's indexes
                if (thisGrid.gridX < 0)
                {
                    --thisGrid.cubeX;
                    thisGrid.gridX = thisGrid.gridX + mapCubeArrayLength;
                }
                else if (thisGrid.gridX >= mapCubeArrayLength)
                {
                    ++thisGrid.cubeX;
                    thisGrid.gridX = thisGrid.gridX - mapCubeArrayLength;
                }
                if (thisGrid.gridY < 0)
                {
                    --thisGrid.cubeY;
                    thisGrid.gridY = thisGrid.gridY + mapCubeArrayLength;
                }
                else if (thisGrid.gridY >= mapCubeArrayLength)
                {
                    ++thisGrid.cubeY;
                    thisGrid.gridY = thisGrid.gridY - mapCubeArrayLength;
                }
                // If the sub-map that the checked grid belongs to is empty or not
                int mapInd = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
                if (mapInd == -1)
                    continue;
                // the neighbor cell
                mapCell_t *thisCell = grid2Cell(&thisGrid);
                // save neighbor cell for calculating traversability
                if (thisCell->elevation != -FLT_MAX)
                {
                    xyzVector.push_back(thisCell->xyz->x);
                    xyzVector.push_back(thisCell->xyz->y);
                    xyzVector.push_back(thisCell->xyz->z);
                }
            }
        }

        return xyzVector;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////// Occupancy Map (local) //////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // yanzhijie
    void publishMap()
    {
        // Publish Occupancy Grid Map and Elevation Map
        pubCount++;
        if (pubCount > visualizationFrequency)
        {
            pubCount = 1;
            publishLocalMap();
            publishTraversabilityMap();
        }
    }

    void publishLocalMap()
    {
        RCLCPP_INFO(this->get_logger(),
            "subs—local: %zu, subs—height: %zu",
            pubOccupancyMapLocal->get_subscription_count(),
            pubOccupancyMapLocalHeight->get_subscription_count());

        if (pubOccupancyMapLocal->get_subscription_count() == 0 &&
            pubOccupancyMapLocalHeight->get_subscription_count() == 0)
            return;

        // 1.3 Initialize local occupancy grid map to unknown, height to -FLT_MAX
        std::fill(occupancyMap2DHeight.occupancy.data.begin(), occupancyMap2DHeight.occupancy.data.end(), -1);
        std::fill(occupancyMap2DHeight.height.begin(), occupancyMap2DHeight.height.end(), -FLT_MAX);
        std::fill(occupancyMap2DHeight.cost_map.begin(), occupancyMap2DHeight.cost_map.end(), 0);

        // local map origin x and y
        localMapOriginPoint.x = robotPoint.x - localMapLength / 2;
        localMapOriginPoint.y = robotPoint.y - localMapLength / 2;
        localMapOriginPoint.z = robotPoint.z;
        // local map origin cube id (in global map)
        localMapOriginGrid.cubeX = int((localMapOriginPoint.x + mapCubeLength / 2.0) / mapCubeLength) + rootCubeIndex;
        localMapOriginGrid.cubeY = int((localMapOriginPoint.y + mapCubeLength / 2.0) / mapCubeLength) + rootCubeIndex;
        if (localMapOriginPoint.x + mapCubeLength / 2.0 < 0)
            --localMapOriginGrid.cubeX;
        if (localMapOriginPoint.y + mapCubeLength / 2.0 < 0)
            --localMapOriginGrid.cubeY;
        // local map origin grid id (in sub-map)
        float originCubeOriginX, originCubeOriginY; // the orign of submap that the local map origin belongs to (note the submap may not be created yet, cannot use originX and originY)
        originCubeOriginX = (localMapOriginGrid.cubeX - rootCubeIndex) * mapCubeLength - mapCubeLength / 2.0;
        originCubeOriginY = (localMapOriginGrid.cubeY - rootCubeIndex) * mapCubeLength - mapCubeLength / 2.0;
        localMapOriginGrid.gridX = int((localMapOriginPoint.x - originCubeOriginX) / mapResolution);
        localMapOriginGrid.gridY = int((localMapOriginPoint.y - originCubeOriginY) / mapResolution);

        // 2 Calculate local occupancy grid map root position
        // occupancyMap2DHeight.header.stamp = ros::Time::now();
        occupancyMap2DHeight.header.stamp = this->now();
        occupancyMap2DHeight.occupancy.header.stamp = occupancyMap2DHeight.header.stamp;
        occupancyMap2DHeight.occupancy.info.origin.position.x = localMapOriginPoint.x;
        occupancyMap2DHeight.occupancy.info.origin.position.y = localMapOriginPoint.y;
        occupancyMap2DHeight.occupancy.info.origin.position.z = localMapOriginPoint.z + 10.0; // add 10, just for visualization

        // extract all info
        int known_count = 0;
        for (int i = 0; i < localMapArrayLength; ++i)
        {
            for (int j = 0; j < localMapArrayLength; ++j)
            {

                int indX = localMapOriginGrid.gridX + i;
                int indY = localMapOriginGrid.gridY + j;

                grid_t thisGrid;

                thisGrid.cubeX = localMapOriginGrid.cubeX + indX / mapCubeArrayLength;
                thisGrid.cubeY = localMapOriginGrid.cubeY + indY / mapCubeArrayLength;

                thisGrid.gridX = indX % mapCubeArrayLength;
                thisGrid.gridY = indY % mapCubeArrayLength;

                // if sub-map is not created yet
                if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1)
                {
                    continue;
                }

                mapCell_t *thisCell = grid2Cell(&thisGrid);

                // skip unknown grid
                if (thisCell->elevation != -FLT_MAX)
                {
                    int index = i + j * localMapArrayLength; // index of the 1-D array
                    occupancyMap2DHeight.height[index] = thisCell->elevation;
                    occupancyMap2DHeight.occupancy.data[index] = thisCell->occupancy > 80 ? 100 : 0;
                    known_count++;
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "[DEBUG] 发布地图时已知栅格数量: %d", known_count);

        occupancyMap2DHeight.header.frame_id = "map";
        occupancyMap2DHeight.occupancy.header.frame_id = "map";

        pubOccupancyMapLocalHeight->publish(occupancyMap2DHeight);
        pubOccupancyMapLocal->publish(occupancyMap2DHeight.occupancy);
    }

    void initializeLocalOccupancyMap()
    {
        // initialization of customized map message
        occupancyMap2DHeight.header.frame_id = "map";
        occupancyMap2DHeight.occupancy.info.width = localMapArrayLength;
        occupancyMap2DHeight.occupancy.info.height = localMapArrayLength;
        occupancyMap2DHeight.occupancy.info.resolution = mapResolution;

        occupancyMap2DHeight.occupancy.info.origin.orientation.x = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.y = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.z = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.w = 1.0;

        occupancyMap2DHeight.occupancy.data.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        occupancyMap2DHeight.height.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        occupancyMap2DHeight.cost_map.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
    }

    bool getRobotPosition()
    {
        try
        {
            transform = tf_buffer.lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            // listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        }
        // catch (tf::TransformException ex)
        catch (const tf2::TransformException &ex)
        {
            // ROS_ERROR("Transfrom Failure.");
            // return false;
            RCLCPP_ERROR(this->get_logger(), "Transform Failure: %s", ex.what());
            return false;
        }

        robotPoint.x = transform.transform.translation.x;
        robotPoint.y = transform.transform.translation.y;
        robotPoint.z = transform.transform.translation.z;
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Point Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void publishTraversabilityMap()
    {

        if (pubElevationCloud->get_subscription_count() == 0)
            return;
        // 1. Find robot current cube index
        int currentCubeX, currentCubeY;
        getPointCubeIndex(&currentCubeX, &currentCubeY, &robotPoint);
        // 2. Loop through all the sub-maps that are nearby
        // int visualLength = int(visualizationRadius / mapCubeLength);
        int visualLength = static_cast<int>(visualizationRadius / mapCubeLength);

        for (int i = -visualLength; i <= visualLength; ++i)
        {
            for (int j = -visualLength; j <= visualLength; ++j)
            {
                // if (sqrt(float(i * i + j * j)) >= visualLength)
                if (std::sqrt(static_cast<float>(i * i + j * j)) >= visualLength)
                {
                    continue;
                }

                int idx = i + currentCubeX;
                int idy = j + currentCubeY;

                if (idx < 0 || idx >= mapArrayLength ||
                     idy < 0 || idy >= mapArrayLength)
                {
                    continue;
                }

                if (mapArrayInd[idx][idy] == -1)
                {
                    continue;
                }

                *laserCloudElevation += mapArray[mapArrayInd[idx][idy]]->cloud;
            }
        }
        // 3. Publish elevation point cloud
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudElevation, laserCloudTemp);
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = this->now();
        pubElevationCloud->publish(laserCloudTemp);
        // 4. free memory
        laserCloudElevation->clear();
    }
};

int main(int argc, char **argv)
{

    // ros::init(argc, argv, "traversability_mapping");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TraversabilityMapping>();

    // ===================== [AI修改6] 只保留Node内部线程，不再main里开线程 =====================
    // std::thread prediction_thread(
    //     [&node]() {
    //       node->TraversabilityThread();
    //     }
    // );

    RCLCPP_INFO(
        node->get_logger(),
        "\033[1;32m----> Traversability Mapping Started.\033[0m");

    RCLCPP_INFO(
        node->get_logger(),
        "\033[1;32m----> Traversability Mapping Scenario: %s.\033[0m",
        urbanMapping == true ? "Urban" : "Terrain");

    rclcpp::spin(node);

    // if (prediction_thread.joinable()) {
    //     prediction_thread.join();
    // }
    rclcpp::shutdown();
    
    return 0;
}