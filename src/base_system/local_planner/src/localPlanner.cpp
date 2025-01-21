#include <cmath>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

constexpr double PI = 3.1415926;

#define PLOTPATHSET 1

string pathFolder;
double vehicleLength = 0.6;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
bool useCost = false;
constexpr int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
double goalClearRange = 0.5;
double goalX = 0;
double goalY = 0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

constexpr int pathNum = 343;
constexpr int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
constexpr int gridVoxelNumX = 161;
constexpr int gridVoxelNumY = 451;
constexpr int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0}; //= 36 * pathNum = 12348 ==> unsafePointCount
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;


//odom_topic
string odom_topic;
//laser_topic
string lidar_topic;

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom) {
    odomTime = odom->header.stamp.toSec();

    double roll, pitch, yaw;
    const geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll = static_cast<float>(roll);
    vehiclePitch = static_cast<float>(pitch);
    vehicleYaw = static_cast<float>(yaw);
    vehicleX = static_cast<float>(odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY);
    vehicleY = static_cast<float>(odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY);
    vehicleZ = static_cast<float>(odom->pose.pose.position.z);
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {
    if (!useTerrainAnalysis) {
        laserCloud->clear();
        pcl::fromROSMsg(*laserCloud2, *laserCloud);

        pcl::PointXYZI point;
        laserCloudCrop->clear();
        const int laserCloudSize = static_cast<int>(laserCloud->points.size());
        for (int i = 0; i < laserCloudSize; i++) {
            point = laserCloud->points[i];

            const float pointX = point.x;
            const float pointY = point.y;
            const float pointZ = point.z;

            const float dis = sqrt(
                (pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
            if (dis < adjacentRange) {
                point.x = pointX;
                point.y = pointY;
                point.z = pointZ;
                laserCloudCrop->push_back(point);
            }
        }

        laserCloudDwz->clear();
        laserDwzFilter.setInputCloud(laserCloudCrop);
        laserDwzFilter.filter(*laserCloudDwz);

        newLaserCloud = true;
    }
}

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr &terrainCloud2) {
    if (useTerrainAnalysis) {
        terrainCloud->clear();
        pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

        pcl::PointXYZI point;
        terrainCloudCrop->clear();
        const int terrainCloudSize = static_cast<int>(terrainCloud->points.size());
        for (int i = 0; i < terrainCloudSize; i++) {
            point = terrainCloud->points[i];

            const float pointX = point.x;
            const float pointY = point.y;
            const float pointZ = point.z;

            const float dis = sqrt(
                (pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
            if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
                point.x = pointX;
                point.y = pointY;
                point.z = pointZ;
                terrainCloudCrop->push_back(point);
            }
        }

        terrainCloudDwz->clear();
        terrainDwzFilter.setInputCloud(terrainCloudCrop);
        terrainDwzFilter.filter(*terrainCloudDwz);

        newTerrainCloud = true;
    }
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
    joyTime = ros::Time::now().toSec();

    joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    joySpeed = joySpeedRaw;
    if (joySpeed > 1.0) joySpeed = 1.0;
    if (joy->axes[4] == 0) joySpeed = 0;

    if (joySpeed > 0) {
        joyDir = static_cast<float>(atan2(joy->axes[3], joy->axes[4]) * 180 / PI);
        if (joy->axes[4] < 0) joyDir *= -1;
    }

    if (joy->axes[4] < 0 && !twoWayDrive) joySpeed = 0;

    if (joy->axes[2] > -0.1) {
        autonomyMode = false;
    } else {
        autonomyMode = true;
    }

    if (joy->axes[5] > -0.1) {
        checkObstacle = true;
    } else {
        checkObstacle = false;
    }
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr &goal) {
    goalX = goal->point.x;
    goalY = goal->point.y;
}

void speedHandler(const std_msgs::Float32::ConstPtr &speed) {
    const double speedTime = ros::Time::now().toSec();

    if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
        joySpeed = static_cast<float>(speed->data / maxSpeed);

        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0) joySpeed = 1.0;
    }
}

void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr &boundary) {
    boundaryCloud->clear();
    pcl::PointXYZI point, point1, point2;
    const int boundarySize = static_cast<int>(boundary->polygon.points.size());

    if (boundarySize >= 1) {
        point2.x = boundary->polygon.points[0].x;
        point2.y = boundary->polygon.points[0].y;
        point2.z = boundary->polygon.points[0].z;
    }

    for (int i = 0; i < boundarySize; i++) {
        point1 = point2;

        point2.x = boundary->polygon.points[i].x;
        point2.y = boundary->polygon.points[i].y;
        point2.z = boundary->polygon.points[i].z;

        if (point1.z == point2.z) {
            const float disX = point1.x - point2.x;
            const float disY = point1.y - point2.y;
            const float dis = sqrt(disX * disX + disY * disY);

            const int pointNum = static_cast<int>(dis / terrainVoxelSize) + 1;
            for (int pointID = 0; pointID < pointNum; pointID++) {
                point.x = static_cast<float>(static_cast<float>(pointID) / static_cast<float>(pointNum) * point1.x + (
                              1.0 - static_cast<float>(pointID) / static_cast<float>(pointNum)) * point2.x);
                point.y = static_cast<float>(static_cast<float>(pointID) / static_cast<float>(pointNum) * point1.y + (
                              1.0 - static_cast<float>(pointID) / static_cast<float>(pointNum)) * point2.y);
                point.z = 0;
                point.intensity = 100.0;

                for (int j = 0; j < pointPerPathThre; j++) {
                    boundaryCloud->push_back(point);
                }
            }
        }
    }
}

void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr &addedObstacles2) {
    addedObstacles->clear();
    pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

    const int addedObstaclesSize = static_cast<int>(addedObstacles->points.size());
    for (int i = 0; i < addedObstaclesSize; i++) {
        addedObstacles->points[i].intensity = 200.0;
    }
}

void checkObstacleHandler(const std_msgs::Bool::ConstPtr &checkObs) {
    const double checkObsTime = ros::Time::now().toSec();

    if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
        checkObstacle = checkObs->data;
    }
}

int readPlyHeader(FILE *filePtr) {
    char str[50];
    int val, pointNum;
    string strCur, strLast;
    while (strCur != "end_header") {
        val = fscanf(filePtr, "%s", str);
        if (val != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        strLast = strCur;
        strCur = string(str);

        if (strCur == "vertex" && strLast == "element") {
            val = fscanf(filePtr, "%d", &pointNum);
            if (val != 1) {
                printf("\nError reading input files, exit.\n\n");
                exit(1);
            }
        }
    }

    return pointNum;
}

void readStartPaths() {
    const string fileName = pathFolder + "/startPaths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == nullptr) {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    const int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZ point;
    int val1, val2, val3, val4, groupID;
    for (int i = 0; i < pointNum; i++) {
        val1 = fscanf(filePtr, "%f", &point.x);
        val2 = fscanf(filePtr, "%f", &point.y);
        val3 = fscanf(filePtr, "%f", &point.z);
        val4 = fscanf(filePtr, "%d", &groupID);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        if (groupID >= 0 && groupID < groupNum) {
            startPaths[groupID]->push_back(point);
        }
    }

    fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths() {
    const string fileName = pathFolder + "/paths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == nullptr) {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    const int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZI point;
    int pointSkipNum = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID;
    for (int i = 0; i < pointNum; i++) {
        val1 = fscanf(filePtr, "%f", &point.x);
        val2 = fscanf(filePtr, "%f", &point.y);
        val3 = fscanf(filePtr, "%f", &point.z);
        val4 = fscanf(filePtr, "%d", &pathID);
        val5 = fscanf(filePtr, "%f", &point.intensity);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        if (pathID >= 0 && pathID < pathNum) {
            pointSkipCount++;
            if (pointSkipCount > pointSkipNum) {
                paths[pathID]->push_back(point);
                pointSkipCount = 0;
            }
        }
    }

    fclose(filePtr);
}
#endif

void readPathList() {
    const string fileName = pathFolder + "/pathList.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == nullptr) {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    if (pathNum != readPlyHeader(filePtr)) {
        printf("\nIncorrect path number, exit.\n\n");
        exit(1);
    }

    int val1, val2, val3, val4, val5, pathID, groupID;
    float endX, endY, endZ;
    for (int i = 0; i < pathNum; i++) {
        val1 = fscanf(filePtr, "%f", &endX);
        val2 = fscanf(filePtr, "%f", &endY);
        val3 = fscanf(filePtr, "%f", &endZ);
        val4 = fscanf(filePtr, "%d", &pathID);
        val5 = fscanf(filePtr, "%d", &groupID);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
            pathList[pathID] = groupID;
            endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
        }
    }

    fclose(filePtr);
}

void readCorrespondences() {
    const string fileName = pathFolder + "/correspondences.txt";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == nullptr) {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    int val1, gridVoxelID, pathID;
    for (int i = 0; i < gridVoxelNum; i++) {
        val1 = fscanf(filePtr, "%d", &gridVoxelID);
        if (val1 != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        while (true) {
            val1 = fscanf(filePtr, "%d", &pathID);
            if (val1 != 1) {
                printf("\nError reading input files, exit.\n\n");
                exit(1);
            }

            if (pathID != -1) {
                if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
                    correspondences[gridVoxelID].push_back(pathID);
                }
            } else {
                break;
            }
        }
    }

    fclose(filePtr);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localPlanner");
    ros::NodeHandle nh;
    auto nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("pathFolder", pathFolder);
    nhPrivate.getParam("vehicleLength", vehicleLength);
    nhPrivate.getParam("vehicleWidth", vehicleWidth);
    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
    nhPrivate.getParam("twoWayDrive", twoWayDrive);
    nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
    nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
    nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
    nhPrivate.getParam("checkObstacle", checkObstacle);
    nhPrivate.getParam("checkRotObstacle", checkRotObstacle);
    nhPrivate.getParam("adjacentRange", adjacentRange);
    nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
    nhPrivate.getParam("groundHeightThre", groundHeightThre);
    nhPrivate.getParam("costHeightThre", costHeightThre);
    nhPrivate.getParam("costScore", costScore);
    nhPrivate.getParam("useCost", useCost);
    nhPrivate.getParam("pointPerPathThre", pointPerPathThre);
    nhPrivate.getParam("minRelZ", minRelZ);
    nhPrivate.getParam("maxRelZ", maxRelZ);
    nhPrivate.getParam("maxSpeed", maxSpeed);
    nhPrivate.getParam("dirWeight", dirWeight);
    nhPrivate.getParam("dirThre", dirThre);
    nhPrivate.getParam("dirToVehicle", dirToVehicle);
    nhPrivate.getParam("pathScale", pathScale);
    nhPrivate.getParam("minPathScale", minPathScale);
    nhPrivate.getParam("pathScaleStep", pathScaleStep);
    nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed);
    nhPrivate.getParam("minPathRange", minPathRange);
    nhPrivate.getParam("pathRangeStep", pathRangeStep);
    nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed);
    nhPrivate.getParam("pathCropByGoal", pathCropByGoal);
    nhPrivate.getParam("autonomyMode", autonomyMode);
    nhPrivate.getParam("autonomySpeed", autonomySpeed);
    nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
    nhPrivate.getParam("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
    nhPrivate.getParam("goalClearRange", goalClearRange);
    nhPrivate.getParam("goalX", goalX);
    nhPrivate.getParam("goalY", goalY);

    //getPram odom topic with default value "/state_estimation"
    nhPrivate.param("odom_topic", odom_topic, std::string("/state_estimation"));
    nhPrivate.param("lidar_topic", lidar_topic, std::string("/registered_scan"));

    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
            (odom_topic, 5, odometryHandler);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
            (lidar_topic, 5, laserCloudHandler);

    ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>
            ("/terrain_map", 5, terrainCloudHandler);

    ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

    ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped>("/way_point", 5, goalHandler);

    ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

    ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped>(
        "/navigation_boundary", 5, boundaryHandler);

    ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2>(
        "/added_obstacles", 5, addedObstaclesHandler);

    ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool>("/check_obstacle", 5, checkObstacleHandler);

    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path_control", 5);
    nav_msgs::Path path;

#if PLOTPATHSET == 1
    ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2>("/free_paths", 2);
#endif

    ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/stacked_scans", 2);

    printf("\nReading path files.\n");

    if (autonomyMode) {
        joySpeed = static_cast<float>(autonomySpeed / maxSpeed);

        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0) joySpeed = 1.0;
    }

    for (auto &i: laserCloudStack) {
        i.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    for (auto &startPath: startPaths) {
        startPath.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
#if PLOTPATHSET == 1
    for (auto &pi: paths) {
        pi.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
#endif
    for (auto &correspondence: correspondences) {
        correspondence.resize(0);
    }

    laserDwzFilter.setLeafSize(static_cast<float>(laserVoxelSize), static_cast<float>(laserVoxelSize), static_cast<float>(laserVoxelSize));
    terrainDwzFilter.setLeafSize(static_cast<float>(terrainVoxelSize), static_cast<float>(terrainVoxelSize), static_cast<float>(terrainVoxelSize));

    readStartPaths();
#if PLOTPATHSET == 1
    readPaths();
#endif
    readPathList();
    readCorrespondences();

    printf("\nInitialization complete.\n\n");

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        if (newLaserCloud || newTerrainCloud) {
            if (newLaserCloud) {
                newLaserCloud = false;

                laserCloudStack[laserCloudCount]->clear();
                *laserCloudStack[laserCloudCount] = *laserCloudDwz;
                laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

                plannerCloud->clear();
                for (const auto &i: laserCloudStack) {
                    *plannerCloud += *i;
                }
            }

            if (newTerrainCloud) {
                newTerrainCloud = false;

                plannerCloud->clear();
                *plannerCloud = *terrainCloudDwz;
            }

            float sinVehicleRoll = sin(vehicleRoll);
            float cosVehicleRoll = cos(vehicleRoll);
            float sinVehiclePitch = sin(vehiclePitch);
            float cosVehiclePitch = cos(vehiclePitch);
            float sinVehicleYaw = sin(vehicleYaw);
            float cosVehicleYaw = cos(vehicleYaw);

            pcl::PointXYZI point;
            plannerCloudCrop->clear();
            int plannerCloudSize = static_cast<int>(plannerCloud->points.size());
            for (int i = 0; i < plannerCloudSize; i++) {
                float pointX1 = plannerCloud->points[i].x - vehicleX;
                float pointY1 = plannerCloud->points[i].y - vehicleY;
                float pointZ1 = plannerCloud->points[i].z - vehicleZ;

                point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw; //convert to vehicle frame
                point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                point.z = pointZ1;
                point.intensity = plannerCloud->points[i].intensity;

                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
                    plannerCloudCrop->push_back(point);
                }
            }

            int boundaryCloudSize = static_cast<int>(boundaryCloud->points.size());
            for (int i = 0; i < boundaryCloudSize; i++) {
                point.x = (boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw
                          + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw;
                point.y = -(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw
                          + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw;
                point.z = boundaryCloud->points[i].z;
                point.intensity = boundaryCloud->points[i].intensity;

                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange) {
                    plannerCloudCrop->push_back(point);
                }
            }

            int addedObstaclesSize = static_cast<int>(addedObstacles->points.size());
            for (int i = 0; i < addedObstaclesSize; i++) {
                point.x = (addedObstacles->points[i].x - vehicleX) * cosVehicleYaw
                          + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw;
                point.y = -(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw
                          + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw;
                point.z = addedObstacles->points[i].z;
                point.intensity = addedObstacles->points[i].intensity;

                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange) {
                    plannerCloudCrop->push_back(point);
                }
            }

            auto pathRange = static_cast<float>(adjacentRange);
            if (pathRangeBySpeed) pathRange = static_cast<float>(adjacentRange * joySpeed);
            if (pathRange < minPathRange) pathRange = static_cast<float>(minPathRange);
            auto relativeGoalDis = static_cast<float>(adjacentRange);

            if (autonomyMode) {
                auto relativeGoalX = static_cast<float>((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
                auto relativeGoalY = static_cast<float>(-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

                relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
                joyDir = static_cast<float>(atan2(relativeGoalY, relativeGoalX) * 180.0 / PI);

                if (!twoWayDrive) {
                    if (joyDir > 90.0) joyDir = 90.0;
                    else if (joyDir < -90.0) joyDir = -90.0;
                }
            }

            bool pathFound = false;
            auto defPathScale = static_cast<float>(pathScale);
            if (pathScaleBySpeed) pathScale = defPathScale * joySpeed; //1.25
            if (pathScale < minPathScale) pathScale = minPathScale;

            while (pathScale >= minPathScale && pathRange >= minPathRange) {
                // pathscale >= 0.75 and pathRange >= 1.0
                for (int i = 0; i < 36 * pathNum; i++) {
                    // 36 * pathNum = 12348
                    clearPathList[i] = 0;
                    pathPenaltyList[i] = 0;
                }
                for (float &i: clearPathPerGroupScore) {
                    i = 0;
                }

                float minObsAngCW = -180.0;
                float minObsAngCCW = 180.0;
                auto diameter = static_cast<float>(sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0)); //
                auto angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI; // 26.565
                int plannerCloudCropSize = static_cast<int>(plannerCloudCrop->points.size());
                for (int i = 0; i < plannerCloudCropSize; i++) {
                    auto x = static_cast<float>(plannerCloudCrop->points[i].x / pathScale);
                    auto y = static_cast<float>(plannerCloudCrop->points[i].y / pathScale);
                    float h = plannerCloudCrop->points[i].intensity;
                    float dis = sqrt(x * x + y * y);

                    if (dis < pathRange / pathScale && (
                            dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) &&
                        checkObstacle) {
                        for (int rotDir = 0; rotDir < 36; rotDir++) {
                            auto rotAng = static_cast<float>((10.0 * rotDir - 180.0) * PI / 180.0);
                            auto angDiff = static_cast<float>(fabs(joyDir - (10.0 * rotDir - 180.0)));
                            if (angDiff > 180.0) {
                                angDiff = static_cast<float>(360.0 - angDiff);
                            }
                            if ((angDiff > dirThre && !dirToVehicle) || (
                                    fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 &&
                                 dirToVehicle)) {
                                continue;
                            }

                            float x2 = cos(rotAng) * x + sin(rotAng) * y;
                            float y2 = -sin(rotAng) * x + cos(rotAng) * y;

                            float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY
                                           * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

                            int indX = static_cast<int>((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
                            int indY = static_cast<int>(
                                (gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
                            if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                                int ind = gridVoxelNumY * indX + indY;
                                int blockedPathByVoxelNum = static_cast<int>(correspondences[ind].size());
                                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                                    if (h > obstacleHeightThre || !useTerrainAnalysis) {
                                        clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                                    } else {
                                        if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h >
                                            groundHeightThre) {
                                            pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (dis < diameter / pathScale && (
                            fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) &&
                        (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
                        auto angObs = static_cast<float>(atan2(y, x) * 180.0 / PI);
                        if (angObs > 0) {
                            if (minObsAngCCW > angObs - angOffset) minObsAngCCW = static_cast<float>(angObs - angOffset);
                            if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = static_cast<float>(angObs + angOffset - 180.0);
                        } else {
                            if (minObsAngCW < angObs + angOffset) minObsAngCW = static_cast<float>(angObs + angOffset);
                            if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = static_cast<float>(180.0 + angObs - angOffset);
                        }
                    }
                }

                if (minObsAngCW > 0) minObsAngCW = 0;
                if (minObsAngCCW < 0) minObsAngCCW = 0;

                for (int i = 0; i < 36 * pathNum; i++) {
                    int rotDir = i / pathNum;
                    auto angDiff = static_cast<float>(fabs(joyDir - (10.0 * rotDir - 180.0)));
                    if (angDiff > 180.0) {
                        angDiff = static_cast<float>(360.0 - angDiff);
                    }
                    if ((angDiff > dirThre && !dirToVehicle) || (
                            fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                        ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 &&
                         dirToVehicle)) {
                        continue;
                    }

                    if (clearPathList[i] < pointPerPathThre) {
                        auto penaltyScore = static_cast<float>(1.0 - pathPenaltyList[i] / costHeightThre);
                        if (penaltyScore < costScore) penaltyScore = static_cast<float>(costScore);

                        auto dirDiff = static_cast<float>(fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0)));
                        if (dirDiff > 360.0) {
                            dirDiff -= 360.0;
                        }
                        if (dirDiff > 180.0) {
                            dirDiff = static_cast<float>(360.0 - dirDiff);
                        }

                        float rotDirW;
                        if (rotDir < 18) rotDirW = static_cast<float>(fabs(fabs(rotDir - 9) + 1));
                        else rotDirW = static_cast<float>(fabs(fabs(rotDir - 27) + 1));

                        //float sigma = 6.0; //
                        //float rotDirW = exp(-pow((rotDir - 18) / sigma, 2));

                        auto score = static_cast<float>((1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore);
                        if (score > 0) {
                            clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
                        }
                    }
                }

                float maxScore = 0;
                int selectedGroupID = -1;
                for (int i = 0; i < 36 * groupNum; i++) {
                    int rotDir = i / groupNum;
                    auto rotAng = static_cast<float>((10.0 * rotDir - 180.0) * PI / 180.0);
                    auto rotDeg = static_cast<float>(10.0 * rotDir);
                    if (rotDeg > 180.0) rotDeg -= 360.0;
                    if (maxScore < clearPathPerGroupScore[i] && (
                            (rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) ||
                            (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
                        maxScore = clearPathPerGroupScore[i];
                        selectedGroupID = i;
                    }
                }

                if (selectedGroupID >= 0) {
                    int rotDir = selectedGroupID / groupNum;
                    auto rotAng = static_cast<float>((10.0 * rotDir - 180.0) * PI / 180.0);

                    selectedGroupID = selectedGroupID % groupNum;
                    int selectedPathLength = static_cast<int>(startPaths[selectedGroupID]->points.size());
                    path.poses.resize(selectedPathLength);
                    for (int i = 0; i < selectedPathLength; i++) {
                        float x = startPaths[selectedGroupID]->points[i].x;
                        float y = startPaths[selectedGroupID]->points[i].y;
                        float z = startPaths[selectedGroupID]->points[i].z;
                        float dis = sqrt(x * x + y * y);

                        if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
                            path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                            path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                            path.poses[i].pose.position.z = pathScale * z;
                        } else {
                            path.poses.resize(i);
                            break;
                        }
                    }

                    path.header.stamp = ros::Time().fromSec(odomTime);
                    path.header.frame_id = "vehicle";
                    pubPath.publish(path);

#if PLOTPATHSET == 1
                    freePaths->clear();
                    for (int i = 0; i < 36 * pathNum; i++) {
                        int rotDir = i / pathNum;
                        auto rotAng = static_cast<float>((10.0 * rotDir - 180.0) * PI / 180);
                        auto rotDeg = static_cast<float>(10.0 * rotDir);
                        if (rotDeg > 180.0) rotDeg -= 360.0;
                        auto angDiff = static_cast<float>(fabs(joyDir - (10.0 * rotDir - 180.0)));
                        if (angDiff > 180.0) {
                            angDiff = static_cast<float>(360.0 - angDiff);
                        }
                        if ((angDiff > dirThre && !dirToVehicle) || (
                                fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                            ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 &&
                             dirToVehicle) ||
                            !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) ||
                              (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
                            continue;
                        }

                        if (clearPathList[i] < pointPerPathThre) {
                            int freePathLength = static_cast<int>(paths[i % pathNum]->points.size());
                            for (int j = 0; j < freePathLength; j++) {
                                point = paths[i % pathNum]->points[j];

                                float x = point.x;
                                float y = point.y;
                                float z = point.z;

                                float dis = sqrt(x * x + y * y);
                                if (dis <= pathRange / pathScale && (
                                        dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {
                                    point.x = static_cast<float>(pathScale * (cos(rotAng) * x - sin(rotAng) * y));
                                    point.y = static_cast<float>(pathScale * (sin(rotAng) * x + cos(rotAng) * y));
                                    point.z = static_cast<float>(pathScale * z);
                                    point.intensity = 1.0;

                                    freePaths->push_back(point);
                                }
                            }
                        }
                    }

                    sensor_msgs::PointCloud2 freePaths2;
                    pcl::toROSMsg(*freePaths, freePaths2);
                    freePaths2.header.stamp = ros::Time().fromSec(odomTime);
                    freePaths2.header.frame_id = "vehicle";
                    pubFreePaths.publish(freePaths2);
#endif
                }

                if (selectedGroupID < 0) {
                    if (pathScale >= minPathScale + pathScaleStep) {
                        pathScale -= pathScaleStep;
                        pathRange = static_cast<float>(adjacentRange * pathScale / defPathScale);
                    } else {
                        pathRange -= static_cast<float>(pathRangeStep);
                    }
                } else {
                    pathFound = true;
                    break;
                }
            }
            pathScale = defPathScale;

            if (!pathFound) {
                path.poses.resize(1);
                path.poses[0].pose.position.x = 0;
                path.poses[0].pose.position.y = 0;
                path.poses[0].pose.position.z = 0;

                path.header.stamp = ros::Time().fromSec(odomTime);
                path.header.frame_id = "vehicle";
                pubPath.publish(path);

#if PLOTPATHSET == 1
                freePaths->clear();
                sensor_msgs::PointCloud2 freePaths2;
                pcl::toROSMsg(*freePaths, freePaths2);
                freePaths2.header.stamp = ros::Time().fromSec(odomTime);
                freePaths2.header.frame_id = "vehicle";
                pubFreePaths.publish(freePaths2);
#endif
            }

            sensor_msgs::PointCloud2 plannerCloud2;
            pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
            plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
            plannerCloud2.header.frame_id = "vehicle";
            pubLaserCloud.publish(plannerCloud2);
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
