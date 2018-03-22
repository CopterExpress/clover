#include <algorithm>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <stdio.h>
#include <tf/transform_broadcaster.h>

#include "util.h"

using std::vector;
using std::string;

namespace aruco_pose {

class ArucoPose : public nodelet::Nodelet {
    tf::TransformBroadcaster br;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Board> board;
    std::string frame_id_;
    image_transport::CameraSubscriber img_sub;
    image_transport::Publisher img_pub;
    ros::Publisher marker_pub;
    ros::Publisher pose_pub;
    ros::NodeHandle nh_, nh_priv_;

    virtual void onInit();
    void createBoard();
    cv::Point3f getObjPointsCenter(cv::Mat objPoints);
    void detect(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr&, cv::Mat&, cv::Mat&);
    tf::Transform aruco2tf(cv::Mat rvec, cv::Mat tvec);
};

void ArucoPose::onInit() {
    ROS_INFO("Initializing aruco_pose");
    nh_ = getNodeHandle();
    nh_priv_ = getPrivateNodeHandle();

    nh_priv_.param("frame_id", frame_id_, std::string("aruco_map"));

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    parameters = cv::aruco::DetectorParameters::create();

    try
    {
        createBoard();
    }
    catch (const std::exception &exc)
    {
        std::cerr << exc.what();
        exit(0);
    }

    image_transport::ImageTransport it(nh_);
    img_sub = it.subscribeCamera("image", 1, &ArucoPose::detect, this);

    image_transport::ImageTransport it_priv(nh_priv_);
    img_pub = it_priv.advertise("debug", 1);

    pose_pub = nh_priv_.advertise<geometry_msgs::PoseStamped>("pose", 1);

    ROS_INFO("aruco_pose nodelet inited");
}

cv::Ptr<cv::aruco::Board> createCustomGridBoard(int markersX, int markersY, float markerLength, float markerSeparationX, float markerSeparationY,
                            const cv::Ptr<cv::aruco::Dictionary> &dictionary, std::vector<int> ids) {

    CV_Assert(markersX > 0 && markersY > 0 && markerLength > 0 && markerSeparationX > 0 && markerSeparationY > 0);

    cv::Ptr<cv::aruco::Board> res = cv::makePtr<cv::aruco::Board>();

    res->dictionary = dictionary;

    size_t totalMarkers = (size_t) markersX * markersY;
    res->ids = ids;
    res->objPoints.reserve(totalMarkers);

    // calculate Board objPoints
    float maxY = (float)markersY * markerLength + (markersY - 1) * markerSeparationY;
    for(int y = 0; y < markersY; y++) {
        for(int x = 0; x < markersX; x++) {
            std::vector< cv::Point3f > corners;
            corners.resize(4);
            corners[0] = cv::Point3f(x * (markerLength + markerSeparationX),
                                 maxY - y * (markerLength + markerSeparationY), 0);
            corners[1] = corners[0] + cv::Point3f(markerLength, 0, 0);
            corners[2] = corners[0] + cv::Point3f(markerLength, -markerLength, 0);
            corners[3] = corners[0] + cv::Point3f(0, -markerLength, 0);
            res->objPoints.push_back(corners);
        }
    }

    return res;
}

cv::Ptr<cv::aruco::Board> createCustomBoard(std::map<string, string> markers, const cv::Ptr<cv::aruco::Dictionary> &dictionary) {
    cv::Ptr<cv::aruco::Board> res = cv::makePtr<cv::aruco::Board>();

    res->dictionary = dictionary;

    size_t total_markers = markers.size();
    res->ids.reserve(total_markers);
    res->objPoints.reserve(total_markers);

    // Generate ids and objPoints
    for(auto const &marker : markers) {
        res->ids.push_back(std::stoi(marker.first));

        vector<string> parts;
        parts = strSplit(marker.second, " ");

        float size = std::stof(parts.at(0));
        float x = std::stof(parts.at(1));
        float y = std::stof(parts.at(2));
        float z = std::stof(parts.at(3));
        float yaw = std::stof(parts.at(4));
        float pitch = std::stof(parts.at(5));
        float roll = std::stof(parts.at(6));

        vector<cv::Point3f> corners;
        corners.resize(4);
        corners[0] = cv::Point3f(x - size / 2, y + size / 2, 0);
        corners[1] = corners[0] + cv::Point3f(size, 0, 0);
        corners[2] = corners[0] + cv::Point3f(size, -size, 0);
        corners[3] = corners[0] + cv::Point3f(0, -size, 0);

        // TODO: process yaw, pitch, roll

        res->objPoints.push_back(corners);
    }

    return res;
}

#include "fix.cpp"

void ArucoPose::createBoard()
{
    static auto map_image_pub = nh_priv_.advertise<sensor_msgs::Image>("map_image", 1, true);
    cv_bridge::CvImage map_image_msg;
    cv::Mat map_image;

    std::string type;

    nh_priv_.param<std::string>("type", type, "gridboard");
    if (type == "gridboard")
    {
        ROS_INFO("Initialize gridboard");

        int markers_x, markers_y, first_marker;
        float markers_side, markers_sep_x, markers_sep_y;
        std::vector<int> marker_ids;
        nh_priv_.param<int>("markers_x", markers_x, 10);
        nh_priv_.param<int>("markers_y", markers_y, 10);
        nh_priv_.param<int>("first_marker", first_marker, 0);

        if (!nh_priv_.getParam("markers_side", markers_side))
        {
            ROS_ERROR("gridboard: required parameter ~markers_side is not set.");
            exit(1);
        }

        if (!nh_priv_.getParam("markers_sep_x", markers_sep_x))
        {
            if (!nh_priv_.getParam("markers_sep", markers_sep_x))
            {
               ROS_ERROR("gridboard: ~markers_sep_x or ~markers_sep parameters are required");
               exit(1);
            }
        }

        if (!nh_priv_.getParam("markers_sep_y", markers_sep_y))
        {
            if (!nh_priv_.getParam("markers_sep", markers_sep_y))
            {
               ROS_ERROR("gridboard: ~markers_sep_y or ~markers_sep parameters are required");
               exit(1);
            }
        }

        if (nh_priv_.getParam("marker_ids", marker_ids))
        {
            if (markers_x * markers_y != marker_ids.size())
            {
                ROS_FATAL("~marker_ids length should be equal to ~markers_x * ~markers_y");
                exit(1);
            }
        }
        else
        {
            // Fill marker_ids automatically
            marker_ids.resize(markers_x * markers_y);
            for(int i = 0; i < markers_x * markers_y; i++)
            {
                marker_ids.at(i) = first_marker++;
            }
        }

        // Create grid board
        board = createCustomGridBoard(markers_x, markers_y, markers_side, markers_sep_x, markers_sep_y, dictionary, marker_ids);

        // Publish map image for debugging
        _drawPlanarBoard(board,  cv::Size(2000, 2000), map_image, 50, 1);

        cv::cvtColor(map_image, map_image, CV_GRAY2BGR);

        map_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        map_image_msg.image = map_image;
        map_image_pub.publish(map_image_msg.toImageMsg());
    }
    else if (type == "custom")
    {
        ROS_INFO("Initialize a custom board");

        std::map<string, string> markers;
        nh_priv_.getParam("markers", markers);

        board = createCustomBoard(markers, dictionary);

        ROS_INFO("Draw a custom board");
        // Publish map image for debugging
        _drawPlanarBoard(board,  cv::Size(2000, 2000), map_image, 50, 1);

        cv::cvtColor(map_image, map_image, CV_GRAY2BGR);

        map_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        map_image_msg.image = map_image;
        map_image_pub.publish(map_image_msg.toImageMsg());
    }
    else
    {
        ROS_ERROR("Incorrect map type '%s'", type.c_str());
    }
}

cv::Point3f ArucoPose::getObjPointsCenter(cv::Mat objPoints) {
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float min_y = min_x, max_y = max_x;
    for (int i = 0; i < objPoints.rows; i++) {
        max_x = std::max(max_x, objPoints.at<float>(i, 0));
        max_y = std::max(max_y, objPoints.at<float>(i, 1));
        min_x = std::min(min_x, objPoints.at<float>(i, 0));
        min_y = std::min(min_y, objPoints.at<float>(i, 1));
    }
    cv::Point3f res((min_x + max_x) / 2, (min_y + max_y) / 2, 0);
    return res;
}

void ArucoPose::detect(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &cinfo) {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;

    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    cv::Mat cameraMatrix(3, 3, CV_64F);
    cv::Mat distCoeffs(8, 1, CV_64F);
    parseCameraInfo(cinfo, cameraMatrix, distCoeffs);

    int valid = 0;
    cv::Mat rvec, tvec, objPoints;

    if (markerIds.size() > 0) {

        valid = _estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs,
                                       rvec, tvec, false, objPoints);

        if (valid) {
            // Send map transform
            tf::StampedTransform transform(aruco2tf(rvec, tvec), msg->header.stamp, cinfo->header.frame_id, frame_id_);
            br.sendTransform(transform);

            // Publish map pose
            static geometry_msgs::PoseStamped ps;
            ps.header.frame_id = frame_id_;
            ps.header.stamp = msg->header.stamp;
            ps.pose.orientation.w = 1;
            pose_pub.publish(ps);

            // Send reference point
            cv::Point3f ref = getObjPointsCenter(objPoints);
            tf::Vector3 ref_vector3 = tf::Vector3(ref.x, ref.y, ref.z);
            tf::Quaternion q(0, 0, 0);
            static tf::StampedTransform ref_transform;
            ref_transform.stamp_ = msg->header.stamp;
            ref_transform.frame_id_ = frame_id_;
            ref_transform.child_frame_id_ = "aruco_map_reference";
            ref_transform.setOrigin(ref_vector3);
            ref_transform.setRotation(q);
            br.sendTransform(ref_transform);
        }
    }

    if (img_pub.getNumSubscribers() > 0)
    {
        cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds); // draw markers
        if (valid)
        {
            cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvec, tvec, 0.3); // draw board axis
        }
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = msg->header.frame_id;
        out_msg.header.stamp = msg->header.stamp;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = image;
        img_pub.publish(out_msg.toImageMsg());
    }
}

void ArucoPose::parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo, cv::Mat &cameraMat, cv::Mat &distCoeffs) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cameraMat.at<double>(i, j) = cinfo->K[3 * i + j];
        }
    }
    for (int k = 0; k < cinfo->D.size(); k++) {
        distCoeffs.at<double>(k) = cinfo->D[k];
    }
}

tf::Transform ArucoPose::aruco2tf(cv::Mat rvec, cv::Mat tvec) {

    cv::Mat rot;
    cv::Rodrigues(rvec, rot);

    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));
    tf::Vector3 tf_orig(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
    return tf::Transform(tf_rot, tf_orig);
}

PLUGINLIB_EXPORT_CLASS(ArucoPose, nodelet::Nodelet)

}
