#include <algorithm>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <tf2_ros/transform_broadcaster.h>
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

// #include <aruco_pose/MarkerArray.h>
// #include <aruco_pose/Marker.h>

namespace aruco_pose {

class ArucoPose : public nodelet::Nodelet {
//    tf2_ros::TransformBroadcaster br;
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
    void publishVisualizationMarkers();
    cv::Point3f getObjPointsCenter(cv::Mat objPoints);
    void detect(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr&, cv::Mat&, cv::Mat&);
    tf::Transform aruco2tf(cv::Mat rvec, cv::Mat tvec);

public:
      ArucoPose() {};
      virtual ~ArucoPose() {};
};

void ArucoPose::onInit() {
    ROS_INFO("Initializing aruco_pose");
    nh_ = getNodeHandle();
    nh_priv_ = getPrivateNodeHandle();

    nh_priv_.param("frame_id", frame_id_, std::string("aruco_map"));

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    parameters = cv::aruco::DetectorParameters::create();
    createBoard();

    image_transport::ImageTransport it(nh_);
    img_sub = it.subscribeCamera("image", 1, &ArucoPose::detect, this);

    image_transport::ImageTransport it_priv(nh_priv_);
    img_pub = it_priv.advertise("debug", 1);

    pose_pub = nh_priv_.advertise<geometry_msgs::PoseStamped>("pose", 1);

    publishVisualizationMarkers();

    ROS_INFO("aruco_pose nodelet inited");
}

cv::Ptr<cv::aruco::Board> createCustomBoard(int markersX, int markersY, float markerLength, float markerSeparation,
                            const cv::Ptr<cv::aruco::Dictionary> &dictionary, std::vector<int> ids) {

    CV_Assert(markersX > 0 && markersY > 0 && markerLength > 0 && markerSeparation > 0);

    cv::Ptr<cv::aruco::Board> res = cv::makePtr<cv::aruco::Board>();

    res->dictionary = dictionary;

    size_t totalMarkers = (size_t) markersX * markersY;
    res->ids = ids;
    res->objPoints.reserve(totalMarkers);

    // calculate Board objPoints
    float maxY = (float)markersY * markerLength + (markersY - 1) * markerSeparation;
    for(int y = 0; y < markersY; y++) {
        for(int x = 0; x < markersX; x++) {
            std::vector< cv::Point3f > corners;
            corners.resize(4);
            corners[0] = cv::Point3f(x * (markerLength + markerSeparation),
                                 maxY - y * (markerLength + markerSeparation), 0);
            corners[1] = corners[0] + cv::Point3f(markerLength, 0, 0);
            corners[2] = corners[0] + cv::Point3f(markerLength, -markerLength, 0);
            corners[3] = corners[0] + cv::Point3f(0, -markerLength, 0);
            res->objPoints.push_back(corners);
        }
    }

    return res;
}

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
        float markers_side, markers_sep;
        std::vector<int> marker_ids;
        nh_priv_.param<int>("markers_x", markers_x, 10);
        nh_priv_.param<int>("markers_y", markers_y, 10);
        nh_priv_.param<int>("first_marker", first_marker, 0);

        if (!nh_priv_.getParam("markers_side", markers_side))
            ROS_ERROR("gridboard: required parameter ~markers_side is not set.");

        if (!nh_priv_.getParam("markers_sep", markers_sep))
            ROS_ERROR("gridboard: required parameter ~markers_sep is not set.");

        if (nh_priv_.getParam("marker_ids", marker_ids)) {
            if (markers_x * markers_y != marker_ids.size()) {
                ROS_FATAL("~marker_ids length should be equal to ~markers_x * ~markers_y");
                exit(1);
            }
            board = createCustomBoard(markers_x, markers_y, markers_side, markers_sep, dictionary, marker_ids);
        }
        else {
            board = cv::aruco::GridBoard::create(markers_x, markers_y, markers_side, markers_sep, dictionary, first_marker);
        }

        // Publish map image for debugging
        cv::aruco::drawPlanarBoard(board, cv::Size(2000, 2000), map_image, 50, 1);

        cv::cvtColor(map_image, map_image, CV_GRAY2BGR);

        map_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        map_image_msg.image = map_image;
        map_image_pub.publish(map_image_msg.toImageMsg());
    }
    else if (type == "custom")
    {
        // Not implemented yet
        ROS_FATAL("Custom boards are not implemented yet.")
    }
    else
    {
        ROS_ERROR("Incorrect map type '%s'", type.c_str());
    }
}

void ArucoPose::publishVisualizationMarkers()
{
    // Create latched publisher
    static auto viz_markers_pub = nh_.advertise<visualization_msgs::MarkerArray>("viz", 1, true);
    visualization_msgs::MarkerArray viz;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0.001;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.color.a = 0.9;
    marker.frame_locked = true;
    viz.markers.push_back(marker);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.3;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 0.8;
    marker.text = "240";
    viz.markers.push_back(marker);
    viz_markers_pub.publish(viz);
}

cv::Point3f ArucoPose::getObjPointsCenter(cv::Mat objPoints) {
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float min_y = min_x, max_y = max_x;
    for (int i = 0; i < objPoints.rows; i++) {
        max_x = std::max(max_x, objPoints.at<float>(i, 0));
        max_y = std::max(max_y, objPoints.at<float>(i, 1));
        min_x = std::min(max_x, objPoints.at<float>(i, 0));
        min_y = std::min(max_y, objPoints.at<float>(i, 1));
    }
    cv::Point3f res((min_x + max_x) / 2, (min_y + max_y) / 2, 0);
    return res;
}

#include "fix.cpp"

void ArucoPose::detect(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &cinfo) {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;

    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    cv::Mat cameraMatrix(3, 3, CV_64F);
    cv::Mat distCoeffs(8, 1, CV_64F);
    parseCameraInfo(cinfo, cameraMatrix, distCoeffs);

    // std::cout << "dist " << distCoeffs << " mat " << cameraMatrix;
    // std::cout << markerIds.size() << std::endl;

    // cv::Vec3d rvec, tvec;
    // int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, rvec, tvec);

    // std::vector< cv::Vec3d > rvecs, tvecs;
    //cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.3362, cameraMatrix, distCoeffs, rvecs, tvecs);
    // cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.15, cameraMatrix, distCoeffs, rvecs, tvecs);


    // Publish markers
    // aruco_pose::MarkerArray markerArray;
    // markerArray.header.frame_id = msg->header.frame_id;
    // markerArray.header.stamp = msg->header.stamp;
    // markerArray.markers.resize(markerIds.size());
    // for (int i = 0; i < markerIds.size(); i++) {
        // markerArray.markers[i].id = markerIds[i];
        // markerArray.markers[i].pose.x = tvect[0];
        // markerArray.markers[i].pose.y = tvect[1];
        // markerArray.markers[i].pose.z = tvect[2];
        // markerArray.markers[i].header.stamp = msg->header.stamp;
        // markerArray.markers[i].header.frame_id = msg->header.frame_id;
    // }
    // marker_pub.publish(markerArray);

    /*
    for (int i = 0; i < markerIds.size(); i++) {
        //if (markerIds[i] == 242) {
        if (markerIds[i] == 9) {
            tf::Transform transform = aruco2tf(rvecs[i], tvecs[i]);
            tf::StampedTransform stampedTransform(transform, msg->header.stamp, msg->header.frame_id, frame_id);
            br.sendTransform(stampedTransform);

            // geometry_msgs::TransformStamped transformStamped;
            // transformStamped.header.stamp = msg->header.stamp;
            // transformStamped.header.frame_id = cinfo->header.frame_id;
            // transformStamped.child_frame_id = frame_id;
            // transformStamped.transform = aruco2tf(rvecs[i], tvecs[i]);
            // transformStamped.transform.translation = transformStamped.transform.translation.normalize();
            // br.sendTransform(transformStamped);
            break;
        }
    }
    */

    // std::cout << "markers: ";
    // for (auto const& c : markerIds) std::cout << c << ' ';

//    return;

    if (markerIds.size() > 0) {
//        for (auto const& c : markerCorners) std::cout << c << ' ';
//        for (auto const& c : markerIds) std::cout << c << ' ';

//        cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);

        // std::vector< cv::Vec3d > rvecs, tvecs;
        // cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.3362, cameraMatrix, distCoeffs, rvecs, tvecs);
        // draw axis for each marker
        // for(int i=0; i<markerIds.size(); i++)
            // cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

        cv::Mat rvec, tvec, objPoints;
        int valid = _estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs,
                                       rvec, tvec, false, objPoints);

        if (valid) {

            tf::StampedTransform transform(aruco2tf(rvec, tvec), msg->header.stamp, cinfo->header.frame_id, frame_id_);
            br.sendTransform(transform);

            // Publish map pose
            static geometry_msgs::PoseStamped ps;
            ps.header.frame_id = frame_id_;
            ps.header.stamp = msg->header.stamp;
            ps.pose.orientation.w = 1;
            pose_pub.publish(ps);

            // Publish reference point
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

//            geometry_msgs::TransformStamped transformMsg;
            // transform.header.stamp = msg->header.stamp;
            // transform.header.frame_id = cinfo->header.frame_id;
            // transform.child_frame_id = frame_id;
            // transform.transform = aruco2tf(rvec, tvec);
//            tf::transformStampedTFToMsg(transform, transformMsg);
//            br.sendTransform(transformMsg);
//            std::cout << rvec << ";" << tvec << std::endl;
//            geometry_msgs::TransformStamped transformStamped;
//            transformStamped.header.stamp = msg->header.stamp;
//            transformStamped.header.frame_id = cinfo->header.frame_id;
//            transformStamped.child_frame_id = frame_id;
//            transformStamped.transform.translation.x = tvec[0];
//            transformStamped.transform.translation.y = tvec[1];
//            transformStamped.transform.translation.z = tvec[1];
//            transformStamped.transform.rotation.w = 1;
//            br.sendTransform(transformStamped);

            if(img_pub.getNumSubscribers() > 0)
            {
                //show input with augmented information
                // for(int i=0; i<markerIds.size(); i++) {
                    // cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                // }

                cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
                cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvec, tvec, 0.3);
                cv_bridge::CvImage out_msg;
                out_msg.header.frame_id = msg->header.frame_id;
                out_msg.header.stamp = msg->header.stamp;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8; // sensor_msgs::image_encodings::RGB8;
                out_msg.image = image;
                img_pub.publish(out_msg.toImageMsg());
            }
        }
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

//    rot = rot.t();          // inverse rotation
    //tvec = -rot * tvec;   // translation of inverse

    // camPose is a 4x4 matrix with the pose of the camera in the object frame
    // cv::Mat camPose = cv::Mat::eye(4, 4, R.type());
    // R.copyTo(camPose.rowRange(0, 3).colRange(0, 3)); // copies R into camPose
    // tvec.copyTo(camPose.rowRange(0, 3).colRange(3, 4)); // copies tvec into camPose

    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));
    tf::Vector3 tf_orig(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
    return tf::Transform(tf_rot, tf_orig);
}

/*
tf::Transform ArucoPose::aruco2tf(cv::Vec3d rvec, cv::Vec3d tvec) {
    cv::Mat rot(3, 3, CV_64FC1);
    // cv::Mat Rvec64;
    // rvec.convertTo(rvec, CV_64FC1);
    cv::Rodrigues(rvec, rot);
    cv::Mat tran64;
    // tvec.convertTo(tran64, CV_64FC1);

    cv::Mat rotate_to_ros(3, 3, CV_64FC1);
    rotate_to_ros.at<float>(0,0) = 1.0;
    rotate_to_ros.at<float>(0,1) = 0.0;
    rotate_to_ros.at<float>(0,2) = 0.0;
    rotate_to_ros.at<float>(1,0) = 0.0;
    rotate_to_ros.at<float>(1,1) = -1.0;
    rotate_to_ros.at<float>(1,2) = 0.0;
    rotate_to_ros.at<float>(2,0) = 0.0;
    rotate_to_ros.at<float>(2,1) = 0.0;
    rotate_to_ros.at<float>(2,2) = -1.0;

    rot = rot*rotate_to_ros.t();

    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    tf::Vector3 tf_orig(tvec[0], tvec[1], tvec[2]);


    return tf::Transform(tf_rot, tf_orig);
}
*/

// tf::Transform ArucoPose::aruco2tf(cv::Vec3d rvec, cv::Vec3d tvec) {
//     /* Code it based on https://github.com/Sahloul/ar_sys/blob/master/src/utils.cpp#L44 */
//     /* TODO: rewrite */

//     cv::Mat rot(3, 3, CV_64FC1);
//     cv::Rodrigues(rvec, rot);

//     cv::Mat rotate_to_sys(3, 3, CV_64FC1);
//     /**
//     /* Fixed the rotation to meet the ROS system
//     /* Doing a basic rotation around X with theta=PI
//     /* By Sahloul
//     /* See http://en.wikipedia.org/wiki/Rotation_matrix for details
//     */

//     //    1    0    0
//     //    0    -1    0
//     //    0    0    -1

//     rotate_to_sys.at<float>(0,0) = 1.0;
//     rotate_to_sys.at<float>(0,1) = 0.0;
//     rotate_to_sys.at<float>(0,2) = 0.0;
//     rotate_to_sys.at<float>(1,0) = 0.0;
//     rotate_to_sys.at<float>(1,1) = -1.0;
//     rotate_to_sys.at<float>(1,2) = 0.0;
//     rotate_to_sys.at<float>(2,0) = 0.0;
//     rotate_to_sys.at<float>(2,1) = 0.0;
//     rotate_to_sys.at<float>(2,2) = -1.0;

//     rot = rot * rotate_to_sys.t();


//     tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
//         rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
//         rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

//     tf::Vector3 tf_orig(tvec[0], tvec[1], tvec[2]);

//     tf::Transform tft(tf_rot, tf_orig);
//     return tft;
// }

PLUGINLIB_EXPORT_CLASS(ArucoPose, nodelet::Nodelet)

}

