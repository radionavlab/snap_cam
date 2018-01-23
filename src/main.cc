#include "main.h"

int main(int argc, char **argv) {
    /* Ros init */
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh("~");

    /* Ros topics */
    std::string attitudeTopic;
    nh.getParam("attitude_topic", attitudeTopic);

    std::string positionTopic;
    nh.getParam("position_topic", positionTopic);

    // Various init functions
    read_camera_position(nh);
 
    /* Subscribers */
    ros::Subscriber attitudeSubscriber = nh.subscribe(attitudeTopic, 1, attitudeMessageHandler);
    ros::Subscriber positionSubscriber = nh.subscribe(positionTopic, 1, positionMessageHandler);

    /* Camera */
    std::shared_ptr<CamConfig> cfg;
    cfg = init_front_camera_config(nh);
    
    cam = std::make_shared<SnapCam>(*cfg);
    cam->setListener(frame_handler);
    cam->start();

    /* Main loop */
    ros::Rate r(20);
    while(nh.ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

void frame_handler(ICameraFrame *frame) {
    // Check that an image is not already being processed
    if(camera_busy == true) {
        frame->releaseRef();
        return;
    } else {
        camera_busy = true;
    }

    // Convert YUV to RGB
    cv::Mat img;
    img = cv::Mat(1.5 * camera_height, camera_width, CV_8UC1, frame->data);
    cv::cvtColor(img, img, CV_YUV420sp2RGB);

    // Release the camera buffer
    frame->releaseRef();

    // Calculate the position of the camera
    Eigen::Matrix<long double, 3, 1> camera_position;
    calc_camera_position(camera_position);

    Eigen::Matrix<long double, 3, 1> camera_orientation;
    camera_orientation << 0,
                          solution.elevation - M_PI/6,
                          solution.azimuth;

    // Pass image and camera pose to student callback
    callback(img, camera_position, camera_orientation);

    // Release the image
    img.release();

    // Finished processing
    camera_busy = false;
}

// Calculate ECEF position of the camera
void calc_camera_position(
    Eigen::Matrix<long double, 3, 1>& camera_position) {
        Eigen::Matrix<long double, 3, 1> primary_antenna_ECEF;
        primary_antenna_ECEF << solution.roverX,
                                solution.roverY,
                                solution.roverZ;

        camera_position = camera_ECEF(primary_antenna_ECEF, camera_body_position, -solution.azimuth, -solution.elevation);
}

void read_camera_position(ros::NodeHandle& nh) {
    /* Position of the camera with respect to primary antenna in body frame */
    std::vector<double> camera_coordinates;
    if (!nh.getParam("camera_coordinates", camera_coordinates)) {
        camera_coordinates = {0.0, 0.0, 0.0};
        ROS_WARN("No camera coordinates. Setting to zeros.");
    }

    camera_body_position << camera_coordinates[0],
                            camera_coordinates[1],
                            camera_coordinates[2];
}

std::shared_ptr<CamConfig> init_front_camera_config(ros::NodeHandle& nh) {
    std::shared_ptr<CamConfig> config = std::make_shared<CamConfig>();

    nh.getParam("focus_mode",       config->focusMode);
    nh.getParam("white_balance",    config->whiteBalance);
    nh.getParam("iso",              config->ISO);
    nh.getParam("preview_format",   config->previewFormat);
    nh.getParam("brightness",       config->brightness);
    nh.getParam("sharpness",        config->sharpness);
    nh.getParam("contrast",         config->contrast);

    /* Set resolution size */
    // Some of these are nonstandard!
    std::string res;
    nh.getParam("camera_resolution", res);
    if (res == "4k") {
        config->previewSize = CameraSizes::UHDSize();
        camera_height = 2176;
        camera_width = 3840;
    } else if (res == "1080p") {
        config->previewSize = CameraSizes::FHDSize();
        camera_height = 1088;
        camera_width = 1920;
    } else if (res == "720p") {
        config->previewSize = CameraSizes::HDSize();
        camera_height = 736;
        camera_width = 1280;
    } else if (res == "VGA") {
        config->previewSize = CameraSizes::VGASize();
        camera_height = 480;
        camera_width = 640;
    } else {
        ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
        config->previewSize = CameraSizes::stereoVGASize();
        camera_height = 480;
        camera_width = 640;
    }

    config->exposure        = 100;
    config->gain            = 50;
    config->cameraId        = 1;
    config->func            = CAM_FUNC_HIRES;

    return config;
}

void attitudeMessageHandler(const gbx_ros_bridge_msgs::Attitude2D msg) {
    const double rx = msg.rx;
    const double ry = msg.ry;
    const double rz = msg.rz;
    const double rxRov = msg.rxRov;
    const double ryRov = msg.ryRov;
    const double rzRov = msg.rzRov;
    const gbx_ros_bridge_msgs::BaseTime tSolution = msg.tSolution;
    const double deltRSec = msg.deltRSec;
    const std::vector<float> P = msg.P;
    const uint32_t nCov = msg.nCov;
    const double azAngle = msg.azAngle;
    const double elAngle = msg.elAngle;
    const double azSigma = msg.azAngle;
    const double elSigma = msg.elSigma;
    const double testStat = msg.testStat;
    const uint8_t numDD = msg.numDD;
    const uint8_t bitfield = msg.bitfield;

    solution.elevation = elAngle;
    solution.azimuth = azAngle;
}

void positionMessageHandler(const gbx_ros_bridge_msgs::SingleBaselineRTK msg) {
    const double rx = msg.rx;
    const double ry = msg.ry;
    const double rz = msg.rz;
    const double rxRov = msg.rxRov;
    const double ryRov = msg.ryRov;
    const double rzRov = msg.rzRov;
    const gbx_ros_bridge_msgs::BaseTime tSolution = msg.tSolution;
    const double deltRSec = msg.deltRSec;
    const std::vector<float> P = msg.P;
    const uint32_t nCov = msg.nCov;
    const double testStat = msg.testStat;
    const double ageOfReferenceData = msg.ageOfReferenceData;
    const uint8_t numDD = msg.numDD;
    const uint8_t bitfield = msg.bitfield;

    solution.roverX = rxRov;
    solution.roverY = ryRov;
    solution.roverZ = rzRov;
}
