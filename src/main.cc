#include "main.h"

/* Extern variables */
SensorParams sensorParams;
GPSSolution gpsSolution;

int main(int argc, char **argv) {
    /* Ros init */
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh("~");

    /* Load parameters from launch file */
    loadParams(nh);
 
    /* Subscribers */
    ros::Subscriber attitudeSubscriber = nh.subscribe(attitudeTopic, 1, attitudeMessageHandler);
    ros::Subscriber positionSubscriber = nh.subscribe(positionTopic, 1, positionMessageHandler);

    /* Publishers */
    balloonInfoPublisher = nh.advertise<mg_msgs::BalloonInfo>(balloonTopic, 1);

    /* Camera */
    std::shared_ptr<CamConfig> cfg;
    cfg = initFrontCameraConfig(nh);
    
    cam = std::make_shared<SnapCam>(*cfg);
    cam->setListener(frameHandler);
    cam->start();

    /* Main loop */
    ros::Rate r(20);
    while(nh.ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

void loadParams(const ros::NodeHandle& nh) {
    /* ROS topics */
    nh.getParam("attitude_topic", attitudeTopic);
    nh.getParam("position_topic", positionTopic);
    nh.getParam("balloon_topic", balloonTopic);

    /* Position of camera in body frame */
    std::vector<double> rcB_;
    if (!nh.getParam("rcB", rcB_)) {
        ROS_WARN("Camera position not defined.");
        exit(EXIT_FAILURE);
    }

    sensorParams.rcB << rcB_[0],
                        rcB_[1],
                        rcB_[2];

    /* Position of primary in body frame */
    std::vector<double> rpB_;
    if (!nh.getParam("rpB", rpB_)) {
        ROS_WARN("Primary position not defined.");
        exit(EXIT_FAILURE);
    }

    sensorParams.rpB << rpB_[0],
                        rpB_[1],
                        rpB_[2];

    /* Position of reference in ECEF frame */
    std::vector<double> rrG_;
    if (!nh.getParam("rrG", rrG_)) {
        ROS_WARN("Reference position not defined.");
        exit(EXIT_FAILURE);
    }

    sensorParams.rrG << rrG_[0],
                        rrG_[1],
                        rrG_[2];

    /* Position of inertial in ECEF frame */
    std::vector<double> riG_;
    if (!nh.getParam("riG", riG_)) {
        ROS_WARN("Inertial position not defined.");
        exit(EXIT_FAILURE);
    }

    sensorParams.riG << riG_[0],
                        riG_[1],
                        riG_[2];

    /* Focal Length of camera */
    double f_;
    if (!nh.getParam("f", f_)) {
        ROS_WARN("Focal length not defined.");
        exit(EXIT_FAILURE);
    }

    sensorParams.f = f_;

    /* Focal Length of camera */
    double k1_;
    if (!nh.getParam("k1", k1_)) {
        ROS_WARN("Camera intrinsics not defined.");
        exit(EXIT_FAILURE);
    }

    sensorParams.k1 = k1_;
}

void frameHandler(ICameraFrame *frame) {
    // Check that an image is not already being processed
    if(isCameraBusy == true) {
        frame->releaseRef();
        return;
    } else {
        isCameraBusy = true;
    }

    // Record where the image was taken.
    double antennaPos[] = {gpsSolution.x, gpsSolution.y, gpsSolution.z};
    double antennaAtt[] = {0, gpsSolution.el, gpsSolution.az};

    // Convert YUV to RGB
    cv::Mat img;
    img = cv::Mat(1.5 * sensorParams.imageHeight, sensorParams.imageWidth, CV_8UC1, frame->data);
    cv::cvtColor(img, img, CV_YUV420sp2RGB);

    // Release the camera buffer
    frame->releaseRef();

    // Process the balloon info
    const BalloonInfo balloonInfo = processImage(img);

    // Report the camera location
    mg_msgs::BalloonInfo balloonInfoMsg;
    balloonInfoMsg.balloonPos.x = balloonInfo.balloonLocation(0);
    balloonInfoMsg.balloonPos.y = balloonInfo.balloonLocation(1);
    balloonInfoMsg.balloonPos.z = balloonInfo.balloonLocation(2);
    balloonInfoMsg.antennaPos.x = antennaPos[0];
    balloonInfoMsg.antennaPos.y = antennaPos[1];
    balloonInfoMsg.antennaPos.z = antennaPos[2];
    balloonInfoMsg.roll.data    = antennaAtt[0];
    balloonInfoMsg.pitch.data   = antennaAtt[1];
    balloonInfoMsg.yaw.data     = antennaAtt[2];
    balloonInfoMsg.rad.data     = balloonInfo.balloonRadius;
    balloonInfoMsg.color.data   = balloonInfo.color;

    balloonInfoPublisher.publish(balloonInfoMsg);

    // Release the image
    img.release();

    // Finished processing
    isCameraBusy = false;
}

std::shared_ptr<CamConfig> initFrontCameraConfig(ros::NodeHandle& nh) {
    std::shared_ptr<CamConfig> config = std::make_shared<CamConfig>();

    nh.getParam("focus_mode",       config->focusMode);
    nh.getParam("white_balance",    config->whiteBalance);
    nh.getParam("iso",              config->ISO);
    nh.getParam("preview_format",   config->previewFormat);
    nh.getParam("brightness",       config->brightness);
    nh.getParam("sharpness",        config->sharpness);
    nh.getParam("contrast",         config->contrast);

    /* Set resolution size */
    /* Some of these are nonstandard! */
    std::string res;
    nh.getParam("camera_resolution", res);
    if (res == "4k") {
        config->previewSize = CameraSizes::UHDSize();
        sensorParams.imageHeight = 2176;
        sensorParams.imageWidth = 3840;
    } else if (res == "1080p") {
        config->previewSize = CameraSizes::FHDSize();
        sensorParams.imageHeight = 1088;
        sensorParams.imageWidth = 1920;
    } else if (res == "720p") {
        config->previewSize = CameraSizes::HDSize();
        sensorParams.imageHeight = 736;
        sensorParams.imageWidth = 1280;
    } else if (res == "VGA") {
        config->previewSize = CameraSizes::VGASize();
        sensorParams.imageHeight = 480;
        sensorParams.imageWidth = 640;
    } else {
        ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
        config->previewSize = CameraSizes::stereoVGASize();
        sensorParams.imageHeight = 480;
        sensorParams.imageWidth = 640;
    }

    config->exposure        = 100;
    config->gain            = 50;
    config->cameraId        = 1;
    config->func            = CAM_FUNC_HIRES;

    return config;
}
