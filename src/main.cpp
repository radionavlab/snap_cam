#include "main.h"

void writer(ICameraFrame *frame) 
{
    if(is_writing == true) {
        frame->releaseRef();
        return;
    } else {
        is_writing = true;
    }

    // Write the FPS
    static long long lastTime = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long currentTime = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000L;
    cout << "Video FPS: " << (1000.0 / (double)(currentTime - lastTime)) << endl;
    lastTime = currentTime;

    // Sequence number of images. Increments for every image
    static uint32_t seq = 0;

    // Convert YUV to RGB
    /* Multiple by 1.5 because of YUV standard */
    cv::Mat img = cv::Mat(1.5 * height, width, CV_8UC1, frame->data);
    cv::cvtColor(img, img, CV_YUV420sp2RGB);

    // Compress to JPEG
    std::vector<uint8_t> buff;//buffer for coding
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 100;
    cv::imencode(".jpg", img, buff, param);
    img.release();

    // Write
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << seq;
    std::string filename = "frame" + ss.str() + ".jpg";
    std::string fullPath = save_directory + filename;
    std::ofstream savefile(fullPath.c_str(), ios::out | ios::binary);
    savefile.write((const char*)&buff[0], buff.size());
    savefile.close();

    // Calculate ECEF position of the camera
    Eigen::Matrix<long double, 3, 1> primary_antenna_ECEF;
    primary_antenna_ECEF << solution.roverX,
                            solution.roverY,
                            solution.roverZ;

    const Eigen::Matrix<long double, 3, 1> camera = camera_ECEF(primary_antenna_ECEF, camera_position, -solution.azimuth, -solution.elevation);

    // Write to the info file
    std::string data = "" + 
        std::to_string(camera(0,0)) + " " + 
        std::to_string(camera(1,0)) + " " + 
        std::to_string(camera(2,0)) + " " + 
        "0" + " " + 
        std::to_string(solution.elevation - M_PI/6) + " " + 
        std::to_string(solution.azimuth);


    Eigen::IOFormat CommaInitFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

    std::cout << "PRIMARY AND CAMERA" << std::endl;
    std::cout << (primary_antenna_ECEF).format(CommaInitFmt) << std::endl;
    std::cout << camera.format(CommaInitFmt) << std::endl << std::endl << std::endl; 

    std::string command = "echo '" + filename + " " + data + "' >> " + save_directory + "/image_poses.txt";
    std::system(command.c_str());
    seq++;
    is_writing = false;
    frame->releaseRef();
}

void publisher(ICameraFrame *frame) 
{
    static int seq = 0;

    if(is_writing == true) {
        frame->releaseRef();
        return;
    } else {
        is_writing = true;
    }

    // Write the FPS
    static long long lastTime = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long currentTime = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000L;
    cout << "Video FPS: " << (1000.0 / (double)(currentTime - lastTime)) << endl;
    lastTime = currentTime;

    // Calculate ECEF position of the camera
    Eigen::Matrix<long double, 3, 1> primary_antenna_ECEF;
    primary_antenna_ECEF << solution.roverX,
                            solution.roverY,
                            solution.roverZ;

    const Eigen::Matrix<long double, 3, 1> camera = camera_ECEF(primary_antenna_ECEF, camera_position, -solution.azimuth, -solution.elevation);

    // Compose point messgae
    geometry_msgs::Point pointMsg;
    pointMsg.x = camera(0,0);
    pointMsg.y = camera(1,0);
    pointMsg.z = camera(2,0);

    // Compose quaternion message
    geometry_msgs::Quaternion quaternionMsg;
    quaternionMsg = tf::createQuaternionMsgFromRollPitchYaw(0.0, solution.elevation - M_PI/6.0, solution.azimuth);

    // Compose Pose message
    geometry_msgs::Pose poseMsg;
    poseMsg.position = pointMsg;
    poseMsg.orientation = quaternionMsg;

    // Convert YUV to RGB
    /* Multiple by 1.5 because of YUV standard */
    cv::Mat img = cv::Mat(1.5 * height, width, CV_8UC1, frame->data);
    cv::cvtColor(img, img, CV_YUV420sp2RGB);

    // Compress to JPEG
    std::vector<uint8_t> buff;//buffer for coding
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 100;
    cv::imencode(".jpg", img, buff, param);
    img.release();

    // Compress ros message
    sensor_msgs::CompressedImage im;
    im.format="jpeg";
    im.data = buff;
    im.header.seq = seq;
    im.header.stamp=ros::Time::now();
    im.header.frame_id="";

    // Publish
    camera_image_pub.publish(im);
    camera_position_pub.publish(poseMsg);

    seq++;
    is_writing = false;
    frame->releaseRef();
}

int main(int argc, char **argv)
{
    /* Ros init */
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh("~");

    std::string attitudeTopic;
    if (!nh.getParam("attitude_topic", attitudeTopic)) {
        ROS_WARN("No attitude topic! Exiting!");
        exit(1);
    }

    std::string positionTopic;
    if (!nh.getParam("position_topic", positionTopic)) {
        ROS_WARN("No position topic! Exiting!");
        exit(1);
    }

    std::string cameraImageTopic;
    if (!nh.getParam("camera_image_topic", cameraImageTopic)) {
        ROS_WARN("No camera image topic! Exiting!");
        exit(1);
    }

    std::string cameraPositionTopic;
    if (!nh.getParam("camera_position_topic", cameraPositionTopic)) {
        ROS_WARN("No camera position topic! Exiting!");
        exit(1);
    }

    // Subscribers for precise positioning information
    ros::Subscriber attitudeSubscriber = nh.subscribe(attitudeTopic, 1, attitudeMessageHandler);
    ros::Subscriber positionSubscriber = nh.subscribe(positionTopic, 1, positionMessageHandler);

    /* CAMERA PARAMS */
    if (!nh.getParam("focus_mode", cfg.focusMode)) {
        cfg.focusMode="auto";
        ROS_WARN("Defaulting to auto focus mode.");
    }

    if (!nh.getParam("white_balance", cfg.whiteBalance)) {
        cfg.whiteBalance="auto";
        ROS_WARN("Defaulting to auto white balance.");
    }

    if (!nh.getParam("iso", cfg.ISO)) {
        cfg.ISO="auto";
        ROS_WARN("Defaulting to auto ISO.");
    }

    if (!nh.getParam("preview_format", cfg.previewFormat)) {
        cfg.previewFormat="yuv420sp";
        ROS_WARN("Defaulting to yuv420sp preview format.");
    }

    if (!nh.getParam("brightness", cfg.brightness)) {
        cfg.brightness=3;
        ROS_WARN("Defaulting to 3 brightness");
    }

    if (!nh.getParam("sharpness", cfg.sharpness)) {
        cfg.sharpness=18;
        ROS_WARN("Defaulting to 18 sharpness");
    }

    if (!nh.getParam("contrast", cfg.contrast)) {
        cfg.contrast=5;
        ROS_WARN("Defaulting to 5 contrast.");
    }

    std::string res;
    if (!nh.getParam("camera_resolution", res)) {
        res = "VGA";
        ROS_WARN("No resolution parameter provided. Defaulting to %s.", res.c_str());
    }

    std::string base_directory;
    if (!nh.getParam("base_directory", base_directory)) {
        base_directory = "/home/linaro/";
        ROS_WARN("No save directory provided. Defaulting to home directory.");
    }

    /* Make a directory for all of the images */
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string year = std::to_string(1900 + ltm->tm_year);
    std::string month = std::to_string(1 + ltm->tm_mon);
    std::string day = std::to_string(ltm->tm_mday);
    std::string hour = std::to_string(ltm->tm_hour);
    std::string min = std::to_string(ltm->tm_min);
    std::string sec = std::to_string(ltm->tm_sec);
    std::string dir = year + "-" + month + "-" + day + "-" + hour + "-" + min + "-" + sec + "/";
    save_directory = base_directory + dir;
    std::cout << save_directory << std::endl;
    const int dir_err = mkdir(save_directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == dir_err){
        ROS_WARN("Error creating directory!");
        exit(1);
    }

    std::string callback_mode;
    if (!nh.getParam("callback_mode", callback_mode)) {
        callback_mode = "publish";
        ROS_WARN("No callback_mode provided. Defaulting to publishing.");
    }

    /* Set resolution size */
    // Some of these are nonstandard!
    if (res == "4k") {
        cfg.previewSize = CameraSizes::UHDSize();
        height = 2176;
        width = 3840;
    } else if (res == "1080p") {
        cfg.previewSize = CameraSizes::FHDSize();
        height = 1088;
        width = 1920;
    } else if (res == "720p") {
        cfg.previewSize = CameraSizes::HDSize();
        height = 720;
        width = 1280;
    } else if (res == "VGA") {
        cfg.previewSize = CameraSizes::VGASize();
        height = 480;
        width = 640;
    } else {
        ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
        cfg.previewSize = CameraSizes::stereoVGASize();
    }

    /* Position of the camera with respect to primary antenna in body frame */
    std::vector<double> camera_coordinates;
    if (!nh.getParam("camera_coordinates", camera_coordinates)) {
        camera_coordinates = {0.0, 0.0, 0.0};
        ROS_WARN("No camera coordinates. Setting to zeros.");
    }
    camera_position <<  camera_coordinates[0],
                        camera_coordinates[1],
                        camera_coordinates[2];

    // Only using hires camera
    cfg.func = CAM_FUNC_HIRES;
        
    /* Program start */
    SnapCam cam(cfg);

    /* Set the callback mode */
    if(callback_mode == "write") {
        cam.setListener(writer);
    } else {
        camera_image_pub = nh.advertise<sensor_msgs::CompressedImage>(cameraImageTopic, 1);
        camera_position_pub = nh.advertise<geometry_msgs::Pose>(cameraPositionTopic, 1);
        cam.setListener(publisher);
    }

    /* Main loop */
    ros::Rate r(20);
    while(nh.ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
