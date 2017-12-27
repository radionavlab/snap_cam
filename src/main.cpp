#include "main.h"

void frame_handler(ICameraFrame *frame) {
    static int seq = 0;

    // Check that an image is not already being processed
    if( front_camera_busy == true) {
        frame->releaseRef();
        return;
    } else {
        front_camera_busy = true;
    }

    // Print the framerate
    print_fps();

    // Convert YUV to RGB
    /* Multiple by 1.5 because of YUV standard */
    cv::Mat img = cv::Mat(1.5 * front_height, front_width, CV_8UC1, frame->data);
    cv::cvtColor(img, img, CV_YUV420sp2RGB);
    frame->releaseRef();

    // Compress to JPEG
    std::vector<uint8_t> buff;
    compress_image(buff, img, 100);
    img.release();

    // Calculate the position of the camera
    Eigen::Matrix<long double, 3, 1> camera_position;
    calc_camera_position(camera_position);

    if(save_image_option == true) {
        save_image(buff, seq);
        save_camera_position(camera_position, seq);
    }

    if(publish_image_option == true) {
        sensor_msgs::CompressedImage image_msg;
        create_compressed_image_message(image_msg, buff, seq);

        geometry_msgs::Pose pose_msg;
        create_camera_position_message(pose_msg, camera_position);

        camera_front_image_pub.publish(image_msg);
        camera_position_pub.publish(pose_msg);
    }

    seq++;
    front_camera_busy = false;
}

void print_fps() {
    static long long lastTime = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long currentTime = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000L;
    std::cout << "Front FPS: " << (1000.0 / (double)(currentTime - lastTime)) << std::endl;
    lastTime = currentTime;
}

void create_camera_position_message(
    geometry_msgs::Pose& pose_msg,
    Eigen::Matrix<long double, 3, 1>& camera_position) {
        // Compose point messgae
        geometry_msgs::Point point_msg;
        point_msg.x = camera_position(0,0);
        point_msg.y = camera_position(1,0);
        point_msg.z = camera_position(2,0);

        // Compose quaternion message
        geometry_msgs::Quaternion quaternion_msg;
        quaternion_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0, solution.elevation - M_PI/6.0, solution.azimuth);

        // Compose Pose message
        pose_msg.position = point_msg;
        pose_msg.orientation = quaternion_msg;
}

void save_image(
    std::vector<uint8_t>& buff,
    const int seq) {
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << seq;

        const std::string filename = "frame" + ss.str() + ".jpg";
        const std::string fullPath = save_directory + filename;

        std::ofstream savefile(fullPath.c_str(), ios::out | ios::binary);
        savefile.write((const char*)&buff[0], buff.size());
        savefile.close();
}

/*
void save_image_time() {
    // Get ROS time 
    ros::Time now = ros::Time::now();
        std::to_string(solution.week) + " " + 
        std::to_string(solution.secondsOfWeek) + " " + 
        std::to_string(solution.fractionOfSecond) + " " + 
        std::to_string(now.sec) + " " + 
        std::to_string(now.nsec);
}
*/

void save_camera_position(
    Eigen::Matrix<long double, 3, 1>& camera_position,
    const int seq) {
        static const Eigen::IOFormat CommaInitFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << seq;
        const std::string filename = "frame" + ss.str() + ".jpg";

        std::string data = "" + 
            std::to_string(camera_position(0,0)) + " " +            // X (ECEF)
            std::to_string(camera_position(1,0)) + " " +            // Y (ECEF)
            std::to_string(camera_position(2,0)) + " " +            // Z (ECEF)
            "0" + " " +                                             // Roll (body frame)
            std::to_string(solution.elevation - M_PI/6) + " " +     // Pitch (body frame)
            std::to_string(solution.azimuth);                       // Yaw (ENU frame)

        std::string command = "echo '" + filename + " " + data + "' >> " + save_directory + "/image_poses.txt";
        std::system(command.c_str());
}

// Calculate ECEF position of the camera
void calc_camera_position(
    Eigen::Matrix<long double, 3, 1>& camera_position) {
        Eigen::Matrix<long double, 3, 1> primary_antenna_ECEF;
        primary_antenna_ECEF << solution.roverX,
                                solution.roverY,
                                solution.roverZ;

        camera_position = camera_ECEF(primary_antenna_ECEF, camera_position, -solution.azimuth, -solution.elevation);
}

int main(int argc, char **argv)
{
    /* Ros init */
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh("~");

    /* Ros topics */
    std::string attitudeTopic;
    nh.getParam("attitude_topic", attitudeTopic);

    std::string positionTopic;
    nh.getParam("position_topic", positionTopic);

    std::string cameraImageTopic;
    nh.getParam("camera_image_topic", cameraImageTopic);

    std::string cameraPositionTopic;
    nh.getParam("camera_position_topic", cameraPositionTopic);

    // Various init functions
    create_images_directory(nh);
    set_callback_mode(nh);
    read_camera_position(nh);
 
    // Subscribers
    ros::Subscriber attitudeSubscriber = nh.subscribe(attitudeTopic, 1, attitudeMessageHandler);
    ros::Subscriber positionSubscriber = nh.subscribe(positionTopic, 1, positionMessageHandler);

    // Publishers
    camera_front_image_pub = nh.advertise<sensor_msgs::CompressedImage>(cameraImageTopic+"front/compressed", 1);
    camera_down_image_pub = nh.advertise<sensor_msgs::CompressedImage>(cameraImageTopic+"down/compressed", 1);
    camera_position_pub = nh.advertise<geometry_msgs::Pose>(cameraPositionTopic, 1);

    // Front camera
    std::shared_ptr<CamConfig> cfg_front = init_front_camera_config(nh);
    SnapCam cam_front(*cfg_front);
    cam_front.setListener(frame_handler);

    // Down camera
    std::shared_ptr<CamConfig> cfg_down = init_down_camera_config();
    SnapCam cam_down(*cfg_down);
    cam_down.setListener(down_callback);

    // Start the cameras
    std::thread(&SnapCam::start, &cam_front).detach();
    std::thread(&SnapCam::start, &cam_down).detach();

    /* Main loop */
    ros::Rate r(20);
    while(nh.ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

void set_callback_mode(ros::NodeHandle& nh) {
    std::string callback_mode;
    nh.getParam("callback_mode", callback_mode);

    if(callback_mode == "publish")  publish_image_option= true;
    if(callback_mode == "save")     save_image_option   = true;
    if(callback_mode == "both") {
        publish_image_option = true;
        save_image_option = true;
    }
}

void create_images_directory(ros::NodeHandle& nh) {
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
}

void read_camera_position(ros::NodeHandle& nh) {
    /* Position of the camera with respect to primary antenna in body frame */
    std::vector<double> camera_coordinates;
    if (!nh.getParam("camera_coordinates", camera_coordinates)) {
        camera_coordinates = {0.0, 0.0, 0.0};
        ROS_WARN("No camera coordinates. Setting to zeros.");
    }

    camera_position <<  camera_coordinates[0],
                        camera_coordinates[1],
                        camera_coordinates[2];
}

void down_callback(ICameraFrame *frame) {
    static int seq = 0;

    if(down_camera_busy == true) {
        frame->releaseRef();
        return;
    } else {
        down_camera_busy = true;
    }

    static long long lastTime = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long currentTime = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000L;
    long long diff = currentTime - lastTime;
    lastTime = currentTime;
    cout << "Down FPS: " << (1000.0 / (double)diff) << endl;

    // Convert YUV to RGB
    cv::Mat img = cv::Mat(down_height, down_width, CV_8UC1, frame->data);
    frame->releaseRef();

    // Compress to JPEG
    std::vector<uint8_t> buff;
    compress_image(buff, img, 100);
    img.release();

    // Compose ros message
    sensor_msgs::CompressedImage msg;
    create_compressed_image_message(msg, buff, seq);

    // Publish
    camera_down_image_pub.publish(msg);

    seq++;
    down_camera_busy = false;
}

void compress_image(
    std::vector<uint8_t>& buff, 
    cv::Mat& img, 
    const int compression_quality) {
        std::vector<int> param(2);
        param[0] = cv::IMWRITE_JPEG_QUALITY;
        param[1] = compression_quality;
        cv::imencode(".jpg", img, buff, param);
}

void create_compressed_image_message(
    sensor_msgs::CompressedImage& im,
    std::vector<uint8_t>& buff,
    const int seq){
        im.format           = "jpeg";
        im.data             = buff;
        im.header.seq       = seq;
        im.header.stamp     = ros::Time::now();
        im.header.frame_id  = "";
}


std::shared_ptr<CamConfig> init_down_camera_config() {
    std::shared_ptr<CamConfig> config = std::make_shared<CamConfig>();
    config->cameraId       = 0;
    config->focusMode      = "auto";
    config->whiteBalance   = "auto";
    config->ISO            = "auto";
    config->previewFormat  = "yuv420sp";
    config->sharpness      = 18;
    config->brightness     = 3;
    config->contrast       = 5;
    config->exposure       = 100;
    config->gain           = 50;
    config->previewSize    = CameraSizes::VGASize();
    config->pictureSize    = CameraSizes::VGASize();
    config->func = CAM_FUNC_OPTIC_FLOW;

    down_height = 480;
    down_width = 640;

    return config;
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
        front_height = 2176;
        front_width = 3840;
    } else if (res == "1080p") {
        config->previewSize = CameraSizes::FHDSize();
        front_height = 1088;
        front_width = 1920;
    } else if (res == "720p") {
        config->previewSize = CameraSizes::HDSize();
        front_height = 736;
        front_width = 1280;
    } else if (res == "VGA") {
        config->previewSize = CameraSizes::VGASize();
        front_height = 480;
        front_width = 640;
    } else {
        ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
        config->previewSize = CameraSizes::stereoVGASize();
        front_height = 480;
        front_width = 640;
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

    solution.week = tSolution.week;
    solution.secondsOfWeek = tSolution.secondsOfWeek;
    solution.fractionOfSecond = tSolution.fractionOfSecond;
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
