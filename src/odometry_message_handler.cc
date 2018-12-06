// Author: Tucker Haydon
#include "odometry_message_handler.h"
#include "odometry_buffer.h"

// Extern Variable
// OdometryBuffer odometry_buffer;

void OdometryMessageHandler(const nav_msgs::Odometry odometry_msg) {
  odometry_buffer.SetOdometryMsg(odometry_msg);
}
