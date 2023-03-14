#include "path_layer/RobotClass.h"

namespace _ROBOT_CLASS_ {

ROBOT_STATE::ROBOT_STATE() {
    Reset();
}

ROBOT_STATE::ROBOT_STATE(ROBOT_TYPE type) {
    Reset();
    SetRobotType(type);
}

void ROBOT_STATE::Reset() {
    Position.position.x = Position.position.y = Position.orientation.w = 0.0;
    Velocity.linear.x = Velocity.linear.y = Velocity.angular.z = 0.0;
    RobotType = ROBOT_TYPE::Robot;
    while (!Path.empty()) {
        Path.pop();
    }
}

void ROBOT_STATE::SetPath(nav_msgs::Path msgs) {
    int size = msgs.poses.size();
    for (int i = 0; i < size; i++) {
        Path.push(msgs.poses[i].pose);
    }
}

void ROBOT_STATE::SetPath(std::queue<geometry_msgs::Pose> GoalPath) {
    this->Path = GoalPath;
}

void ROBOT_STATE::SetPosition(geometry_msgs::Pose Pose) {
    this->Position = Pose;
}

void ROBOT_STATE::SetRPY(geometry_msgs::Pose Pose) {
    this->Position.orientation.z = Pose.orientation.z;
    this->Position.orientation.w = Pose.orientation.w;
}

geometry_msgs::Pose ROBOT_STATE::GetPosition() {
    return this->Position;
}

bool ROBOT_STATE::GoToNextPoint() {
    if (Path.empty()) {
        return false;
    }

    Position = Path.front();
    Path.pop();

    return true;
}

bool ROBOT_STATE::isReach(geometry_msgs::Pose pose) {
    return (sqrt(pow(this->Position.position.x - pose.position.x, 2) + pow(this->Position.position.y - pose.position.y, 2)) < 0.02) ? true : false;
}

bool ROBOT_STATE::isReach(geometry_msgs::Point point) {
    return (sqrt(pow(this->Position.position.x - point.x, 2) + pow(this->Position.position.y - point.y, 2)) < 0.04) ? true : false;
}

void ROBOT_STATE::SetRobotType(ROBOT_TYPE type) {
    this->RobotType = type;
}

ROBOT_TYPE ROBOT_STATE::GetRobotType() {
    return RobotType;
}

}  // namespace _ROBOT_CLASS_
