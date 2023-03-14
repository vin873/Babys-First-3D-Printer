#ifndef _ROBOT_CLASS_H_
#define _ROBOT_CLASS_H_

// msgs
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"

// other
#include <cmath>
#include <queue>
#include <vector>

namespace _ROBOT_CLASS_ {

enum class ROBOT_TYPE {
    Robot,
    Robot1,
    Robot2,
    Rival,
    Rival1,
    Rival2
};

class ROBOT_STATE {
   public:
    ROBOT_STATE();
    ROBOT_STATE(ROBOT_TYPE type);

    void SetRobotType(ROBOT_TYPE type);
    ROBOT_TYPE GetRobotType();

    void SetPath(nav_msgs::Path msgs);
    void SetPath(std::queue<geometry_msgs::Pose> GoalPath);
    void SetPosition(geometry_msgs::Pose Pose);
    void SetRPY(geometry_msgs::Pose Pose);
    geometry_msgs::Pose GetPosition();

    void Reset();

    bool GoToNextPoint();
    bool isReach(geometry_msgs::Pose pose);
    bool isReach(geometry_msgs::Point pose);

    //    private:

    geometry_msgs::Pose Position;
    geometry_msgs::Twist Velocity;

    std::queue<geometry_msgs::Pose> Path;

    ROBOT_TYPE RobotType;
};

}  // namespace _ROBOT_CLASS_

#endif