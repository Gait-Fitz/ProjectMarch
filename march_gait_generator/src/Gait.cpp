
#include <march_gait_generator/Gait.h>

Gait::Gait(): Gait("Default", "Description", "0.1")
{
    std::vector<std::string> names;
    names.push_back("joint1");
    names.push_back("joint2");
    PoseStamped emptyPose = PoseStamped(names);
    this->addPoseStamped(emptyPose);
}

Gait::Gait(const std::string &name, const std::string &comment, const std::string &version, const ros::Duration &duration)
        : name(name), comment(comment), version(version), duration(duration), poseList({}) {

}

Gait::Gait(const std::string &name, const std::string &comment, const std::string &version)
        : name(name), comment(comment), version(version), duration(ros::Duration(10)) {

}

void Gait::addPoseStamped(int index, PoseStamped poseStamped)
{
    poseList.insert(poseList.begin() + index, poseStamped);
}

void Gait::addPoseStamped(PoseStamped poseStamped)
{
    poseList.push_back(poseStamped);
}
