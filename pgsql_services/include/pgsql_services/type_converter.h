#ifndef POSE_JSON_CONVERTER_H
#define POSE_JSON_CONVERTER_H

#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>

namespace pose_json_converter {

// Convert Pose to JSON
nlohmann::json PoseToJson(const geometry_msgs::msg::Pose& pose) {
    nlohmann::json j;
    j["position"]["x"] = pose.position.x;
    j["position"]["y"] = pose.position.y;
    j["position"]["z"] = pose.position.z;
    j["orientation"]["x"] = pose.orientation.x;
    j["orientation"]["y"] = pose.orientation.y;
    j["orientation"]["z"] = pose.orientation.z;
    j["orientation"]["w"] = pose.orientation.w;
    return j;
}

// Convert JSON to Pose
geometry_msgs::msg::Pose JsonToPose(const nlohmann::json& j) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = j["position"]["x"];
    pose.position.y = j["position"]["y"];
    pose.position.z = j["position"]["z"];
    pose.orientation.x = j["orientation"]["x"];
    pose.orientation.y = j["orientation"]["y"];
    pose.orientation.z = j["orientation"]["z"];
    pose.orientation.w = j["orientation"]["w"];
    return pose;
}

} // namespace pose_json_converter

#endif // POSE_JSON_CONVERTER_H
