// ROS2
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// MSG
// #include "pgsql_interfaces/msg/chemical_location.hpp"
// #include "pgsql_interfaces/msg/workstation_location.hpp"

// SRV
#include "pgsql_interfaces/srv/get_vessel.hpp"
#include "pgsql_interfaces/srv/get_chemical.hpp"
#include "pgsql_interfaces/srv/place_vessel.hpp"
#include "pgsql_interfaces/srv/place_chemical.hpp"
#include "pgsql_interfaces/srv/remove_chemical_placement.hpp"
#include "pgsql_interfaces/srv/remove_vessel_placement.hpp"

// LIBS
#include "pgsql_services/type_converter.h"

#include <nlohmann/json.hpp>
#include <pqxx/pqxx>
#include "csv.h"

#include <iostream>


class DatabaseService : public rclcpp::Node {
public:
    DatabaseService() : Node("database_service") {
        this->db_name = this->declare_parameter<std::string>("db_name", "postgres");
        this->db_user = this->declare_parameter<std::string>("db_user", "postgres");
        this->db_password = this->declare_parameter<std::string>("db_password", "your-super-secret-and-long-postgres-password");
        this->db_host = this->declare_parameter<std::string>("db_host", "127.0.0.1");
        this->db_port = this->declare_parameter<std::string>("db_port", "5432");


        get_vessel_service_ = this->create_service<pgsql_interfaces::srv::GetVessel>(
            "get_vessel",
            std::bind(&DatabaseService::get_vessel, this, std::placeholders::_1, std::placeholders::_2));

        get_chemical_service_ = this->create_service<pgsql_interfaces::srv::GetChemical>(
            "get_chemical",
            std::bind(&DatabaseService::get_chemical, this, std::placeholders::_1, std::placeholders::_2));

        place_vessel_service_ = this->create_service<pgsql_interfaces::srv::PlaceVessel>(
            "place_vessel",
            std::bind(&DatabaseService::place_vessel, this, std::placeholders::_1, std::placeholders::_2));

        place_chemical_service_ = this->create_service<pgsql_interfaces::srv::PlaceChemical>(
            "place_chemical",
            std::bind(&DatabaseService::place_chemical, this, std::placeholders::_1, std::placeholders::_2));


        remove_chemical_placement_service_ = this->create_service<pgsql_interfaces::srv::RemoveChemicalPlacement>(
            "remove_chemical_placement",
            std::bind(&DatabaseService::remove_chemical_placement, this, std::placeholders::_1, std::placeholders::_2));

        remove_vessel_placement_service_ = this->create_service<pgsql_interfaces::srv::RemoveVesselPlacement>(
            "remove_vessel_placement",
            std::bind(&DatabaseService::remove_vessel_placement, this, std::placeholders::_1, std::placeholders::_2));
    }




private:
    std::string db_name;
    std::string db_user;
    std::string db_password;
    std::string db_host;
    std::string db_port;


    // ros2 service call /get_vessel pgsql_interfaces/srv/GetVessel "{name: 'vessel_name'}"
    void get_vessel(const std::shared_ptr<pgsql_interfaces::srv::GetVessel::Request> request,
                    std::shared_ptr<pgsql_interfaces::srv::GetVessel::Response> response) {
        try {
            pqxx::connection C("dbname=" + db_name + " user=" + db_user +
                            " password=" + db_password + " hostaddr=" + db_host +
                            " port=" + db_port);
            pqxx::work W(C); // Start a transaction

            std::string query = "SELECT vessel_id FROM vessel WHERE name = " + W.quote(request->name) + ";";
            pqxx::result result = W.exec(query);

            if (result.empty()) {
                response->success = false;
                response->message = "Vessel not found";
                return;
            }

            for (auto row : result) {
                int vessel_id = row["vessel_id"].as<int>();
                std::string placement_query = "SELECT * FROM vessel_placements WHERE vessel_id = " + W.quote(vessel_id) + ";";
                pqxx::result placement_result = W.exec(placement_query);

                if (placement_result.empty()) {
                    response->success = false;
                    response->message = "Vessel has no placements";
                    return;
                }

                for (auto new_row : placement_result) {
                    int slot_id = new_row["slot_id"].as<int>();
                    std::string slot_query = "SELECT * FROM tray_slot WHERE slot_id = " + W.quote(slot_id) + ";";
                    pqxx::result slot_result = W.exec(slot_query);

                    for (auto row : slot_result) {
                        int tray_id = row["tray_id"].as<int>();
                        std::string tray_query = "SELECT workstation_id, type, aruco_id FROM tray WHERE tray_id = " + W.quote(tray_id) + ";";
                        pqxx::result tray_result = W.exec(tray_query);

                        for (auto tray_row : tray_result) {
                            int workstation_id = tray_row["workstation_id"].as<int>();
                            std::string tray_type = tray_row["type"].as<std::string>();
                            int aruco_id = tray_row["aruco_id"].as<int>();

                            std::string package_share_directory = ament_index_cpp::get_package_share_directory("pgsql_services");
                            std::string aruco_to_first_slot_csv = package_share_directory + "/data/" + tray_type + "_aruco_to_first_slot.csv";
                            std::string first_slot_to_every_slot_csv = package_share_directory + "/data/" + tray_type + "_first_slot_to_every_slot.csv";

                            geometry_msgs::msg::TransformStamped aruco_to_first_slot_transform = read_transform_from_csv(aruco_to_first_slot_csv);
                            aruco_to_first_slot_transform.header.frame_id = "panda_link0"; // replace with your frame ID
                            aruco_to_first_slot_transform.header.stamp = this->get_clock()->now(); // set the timestamp to the current time

                            geometry_msgs::msg::TransformStamped first_slot_to_current_slot_transform = read_transform_from_csv(first_slot_to_every_slot_csv, slot_id);
                            first_slot_to_current_slot_transform.header.frame_id = "panda_link0"; // replace with your frame ID
                            first_slot_to_current_slot_transform.header.stamp = this->get_clock()->now(); // set the timestamp to the current time

                            std::string workstation_query = "SELECT name, lookout_pose FROM workstation WHERE workstation_id = " + W.quote(workstation_id) + ";";

                            pqxx::result workstation_result = W.exec(workstation_query);

                            for (auto workstation_row : workstation_result) {
                                std::string name = workstation_row["name"].as<std::string>();
                                std::string lookout_pose_json = workstation_row["lookout_pose"].as<std::string>();

                                nlohmann::json j = nlohmann::json::parse(lookout_pose_json);
                                geometry_msgs::msg::Pose lookout_pose = pose_json_converter::JsonToPose(j);
                        
                                std_msgs::msg::Header header;
                                header.frame_id = "panda_link0";
                                header.stamp = this->get_clock()->now(); // Set the timestamp to the current time

                                geometry_msgs::msg::PoseStamped lookout_pose_stamped;
                                lookout_pose_stamped.header = header;
                                lookout_pose_stamped.pose = lookout_pose;

                                response->workstation_name = name;
                                response->lookout_pose = lookout_pose_stamped;
                                response->aruco_id = aruco_id; // You need to get this value from somewhere
                                response->aruco_to_slot_transform = aruco_to_first_slot_transform;
                                response->slot_to_slot_transform = first_slot_to_current_slot_transform;
                                response->success = true;
                                response->message = "Vessel found";
                          }
                    }
                }
            }
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = std::string("Caught exception: ") + e.what();
    } catch (...) {
        response->success = false;
        response->message = "Caught unknown exception";
    }
}

    // ros2 service call /get_chemical pgsql_interfaces/srv/GetChemical "{name: 'chemical_name'}"
    void get_chemical(const std::shared_ptr<pgsql_interfaces::srv::GetChemical::Request> request,
                    std::shared_ptr<pgsql_interfaces::srv::GetChemical::Response> response) { 

        // std::cout << request << std::endl;

        try {
            pqxx::connection C("dbname=" + db_name + " user=" + db_user +
                            " password=" + db_password + " hostaddr=" + db_host +
                            " port=" + db_port);
            pqxx::work W(C); // Start a transaction

            std::string query = "SELECT chemical_id FROM chemical WHERE name = " + W.quote(request->name) + ";";
            pqxx::result result = W.exec(query);

            if (result.empty()) {
                response->success = false;
                response->message = "Chemical not found";
                return;
            }


            if (request->type != "pick" && request->type != "pour") {
                response->success = false;
                response->message = "Type should be 'pick' or 'pour'";
                return;
            }

            for (auto row : result) {
                int chemical_id = row["chemical_id"].as<int>();
                std::string placement_query = "SELECT * FROM chemical_placements WHERE chemical_id = " + W.quote(chemical_id) + ";";
                pqxx::result placement_result = W.exec(placement_query);

                if (placement_result.empty()) {
                    response->success = false;
                    response->message = "Chemical has no placements";
                    return;
                }

                for (auto new_row : placement_result) {
                    int slot_id = new_row["slot_id"].as<int>();
                    bool empty_bottle = new_row["empty"].as<bool>();
                    std::string slot_query = "SELECT * FROM tray_slot WHERE slot_id = " + W.quote(slot_id) + ";";
                    pqxx::result slot_result = W.exec(slot_query);

                    for (auto row : slot_result) {
                        int tray_id = row["tray_id"].as<int>();
                        std::string tray_query = "SELECT workstation_id, type, aruco_id FROM tray WHERE tray_id = " + W.quote(tray_id) + ";";
                        pqxx::result tray_result = W.exec(tray_query);

                        for (auto tray_row : tray_result) {
                            int workstation_id = tray_row["workstation_id"].as<int>();
                            std::string tray_type = tray_row["type"].as<std::string>();
                            int aruco_id = tray_row["aruco_id"].as<int>();


                            std::string map_query = "SELECT slot_id FROM tray_slot WHERE tray_id = " + std::to_string(tray_id) + " ORDER BY slot_id" + ";";
                            pqxx::result map_result = W.exec(placement_query);
                            // Map the slot_ids in the range 1, 2, 3 ... sizeof(tray_id) to the tray_id
                            std::map<int, int> slot_id_map;
                            for (size_t i = 0; i < map_result.size(); ++i) {
                                slot_id_map[map_result[i][0].as<int>()] = i + 1;
                            }

                            std::string package_share_directory = ament_index_cpp::get_package_share_directory("pgsql_services");
                            std::string aruco_to_first_slot_csv = package_share_directory + "/data/" + tray_type + "_" + request->type + "_aruco_to_first_slot.csv";
                            std::string first_slot_to_every_slot_csv = package_share_directory + "/data/" + tray_type + "_" + request->type + "_first_slot_to_every_slot.csv";

                            


                            geometry_msgs::msg::TransformStamped aruco_to_first_slot_transform = read_transform_from_csv(aruco_to_first_slot_csv);
                            aruco_to_first_slot_transform.header.frame_id = "panda_link0"; // replace with your frame ID
                            aruco_to_first_slot_transform.header.stamp = this->get_clock()->now(); // set the timestamp to the current time

                            geometry_msgs::msg::TransformStamped first_slot_to_current_slot_transform = read_transform_from_csv(first_slot_to_every_slot_csv, slot_id_map[slot_id]);
                            first_slot_to_current_slot_transform.header.frame_id = "panda_link0"; // replace with your frame ID
                            first_slot_to_current_slot_transform.header.stamp = this->get_clock()->now(); // set the timestamp to the current time

                            std::cout << "slot_id: " << slot_id << std::endl;

                            for(const auto &pair : slot_id_map) {
                                std::cout << "Key: " << pair.first << " Value: " << pair.second << std::endl;
                            }

                            std::string workstation_query = "SELECT name, lookout_pose FROM workstation WHERE workstation_id = " + W.quote(workstation_id) + ";";


                            pqxx::result workstation_result = W.exec(workstation_query);

                            for (auto workstation_row : workstation_result) {
                                std::string name = workstation_row["name"].as<std::string>();
                                std::string lookout_pose_json = workstation_row["lookout_pose"].as<std::string>();

                                nlohmann::json j = nlohmann::json::parse(lookout_pose_json);
                                geometry_msgs::msg::Pose lookout_pose = pose_json_converter::JsonToPose(j);
                        
                                std_msgs::msg::Header header;
                                header.frame_id = "panda_link0";
                                header.stamp = this->get_clock()->now(); // Set the timestamp to the current time

                                geometry_msgs::msg::PoseStamped lookout_pose_stamped;
                                lookout_pose_stamped.header = header;
                                lookout_pose_stamped.pose = lookout_pose;

                                response->workstation_name = name;
                                response->lookout_pose = lookout_pose_stamped;
                                response->aruco_id = aruco_id; // You need to get this value from somewhere
                                response->aruco_to_slot_transform = aruco_to_first_slot_transform;
                                response->slot_to_slot_transform = first_slot_to_current_slot_transform;
                                response->empty = empty_bottle;
                                response->success = true;
                                response->message = "Chemical found";

                                // std::cout << response << std::endl;
                          }
                    }
                }
            }
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = std::string("Caught exception: ") + e.what();
    } catch (...) {
        response->success = false;
        response->message = "Caught unknown exception";
    }
    }


    void place_vessel(const std::shared_ptr<pgsql_interfaces::srv::PlaceVessel::Request> request,
                    std::shared_ptr<pgsql_interfaces::srv::PlaceVessel::Response> response) {
        try {
            // Connect to the database
            pqxx::connection C("dbname=" + db_name + " user=" + db_user + " password=" + db_password + " hostaddr=" + db_host + " port=" + db_port);
            if (C.is_open()) {
                // Create a transactional object
                pqxx::work W(C);

                // Lookup vessel id from name in 'vessel'
                pqxx::result R = W.exec("SELECT vessel_id FROM vessel WHERE name = '" + request->name + "'");
                if (R.empty()) {
                    response->success = false;
                    response->message = "Vessel not found";
                    return;
                }
                int vessel_id = R[0][0].as<int>();

                // Find the amount of tray ids in 'tray_slot' for a given 'tray_id' (sizeof) also check if there are any free slots
                R = W.exec("SELECT COUNT(*), COUNT(*) - (SELECT COUNT(*) FROM vessel_placements INNER JOIN tray_slot ON vessel_placements.slot_id = tray_slot.slot_id WHERE tray_slot.tray_id = " + std::to_string(request->tray_id) + ") FROM tray_slot WHERE tray_id = " + std::to_string(request->tray_id));
                int total_slots = R[0][0].as<int>(); //! Unused var
                int free_slots = R[0][1].as<int>();

                if (free_slots == 0) {
                    response->success = false;
                    response->message = "No free slots";
                    return;
                }

                // Map the slot_ids in the range 1, 2, 3 ... sizeof(tray_id) to the tray_id
                R = W.exec("SELECT slot_id FROM tray_slot WHERE tray_id = " + std::to_string(request->tray_id) + " ORDER BY slot_id");
                std::map<int, int> slot_id_map;
                for (size_t i = 0; i < R.size(); ++i) {
                    slot_id_map[R[i][0].as<int>()] = i + 1;
                }

                // Pick the first free slot and insert a new row in 'vessel_placements' with the (original) vessel_id and slot_id
                R = W.exec("SELECT slot_id FROM tray_slot WHERE tray_id = " + std::to_string(request->tray_id) + " AND slot_id NOT IN (SELECT slot_id FROM vessel_placements) LIMIT 1");
                int slot_id = R[0][0].as<int>();
                W.exec("INSERT INTO vessel_placements (vessel_id, slot_id) VALUES (" + std::to_string(vessel_id) + ", " + std::to_string(slot_id) + ")");

                // Find the aruco_id of the tray_id in 'tray'
                R = W.exec("SELECT aruco_id, type, workstation_id FROM tray WHERE tray_id = " + std::to_string(request->tray_id) + " AND type = 'vessel'");
                if (R.empty()) {
                    response->success = false;
                    response->message = "Tray type is not vessel";
                    return;
                }
                int aruco_id = R[0][0].as<int>();
                std::string tray_type = R[0][1].as<std::string>();
                int workstation_id = R[0][2].as<int>();

                R = W.exec("SELECT name, lookout_pose FROM workstation WHERE workstation_id = " + std::to_string(workstation_id));
                std::string workstation_name = R[0][0].as<std::string>();
                std::string lookout_pose_json = R[0][1].as<std::string>();

                nlohmann::json j = nlohmann::json::parse(lookout_pose_json);
                geometry_msgs::msg::Pose lookout_pose = pose_json_converter::JsonToPose(j);

                std_msgs::msg::Header header;
                header.frame_id = "panda_link0";
                header.stamp = this->get_clock()->now(); // Set the timestamp to the current time

                geometry_msgs::msg::PoseStamped lookout_pose_stamped;
                lookout_pose_stamped.header = header;
                lookout_pose_stamped.pose = lookout_pose;

                std::string package_share_directory = ament_index_cpp::get_package_share_directory("pgsql_services");
                std::string aruco_to_first_slot_csv = package_share_directory + "/data/" + tray_type + "_aruco_to_first_slot.csv";
                std::string first_slot_to_every_slot_csv = package_share_directory + "/data/" + tray_type + "_first_slot_to_every_slot.csv";

                geometry_msgs::msg::TransformStamped aruco_to_first_slot_transform = read_transform_from_csv(aruco_to_first_slot_csv);
                aruco_to_first_slot_transform.header.frame_id = "panda_link0";
                aruco_to_first_slot_transform.header.stamp = this->get_clock()->now();

                geometry_msgs::msg::TransformStamped first_slot_to_current_slot_transform = read_transform_from_csv(first_slot_to_every_slot_csv, slot_id_map[slot_id]);
                first_slot_to_current_slot_transform.header.frame_id = "panda_link0"; 
                first_slot_to_current_slot_transform.header.stamp = this->get_clock()->now(); 

                // Commit the transaction
                W.commit();

                // Assuming you have a response object of the appropriate type
                response->workstation_name = workstation_name;
                response->lookout_pose = lookout_pose_stamped;
                response->aruco_id = aruco_id;
                response->aruco_to_slot_transform = aruco_to_first_slot_transform;
                response->slot_to_slot_transform = first_slot_to_current_slot_transform;

                // Set success to true if the operation was successful, false otherwise
                response->success = true; // or false

                // Set the message to provide additional information about the operation
                response->message = "Operation completed successfully"; // or an error message
            } else {
                std::cout << "Can't open database" << std::endl;
                response->success = false;
            }
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
            response->success = false;
        }
    }

    void place_chemical(const std::shared_ptr<pgsql_interfaces::srv::PlaceChemical::Request> request,
                    std::shared_ptr<pgsql_interfaces::srv::PlaceChemical::Response> response) {
        try {
            // Connect to the database
            pqxx::connection C("dbname=" + db_name + " user=" + db_user + " password=" + db_password + " hostaddr=" + db_host + " port=" + db_port);
            if (C.is_open()) {
                // Create a transactional object
                pqxx::work W(C);

                // Lookup chemical id from name in 'chemical'
                pqxx::result R = W.exec("SELECT chemical_id FROM chemical WHERE name = '" + request->name + "'");
                if (R.empty()) {
                    response->success = false;
                    response->message = "chemical not found";
                    return;
                }
                int chemical_id = R[0][0].as<int>();

                // Find the amount of tray ids in 'tray_slot' for a given 'tray_id' (sizeof) also check if there are any free slots
                R = W.exec("SELECT COUNT(*), COUNT(*) - (SELECT COUNT(*) FROM chemical_placements INNER JOIN tray_slot ON chemical_placements.slot_id = tray_slot.slot_id WHERE tray_slot.tray_id = " + std::to_string(request->tray_id) + ") FROM tray_slot WHERE tray_id = " + std::to_string(request->tray_id));
                int total_slots = R[0][0].as<int>(); //! Unused var
                int free_slots = R[0][1].as<int>();

                if (free_slots == 0) {
                    response->success = false;
                    response->message = "No free slots";
                    return;
                }

                // Map the slot_ids in the range 1, 2, 3 ... sizeof(tray_id) to the tray_id
                R = W.exec("SELECT slot_id FROM tray_slot WHERE tray_id = " + std::to_string(request->tray_id) + " ORDER BY slot_id");
                std::map<int, int> slot_id_map;
                for (size_t i = 0; i < R.size(); ++i) {
                    slot_id_map[R[i][0].as<int>()] = i + 1;
                }

                // Pick the first free slot and insert a new row in 'chemical_placements' with the (original) chemical_id and slot_id
                R = W.exec("SELECT slot_id FROM tray_slot WHERE tray_id = " + std::to_string(request->tray_id) + " AND slot_id NOT IN (SELECT slot_id FROM chemical_placements) LIMIT 1");
                int slot_id = R[0][0].as<int>();
                W.exec("INSERT INTO chemical_placements (chemical_id, slot_id) VALUES (" + std::to_string(chemical_id) + ", " + std::to_string(slot_id) + ")");

                // Find the aruco_id of the tray_id in 'tray'
                R = W.exec("SELECT aruco_id, type, workstation_id FROM tray WHERE tray_id = " + std::to_string(request->tray_id) + " AND type = 'chemical'");
                if (R.empty()) {
                    response->success = false;
                    response->message = "Tray type is not chemical";
                    return;
                }
                int aruco_id = R[0][0].as<int>();
                std::string tray_type = R[0][1].as<std::string>();
                int workstation_id = R[0][2].as<int>();

                R = W.exec("SELECT name, lookout_pose FROM workstation WHERE workstation_id = " + std::to_string(workstation_id));
                std::string workstation_name = R[0][0].as<std::string>();
                std::string lookout_pose_json = R[0][1].as<std::string>();

                nlohmann::json j = nlohmann::json::parse(lookout_pose_json);
                geometry_msgs::msg::Pose lookout_pose = pose_json_converter::JsonToPose(j);

                std_msgs::msg::Header header;
                header.frame_id = "panda_link0";
                header.stamp = this->get_clock()->now(); // Set the timestamp to the current time

                geometry_msgs::msg::PoseStamped lookout_pose_stamped;
                lookout_pose_stamped.header = header;
                lookout_pose_stamped.pose = lookout_pose;

                std::string package_share_directory = ament_index_cpp::get_package_share_directory("pgsql_services");
                std::string aruco_to_first_slot_csv = package_share_directory + "/data/" + tray_type + "_aruco_to_first_slot.csv";
                std::string first_slot_to_every_slot_csv = package_share_directory + "/data/" + tray_type + "_first_slot_to_every_slot.csv";

                geometry_msgs::msg::TransformStamped aruco_to_first_slot_transform = read_transform_from_csv(aruco_to_first_slot_csv);
                aruco_to_first_slot_transform.header.frame_id = "panda_link0";
                aruco_to_first_slot_transform.header.stamp = this->get_clock()->now();

                geometry_msgs::msg::TransformStamped first_slot_to_current_slot_transform = read_transform_from_csv(first_slot_to_every_slot_csv, slot_id_map[slot_id]);
                first_slot_to_current_slot_transform.header.frame_id = "panda_link0"; 
                first_slot_to_current_slot_transform.header.stamp = this->get_clock()->now(); 

                // Commit the transaction
                W.commit();

                // Assuming you have a response object of the appropriate type
                response->workstation_name = workstation_name;
                response->lookout_pose = lookout_pose_stamped;
                response->aruco_id = aruco_id;
                response->aruco_to_slot_transform = aruco_to_first_slot_transform;
                response->slot_to_slot_transform = first_slot_to_current_slot_transform;

                // Set success to true if the operation was successful, false otherwise
                response->success = true; // or false

                // Set the message to provide additional information about the operation
                response->message = "Operation completed successfully"; // or an error message
            } else {
                std::cout << "Can't open database" << std::endl;
                response->success = false;
            }
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
            response->success = false;
        }
    }

    void remove_chemical_placement(const std::shared_ptr<pgsql_interfaces::srv::RemoveChemicalPlacement::Request> request,
                    std::shared_ptr<pgsql_interfaces::srv::RemoveChemicalPlacement::Response> response) {
        try {
            // Connect to the database
            pqxx::connection C("dbname=" + db_name + " user=" + db_user + " password=" + db_password + " hostaddr=" + db_host + " port=" + db_port);
            if (C.is_open()) {
                // Create a transactional object
                pqxx::work W(C);

                // Lookup chemical id from name in 'chemical'
                pqxx::result R = W.exec("SELECT chemical_id FROM chemical WHERE name = '" + request->name + "'");
                if (R.empty()) {
                    response->success = false;
                    response->message = "chemical not found";
                    return;
                }
                int chemical_id = R[0][0].as<int>();

                pqxx::result R2 = W.exec("DELETE FROM chemical_placements WHERE slot_id = " + std::to_string(request->slot_id) + " AND chemical_id = " + std::to_string(chemical_id));                

                // Check if the DELETE operation affected any rows
                if (R2.affected_rows() == 0) {
                    response->success = false;
                    response->message = "No chemical found in the specified slot";
                    return;
                }            

                // Commit the transaction
                W.commit();

                // Set success to true if the operation was successful, false otherwise
                response->success = true; // or false

                // Set the message to provide additional information about the operation
                response->message = "Operation completed successfully"; // or an error message
            } else {
                std::cout << "Could not access database" << std::endl;
                response->success = false;
            }
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
            response->success = false;
        }
    }

    void remove_vessel_placement(const std::shared_ptr<pgsql_interfaces::srv::RemoveVesselPlacement::Request> request,
                    std::shared_ptr<pgsql_interfaces::srv::RemoveVesselPlacement::Response> response) {
        try {
            // Connect to the database
            pqxx::connection C("dbname=" + db_name + " user=" + db_user + " password=" + db_password + " hostaddr=" + db_host + " port=" + db_port);
            if (C.is_open()) {
                // Create a transactional object
                pqxx::work W(C);

                // Lookup chemical id from name in 'chemical'
                pqxx::result R = W.exec("SELECT vessel_id FROM vessel WHERE name = '" + request->name + "'");
                if (R.empty()) {
                    response->success = false;
                    response->message = "Vessel not found";
                    return;
                }
                int vessel_id = R[0][0].as<int>();

                pqxx::result R2 = W.exec("DELETE FROM vessel_placements WHERE slot_id = " + std::to_string(request->slot_id) + " AND vessel_id = " + std::to_string(vessel_id));                

                // Check if the DELETE operation affected any rows
                if (R2.affected_rows() == 0) {
                    response->success = false;
                    response->message = "No vessel found in the specified slot";
                    return;
                }            

                // Commit the transaction
                W.commit();

                // Set success to true if the operation was successful, false otherwise
                response->success = true; // or false

                // Set the message to provide additional information about the operation
                response->message = "Operation completed successfully"; // or an error message
            } else {
                std::cout << "Could not access database" << std::endl;
                response->success = false;
            }
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
            response->success = false;
        }
    }

    geometry_msgs::msg::TransformStamped read_transform_from_csv(const std::string& csv_file, int slot_id = -1) {
        // Open the CSV file
        io::CSVReader<9> in(csv_file);
        in.read_header(io::ignore_extra_column, "frame_id", "child_frame_id", "translation_x", "translation_y", "translation_z", "rotation_x", "rotation_y", "rotation_z", "rotation_w");
        std::string frame_id, child_frame_id;
        double translation_x, translation_y, translation_z, rotation_x, rotation_y, rotation_z, rotation_w;
        while(in.read_row(frame_id, child_frame_id, translation_x, translation_y, translation_z, rotation_x, rotation_y, rotation_z, rotation_w)){
            // If a slot_id is provided, skip rows until the slot_id matches the child_frame_id
            if (slot_id != -1 && child_frame_id != "slot_" + std::to_string(slot_id)) {
                continue;
            }

            // Create a TransformStamped
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.frame_id = frame_id;
            transform_stamped.child_frame_id = child_frame_id;
            transform_stamped.transform.translation.x = translation_x;
            transform_stamped.transform.translation.y = translation_y;
            transform_stamped.transform.translation.z = translation_z;
            transform_stamped.transform.rotation.x = rotation_x;
            transform_stamped.transform.rotation.y = rotation_y;
            transform_stamped.transform.rotation.z = rotation_z;
            transform_stamped.transform.rotation.w = rotation_w;

            return transform_stamped;
        }

        // If no matching transform is found, return an empty TransformStamped
        return geometry_msgs::msg::TransformStamped();
    }

    rclcpp::Service<pgsql_interfaces::srv::GetVessel>::SharedPtr get_vessel_service_;
    rclcpp::Service<pgsql_interfaces::srv::GetChemical>::SharedPtr get_chemical_service_;
    rclcpp::Service<pgsql_interfaces::srv::PlaceVessel>::SharedPtr place_vessel_service_;
    rclcpp::Service<pgsql_interfaces::srv::PlaceChemical>::SharedPtr place_chemical_service_;
    rclcpp::Service<pgsql_interfaces::srv::RemoveChemicalPlacement>::SharedPtr remove_chemical_placement_service_;
    rclcpp::Service<pgsql_interfaces::srv::RemoveVesselPlacement>::SharedPtr remove_vessel_placement_service_;

    // rclcpp::Service<pgsql_interfaces::srv::RemoveChemical>::SharedPtr remove_vessel_service_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DatabaseService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
