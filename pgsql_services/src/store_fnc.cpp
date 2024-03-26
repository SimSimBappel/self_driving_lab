
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
    void get_chemical(const std::shared_ptr<pgsql_interfaces::srv::Chemical::Request> request,
                    std::shared_ptr<pgsql_interfaces::srv::GetChemical::Response> response) {
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
                                response->message = "Chemical found";
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

