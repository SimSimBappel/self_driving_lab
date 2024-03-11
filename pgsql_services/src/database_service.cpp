// ROS2
#include <rclcpp/rclcpp.hpp>

// MSG
#include "pgsql_interfaces/msg/chemical_location.hpp"
#include "pgsql_interfaces/msg/workstation_location.hpp"

// SRV
#include "pgsql_interfaces/srv/add_chemical.hpp"
#include "pgsql_interfaces/srv/add_workstation.hpp"
#include "pgsql_interfaces/srv/upsert_chemical_location.hpp"
#include "pgsql_interfaces/srv/upsert_workstation_location.hpp"
#include "pgsql_interfaces/srv/get_all_chemical_locations.hpp"
#include "pgsql_interfaces/srv/get_all_workstation_locations.hpp"
#include "pgsql_interfaces/srv/remove_chemical.hpp"
#include "pgsql_interfaces/srv/remove_workstation.hpp"

// LIBS
#include "pgsql_services/type_converter.h"
#include <nlohmann/json.hpp>
#include <pqxx/pqxx>

class DatabaseService : public rclcpp::Node {
public:
    DatabaseService() : Node("database_service") {
        this->db_name = this->declare_parameter<std::string>("db_name", "postgres");
        this->db_user = this->declare_parameter<std::string>("db_user", "postgres");
        this->db_password = this->declare_parameter<std::string>("db_password", "your-super-secret-and-long-postgres-password");
        this->db_host = this->declare_parameter<std::string>("db_host", "127.0.0.1");
        this->db_port = this->declare_parameter<std::string>("db_port", "5432");


        add_chemical_service_ = this->create_service<pgsql_interfaces::srv::AddChemical>(
            "add_chemical",
            std::bind(&DatabaseService::add_chemical, this, std::placeholders::_1, std::placeholders::_2));

        add_workstation_service_ = this->create_service<pgsql_interfaces::srv::AddWorkstation>(
            "add_workstation",
            std::bind(&DatabaseService::add_workstation, this, std::placeholders::_1, std::placeholders::_2));

        upsert_chemical_location_service_ = this->create_service<pgsql_interfaces::srv::UpsertChemicalLocation>(
            "upsert_chemical_location",
            std::bind(&DatabaseService::upsert_chemical_location, this, std::placeholders::_1, std::placeholders::_2));

        upsert_workstation_location_service_ = this->create_service<pgsql_interfaces::srv::UpsertWorkstationLocation>(
            "upsert_workstation_location",
            std::bind(&DatabaseService::upsert_workstation_location, this, std::placeholders::_1, std::placeholders::_2));

        get_all_chemical_locations_service_ = this->create_service<pgsql_interfaces::srv::GetAllChemicalLocations>(
            "get_all_chemical_locations",
            std::bind(&DatabaseService::get_all_chemical_locations, this, std::placeholders::_1, std::placeholders::_2));

        get_all_workstation_locations_service_ = this->create_service<pgsql_interfaces::srv::GetAllWorkstationLocations>(
            "get_all_workstation_locations",
            std::bind(&DatabaseService::get_all_workstation_locations, this, std::placeholders::_1, std::placeholders::_2));

        remove_chemical_service_ = this->create_service<pgsql_interfaces::srv::RemoveChemical>(
            "remove_chemical",
            std::bind(&DatabaseService::remove_chemical, this, std::placeholders::_1, std::placeholders::_2));

        remove_workstation_service_ = this->create_service<pgsql_interfaces::srv::RemoveWorkstation>(
            "remove_workstation",
            std::bind(&DatabaseService::remove_workstation, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    std::string db_name;
    std::string db_user;
    std::string db_password;
    std::string db_host;
    std::string db_port;

    /**
     * @brief Adds a new chemical to the database.
     *
     * This function inserts a new chemical record into the database with the given details.
     * It expects the chemical's name, formula, and safety level to be provided.
     *
     * @param request Shared pointer to the AddChemical Request, containing:
     *                - name: The name of the chemical.
     *                - formula: The chemical formula.
     *                - safety_level: The safety level of the chemical.
     * @param response Shared pointer to the AddChemical Response, containing:
     *                 - success: Boolean indicating the success or failure of the operation.
     *                 - message: Detailed message about the outcome of the operation.
     *                 - id: The ID of the newly added chemical (on success).
     * 
     * @throws std::runtime_error If the database operation fails.
     */
    void add_chemical(const std::shared_ptr<pgsql_interfaces::srv::AddChemical::Request> request,
                                    std::shared_ptr<pgsql_interfaces::srv::AddChemical::Response> response) {
        try {
            // Create a connection to the database
            pqxx::connection C("dbname=" + db_name + " user=" + db_user +
                            " password=" + db_password + " hostaddr=" + db_host +
                            " port=" + db_port);
            pqxx::work W(C);  // Start a transaction

            // Prepare the SQL statement to insert the new chemical record
            std::string query = "INSERT INTO chemical (name, formula, safety_level) VALUES (" +
                                W.quote(request->name) + ", " +
                                W.quote(request->formula) + ", " +
                                W.quote(std::to_string(request->safety_level)) + ") RETURNING id;";

            // Execute the query and commit the transaction
            pqxx::row new_chemical_row = W.exec1(query);
            W.commit();

            // Retrieve the chemical_id of the new record
            int new_chemical_id = new_chemical_row["id"].as<int>();

            // Set the response success status and message
            response->success = true;
            response->message = "Chemical added successfully with ID: " + std::to_string(new_chemical_id);
            response->id = new_chemical_id;  // Include the new chemical_id in the response
        } catch (const std::exception& e) {
            // If an error occurs, log it and set the response to indicate failure
            RCLCPP_ERROR(this->get_logger(), "Database operation failed: %s", e.what());
            response->success = false;
            response->message = "Database operation failed: " + std::string(e.what());
        }
    }

    /**
     * @brief Adds a new workstation to the database.
     *
     * This function inserts a new workstation record into the database. It expects the workstation's
     * name and type to be provided in the request.
     *
     * @param request Shared pointer to the AddWorkstation Request, containing:
     *                - name: The name of the workstation.
     *                - type: The type of the workstation.
     * @param response Shared pointer to the AddWorkstation Response, containing:
     *                 - success: Boolean indicating the success or failure of the operation.
     *                 - message: Detailed message about the outcome of the operation.
     *                 - id: The ID of the newly added workstation (on success).
     * 
     * @throws std::runtime_error If the database operation fails.
     */
    void add_workstation(const std::shared_ptr<pgsql_interfaces::srv::AddWorkstation::Request> request,
                        std::shared_ptr<pgsql_interfaces::srv::AddWorkstation::Response> response) {
        try {
        // Create a connection to the database
        pqxx::connection C("dbname=" + db_name + " user=" + db_user +
                        " password=" + db_password + " hostaddr=" + db_host +
                        " port=" + db_port);
        pqxx::work W(C);  // Start a transaction

        // Prepare the SQL statement to insert the new chemical record
        std::string query = "INSERT INTO workstation (name, type) VALUES (" +
                            W.quote(request->name) + ", " +
                            W.quote(request->type) + ") RETURNING id;";
                            
            // Execute the query and commit the transaction
            pqxx::row new_workstation_row = W.exec1(query);
            W.commit();

            // Retrieve the chemical_id of the new record
            int new_workstation_id = new_workstation_row["id"].as<int>();

            // Set the response success status and messageAddWor
            response->success = true;
            response->message = "Chemical added successfully with ID: " + std::to_string(new_workstation_id);
            response->id = new_workstation_id;  // Include the new chemical_id in the response
        } catch (const std::exception& e) {
            // If an error occurs, log it and set the response to indicate failure
            RCLCPP_ERROR(this->get_logger(), "Database operation failed: %s", e.what());
            response->success = false;
            response->message = "Database operation failed: " + std::string(e.what());
        }
    }


    /**
     * @brief Upserts the location of a chemical in the database.
     *
     * This function handles the insertion or update of a chemical's location in the database.
     * It can work in two modes:
     * 1. If a location_id is provided, it updates the existing location with new coordinates.
     * 2. If no location_id is provided, it inserts a new location record and associates it with the chemical.
     *
     * The chemical to update is determined based on one of the following provided parameters:
     * - name
     * - formula
     * - id
     * 
     * The function ensures that exactly one of these identifiers is provided to accurately locate the chemical.
     *
     * @param request Shared pointer to the UpsertChemicalLocation Request, containing:
     *                - id: The ID of the chemical (if known).
     *                - name: The name of the chemical (alternative to id).
     *                - formula: The chemical formula (alternative to id and name).
     *                - location_base: The base location as a geometry_msgs/Pose.
     *                - location_hand: The hand location as a geometry_msgs/Pose.
     *                - location_id: The ID of the location to update (if updating existing location).
     * @param response Shared pointer to the UpsertChemicalLocation Response, containing:
     *                 - success: Boolean indicating the success or failure of the operation.
     *                 - message: Detailed message about the outcome of the operation.
     * 
     * @throws std::runtime_error If the chemical cannot be identified or if database operations fail.
     * 
     * @note The function commits the transaction if successful and rolls back in case of an exception.
     */
    void upsert_chemical_location(const std::shared_ptr<pgsql_interfaces::srv::UpsertChemicalLocation::Request> request,
                                                std::shared_ptr<pgsql_interfaces::srv::UpsertChemicalLocation::Response> response) {      
        try {
            pqxx::connection C("dbname = " + db_name + " user = " + db_user + 
                                " password = " + db_password + " hostaddr = " + db_host + 
                                " port = " + db_port);
            pqxx::work W(C);

            // Initialize the chemical ID variable
            int chemical_id = -1;


            bool name_provided = !request->name.empty();
            bool formula_provided = !request->formula.empty();
            bool id_provided = request->id != 0;  // Only consider id provided if it's non-zero

            int provided_count = name_provided + formula_provided + id_provided;

            if (provided_count != 1) {
                RCLCPP_ERROR(this->get_logger(), "Request must specify exactly one of name, formula, or id.");
                response->success = false;
                response->message = "Request must specify exactly one of name, formula, or id.";
                return;
            }

            // Check if the location is already associated with a different chemical
            if (request->location_id != 0) {
                pqxx::result existing_association = W.exec("SELECT chemical_id FROM chemical_locations WHERE location_id = " + W.quote(std::to_string(request->location_id)) + ";");
                if (!existing_association.empty() && existing_association[0][0].as<int>() != chemical_id) {
                    // Location already associated with a different chemical
                    response->success = false;
                    response->message = "Location is already associated with a different chemical.";
                    return;
                }
            }

            // Check if the request specifies a chemical by name
            if (!request->name.empty()) {
                // Execute query and check for results
                pqxx::result res = W.exec("SELECT id FROM chemical WHERE name = " + W.quote(request->name));
                if (res.empty()) {
                    throw std::runtime_error("No rows returned for the given chemical name.");
                }
                chemical_id = res[0][0].as<int>();
            } 
            // Check if the request specifies a chemical by formula
            else if (!request->formula.empty()) {
                // Execute query and check for results
                pqxx::result res = W.exec("SELECT id FROM chemical WHERE formula = " + W.quote(request->formula));
                if (res.empty()) {
                    throw std::runtime_error("No rows returned for the given chemical formula.");
                }
                if (res.size() != 1) {
                    throw std::runtime_error("Ambiguous formula. Multiple chemicals found.");
                }
                chemical_id = res[0][0].as<int>();
            } 
            // Use the provided ID if present and not zero
            else if (request->id != 0) {
                // Directly use the provided ID
                chemical_id = static_cast<int>(request->id);
            }

            // Check if a valid chemical ID was obtained
            if (chemical_id == -1) {
                response->success = false;
                response->message = "No rows returned for the given chemical, please check the given input.";
                return;
            }
            int location_id = request->location_id;
            // Serialize the location_base and location_hand as JSON strings
            auto location_base_json = pose_json_converter::PoseToJson(request->location_base);
            auto location_hand_json = pose_json_converter::PoseToJson(request->location_hand);

            if (location_id == 0) {
                // Insert the new location record
                std::string location_query = 
                    "INSERT INTO location (location_base, location_hand) VALUES (" +
                    W.quote(location_base_json.dump()) + ", " +
                    W.quote(location_hand_json.dump()) + ") RETURNING id;";
                pqxx::row location_row = W.exec1(location_query);
                location_id = location_row[0].as<int>();
            } else {
                // Update the existing location record
                std::string location_update_query = 
                    "UPDATE location SET location_base = " + W.quote(location_base_json.dump()) + 
                    ", location_hand = " + W.quote(location_hand_json.dump()) + 
                    " WHERE id = " + W.quote(std::to_string(location_id)) + ";";
                W.exec(location_update_query);
            }

            // Associate or update the location with the chemical
            std::string chemical_location_query = 
                "INSERT INTO chemical_locations (chemical_id, location_id) VALUES (" +
                W.quote(std::to_string(chemical_id)) + ", " +
                W.quote(std::to_string(location_id)) + ") " +
                "ON CONFLICT (location_id) DO UPDATE SET chemical_id = EXCLUDED.chemical_id;";
            W.exec(chemical_location_query);
            W.commit();

            response->success = true;
            response->message = "Chemical location upserted successfully.";
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Database operation failed: %s", e.what());
            response->success = false;
            response->message = "Database operation failed: " + std::string(e.what());
        }
    }

    /**
     * @brief Upserts the location of a workstation in the database.
     *
     * This function handles the insertion or update of a workstation's location in the database.
     * It can work in two modes:
     * 1. If a location_id is provided, it updates the existing location with new coordinates.
     * 2. If no location_id is provided, it inserts a new location record and associates it with the workstation.
     *
     * The workstation to update is determined based on one of the following provided parameters:
     * - name
     * - id
     * 
     * The function ensures that exactly one of these identifiers is provided to accurately locate the workstation.
     *
     * @param request Shared pointer to the UpsertWorkstationLocation Request, containing:
     *                - id: The ID of the workstation (if known).
     *                - name: The name of the workstation (alternative to id).
     *                - location_base: The base location as a geometry_msgs/Pose.
     *                - location_hand: The hand location as a geometry_msgs/Pose.
     *                - location_id: The ID of the location to update (if updating existing location).
     * @param response Shared pointer to the UpsertWorkstationLocation Response, containing:
     *                 - success: Boolean indicating the success or failure of the operation.
     *                 - message: Detailed message about the outcome of the operation.
     * 
     * @throws std::runtime_error If the workstation cannot be identified or if database operations fail.
     * 
     * @note The function commits the transaction if successful and rolls back in case of an exception.
     */
    void upsert_workstation_location(const std::shared_ptr<pgsql_interfaces::srv::UpsertWorkstationLocation::Request> request,
                                                std::shared_ptr<pgsql_interfaces::srv::UpsertWorkstationLocation::Response> response) {      
        try {
            pqxx::connection C("dbname = " + db_name + " user = " + db_user + 
                                " password = " + db_password + " hostaddr = " + db_host + 
                                " port = " + db_port);
            pqxx::work W(C);

            // Initialize the workstation ID variable
            int workstation_id = -1;


            bool name_provided = !request->name.empty();
            bool id_provided = request->id != 0;  // Only consider id provided if it's non-zero

            int provided_count = name_provided + id_provided;

            if (provided_count != 1) {
                RCLCPP_ERROR(this->get_logger(), "Request must specify exactly one of name, or id.");
                response->success = false;
                response->message = "Request must specify exactly one of name, or id.";
                return;
            }

            // Check if the location is already associated with a different workstation
            if (request->location_id != 0) {
                pqxx::result existing_association = W.exec("SELECT workstation_id FROM workstation_locations WHERE location_id = " + W.quote(std::to_string(request->location_id)) + ";");
                if (!existing_association.empty() && existing_association[0][0].as<int>() != workstation_id) {
                    // Location already associated with a different workstation
                    response->success = false;
                    response->message = "Location is already associated with a different workstation.";
                    return;
                }
            }

            // Check if the request specifies a workstation by formula
            if (!request->name.empty()) {
                // Execute query and check for results
                pqxx::result res = W.exec("SELECT id FROM workstation WHERE name = " + W.quote(request->name));
                if (res.empty()) {
                    throw std::runtime_error("No rows returned for the given workstation name.");
                }
                if (res.size() != 1) {
                    throw std::runtime_error("Ambiguous name. Multiple workstations found.");
                }
                workstation_id = res[0][0].as<int>();
            } 
            // Use the provided ID if present and not zero
            else if (request->id != 0) {
                // Directly use the provided ID
                workstation_id = static_cast<int>(request->id);
            }

            // Check if a valid workstation ID was obtained
            if (workstation_id == -1) {
                response->success = false;
                response->message = "No rows returned for the given workstation, please check the given input.";
                return;
            }
            int location_id = request->location_id;
            // Serialize the location_base and location_hand as JSON strings
            auto location_base_json = pose_json_converter::PoseToJson(request->location_base);
            auto location_hand_json = pose_json_converter::PoseToJson(request->location_hand);

            if (location_id == 0) {
                // Insert the new location record
                std::string location_query = 
                    "INSERT INTO location (location_base, location_hand) VALUES (" +
                    W.quote(location_base_json.dump()) + ", " +
                    W.quote(location_hand_json.dump()) + ") RETURNING id;";
                pqxx::row location_row = W.exec1(location_query);
                location_id = location_row[0].as<int>();
            } else {
                // Update the existing location record
                std::string location_update_query = 
                    "UPDATE location SET location_base = " + W.quote(location_base_json.dump()) + 
                    ", location_hand = " + W.quote(location_hand_json.dump()) + 
                    " WHERE id = " + W.quote(std::to_string(location_id)) + ";";
                W.exec(location_update_query);
            }

            // Associate or update the location with the workstation
            std::string workstation_location_query = 
                "INSERT INTO workstation_locations (workstation_id, location_id) VALUES (" +
                W.quote(std::to_string(workstation_id)) + ", " +
                W.quote(std::to_string(location_id)) + ") " +
                "ON CONFLICT (location_id) DO UPDATE SET workstation_id = EXCLUDED.workstation_id;";
            W.exec(workstation_location_query);
            W.commit();

            response->success = true;
            response->message = "workstation location upserted successfully.";
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Database operation failed: %s", e.what());
            response->success = false;
            response->message = "Database operation failed: " + std::string(e.what());
        }
    }

    /**
     * @brief Removes a chemical from the database.
     *
     * This function handles the removal of a chemical from the database. It can remove the chemical entirely or remove its association with a specific location if a location ID is provided.
     * 
     * @param request Shared pointer to the RemoveChemical Request, containing:
     *                - chemical_id: The ID of the chemical to remove.
     *                - location_id: (Optional) The ID of the location from which to remove the chemical. If provided, only the association with this location will be removed.
     * @param response Shared pointer to the RemoveChemical Response, containing:
     *                 - success: Boolean indicating the success or failure of the operation.
     *                 - message: Detailed message about the outcome of the operation.
     * 
     * @throws std::runtime_error If database operations fail.
     */
    void remove_chemical(const std::shared_ptr<pgsql_interfaces::srv::RemoveChemical::Request> request,
                                        std::shared_ptr<pgsql_interfaces::srv::RemoveChemical::Response> response) {
        try {
            // Establish a connection to the database
            pqxx::connection C("dbname = " + db_name + " user = " + db_user +
                                " password = " + db_password + " hostaddr = " + db_host +
                                " port = " + db_port);
            pqxx::work W(C);

            if (request->location_id != 0) {
                // If location_id is provided, delete records from the ChemicalLocations table for the specified location
                std::string chemicalLocationsDeleteQuery = "DELETE FROM chemical_locations WHERE chemical_id = " + W.quote(std::to_string(request->chemical_id)) +
                                                            " AND location_id = " + W.quote(std::to_string(request->location_id)) + ";";
                W.exec(chemicalLocationsDeleteQuery);
            } else {
                // If location_id is not provided, delete the chemical record from the Chemical table
                std::string chemicalDeleteQuery = "DELETE FROM chemical WHERE id = " + W.quote(std::to_string(request->chemical_id)) + ";";
                W.exec(chemicalDeleteQuery);

                // Also delete associated records in the ChemicalLocations table
                std::string chemicalLocationsDeleteQuery = "DELETE FROM chemical_locations WHERE chemical_id = " + W.quote(std::to_string(request->chemical_id)) + ";";
                W.exec(chemicalLocationsDeleteQuery);
            }

            // Commit the transaction
            W.commit();

            // Set the response success status and message
            response->success = true;
            response->message = "Chemical removed successfully.";
        } catch (const std::exception &e) {
            // Handle any errors
            RCLCPP_ERROR(this->get_logger(), "Database query failed: %s", e.what());
            response->success = false;
            response->message = "Failed to remove chemical: " + std::string(e.what());
        }
    }

    /**
     * @brief Removes a workstation from the database.
     *
     * This function handles the removal of a workstation from the database. It can remove the workstation entirely or remove its association with a specific location if a location ID is provided.
     * 
     * @param request Shared pointer to the RemoveWorkstation Request, containing:
     *                - workstation_id: The ID of the workstation to remove.
     *                - location_id: (Optional) The ID of the location from which to remove the workstation. If provided, only the association with this location will be removed.
     * @param response Shared pointer to the RemoveWorkstation Response, containing:
     *                 - success: Boolean indicating the success or failure of the operation.
     *                 - message: Detailed message about the outcome of the operation.
     * 
     * @throws std::runtime_error If database operations fail.
     */
    void remove_workstation(const std::shared_ptr<pgsql_interfaces::srv::RemoveWorkstation::Request> request,
                                        std::shared_ptr<pgsql_interfaces::srv::RemoveWorkstation::Response> response) {
        try {
            // Establish a connection to the database
            pqxx::connection C("dbname = " + db_name + " user = " + db_user +
                                " password = " + db_password + " hostaddr = " + db_host +
                                " port = " + db_port);
            pqxx::work W(C);

            if (request->location_id != 0) {
                // If location_id is provided, delete records from the workstationLocations table for the specified location
                std::string workstationLocationsDeleteQuery = "DELETE FROM workstation_locations WHERE workstation_id = " + W.quote(std::to_string(request->workstation_id)) +
                                                            " AND location_id = " + W.quote(std::to_string(request->location_id)) + ";";
                W.exec(workstationLocationsDeleteQuery);
            } else {
                // If location_id is not provided, delete the workstation record from the workstation table
                std::string workstationDeleteQuery = "DELETE FROM workstation WHERE id = " + W.quote(std::to_string(request->workstation_id)) + ";";
                W.exec(workstationDeleteQuery);

                // Also delete associated records in the workstationLocations table
                std::string workstationLocationsDeleteQuery = "DELETE FROM workstation_locations WHERE workstation_id = " + W.quote(std::to_string(request->workstation_id)) + ";";
                W.exec(workstationLocationsDeleteQuery);
            }

            // Commit the transaction
            W.commit();

            // Set the response success status and message
            response->success = true;
            response->message = "workstation removed successfully.";
        } catch (const std::exception &e) {
            // Handle any errors
            RCLCPP_ERROR(this->get_logger(), "Database query failed: %s", e.what());
            response->success = false;
            response->message = "Failed to remove workstation: " + std::string(e.what());
        }
    }


    /**
     * @brief Retrieves all locations associated with a specific chemical from the database.
     *
     * This function queries the database for all locations where a particular chemical is present.
     * The chemical is identified by either its id, name, or formula as provided in the request.
     * The response includes details for each location associated with the chemical, such as the chemical's name,
     * chemical ID, location ID, and the pose at the base and hand locations.
     *
     * @param request Shared pointer to the GetAllChemicalLocations Request, containing:
     *                - id: The ID of the chemical (optional).
     *                - name: The name of the chemical (optional).
     *                - formula: The chemical formula (optional).
     * @param response Shared pointer to the GetAllChemicalLocations Response, containing:
     *                 - chemical_id: The ID of the chemical.
     *                 - name: The name of the chemical.
     *                 - location_id: The ID of each location associated with the chemical.
     *                 - location_base: The base location as geometry_msgs/Pose.
     *                 - location_hand: The hand location as geometry_msgs/Pose.
     *                 - success: Boolean indicating the success or failure of the operation.
     *                 - message: Detailed message about the outcome of the operation.
     * 
     * @throws std::runtime_error If the database operation fails or if no matching chemical is found.
     */
    void get_all_chemical_locations(const std::shared_ptr<pgsql_interfaces::srv::GetAllChemicalLocations::Request> request,
                                                    std::shared_ptr<pgsql_interfaces::srv::GetAllChemicalLocations::Response> response) {
        try {
            // Establish a connection to the database
            pqxx::connection C("dbname = " + db_name + " user = " + db_user + 
                                " password = " + db_password + " hostaddr = " + db_host + 
                                " port = " + db_port);
            pqxx::work W(C);

            // Prepare the SQL query to get all locations with details for the specified chemical ID
            std::string query = "SELECT C.name, C.id as chemical_id, L.id as location_id, L.location_base, L.location_hand "
                                "FROM chemical C "
                                "INNER JOIN chemical_locations CL ON C.id = CL.chemical_id "
                                "INNER JOIN location L ON CL.location_id = L.id "
                                "WHERE C.id = " + W.quote(std::to_string(request->id)) + ";";

            // Execute the query
            pqxx::result R = W.exec(query);

            // Check if the result is empty (no chemicals found)
            if (R.empty()) {
                response->success = false;
                response->message = "No locations found for the given chemical ID.";
                return;
            }

            // Fill the response with the retrieved data
            for (const auto &row : R) {
                // Create a ChemicalLocation object and fill it with data
                pgsql_interfaces::msg::ChemicalLocation chemicalLocation;
                chemicalLocation.name = row["name"].as<std::string>();
                chemicalLocation.chemical_id = row["chemical_id"].as<int>();
                chemicalLocation.location_id = row["location_id"].as<int>();
                chemicalLocation.location_base = pose_json_converter::JsonToPose(nlohmann::json::parse(row["location_base"].as<std::string>()));
                chemicalLocation.location_hand = pose_json_converter::JsonToPose(nlohmann::json::parse(row["location_hand"].as<std::string>()));

                // Add the object to the response list
                response->locations.push_back(chemicalLocation);
            }

            response->success = true;
            response->message = "Successfully retrieved location details for the chemical.";
        } catch (const std::exception &e) {
            // Handle any errors
            RCLCPP_ERROR(this->get_logger(), "Database query failed: %s", e.what());
            response->success = false;
            response->message = "Failed to retrieve location details: " + std::string(e.what());
        }
    }

    void get_all_workstation_locations(const std::shared_ptr<pgsql_interfaces::srv::GetAllWorkstationLocations::Request> request,
                                                    std::shared_ptr<pgsql_interfaces::srv::GetAllWorkstationLocations::Response> response) {
        try {
            // Establish a connection to the database
            pqxx::connection C("dbname = " + db_name + " user = " + db_user + 
                                " password = " + db_password + " hostaddr = " + db_host + 
                                " port = " + db_port);
            pqxx::work W(C);

            // Prepare the SQL query to get all locations with details for the specified workstation ID
            std::string query = "SELECT C.name, C.id as workstation_id, L.id as location_id, L.location_base, L.location_hand "
                                "FROM workstation C "
                                "INNER JOIN workstation_locations CL ON C.id = CL.workstation_id "
                                "INNER JOIN location L ON CL.location_id = L.id "
                                "WHERE C.id = " + W.quote(std::to_string(request->id)) + ";";

            // Execute the query
            pqxx::result R = W.exec(query);

            // Check if the result is empty (no workstations found)
            if (R.empty()) {
                response->success = false;
                response->message = "No locations found for the given workstation ID.";
                return;
            }

            // Fill the response with the retrieved data
            for (const auto &row : R) {
                // Create a workstationLocation object and fill it with data
                pgsql_interfaces::msg::WorkstationLocation workstationLocation;
                workstationLocation.name = row["name"].as<std::string>();
                workstationLocation.type = row["type"].as<std::string>();
                workstationLocation.workstation_id = row["workstation_id"].as<int>();
                workstationLocation.location_id = row["location_id"].as<int>();
                workstationLocation.location_base = pose_json_converter::JsonToPose(nlohmann::json::parse(row["location_base"].as<std::string>()));
                workstationLocation.location_hand = pose_json_converter::JsonToPose(nlohmann::json::parse(row["location_hand"].as<std::string>()));

                // Add the object to the response list
                response->locations.push_back(workstationLocation);
            }

            response->success = true;
            response->message = "Successfully retrieved location details for the workstation.";
        } catch (const std::exception &e) {
            // Handle any errors
            RCLCPP_ERROR(this->get_logger(), "Database query failed: %s", e.what());
            response->success = false;
            response->message = "Failed to retrieve location details: " + std::string(e.what());
        }
    }

    // TODO: Add aruco support, Add item support (vial holder, pipette, etc.), maybe some frontend, 

    rclcpp::Service<pgsql_interfaces::srv::AddChemical>::SharedPtr add_chemical_service_;
    rclcpp::Service<pgsql_interfaces::srv::AddWorkstation>::SharedPtr add_workstation_service_;
    rclcpp::Service<pgsql_interfaces::srv::UpsertChemicalLocation>::SharedPtr upsert_chemical_location_service_;
    rclcpp::Service<pgsql_interfaces::srv::UpsertWorkstationLocation>::SharedPtr upsert_workstation_location_service_;
    rclcpp::Service<pgsql_interfaces::srv::GetAllChemicalLocations>::SharedPtr get_all_chemical_locations_service_;
    rclcpp::Service<pgsql_interfaces::srv::GetAllWorkstationLocations>::SharedPtr get_all_workstation_locations_service_; 
    rclcpp::Service<pgsql_interfaces::srv::RemoveChemical>::SharedPtr remove_chemical_service_;
    rclcpp::Service<pgsql_interfaces::srv::RemoveWorkstation>::SharedPtr remove_workstation_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DatabaseService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
