import requests
import json
import math
import time

class MiR():

    def __init__(self):

        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Accept-Language'] = 'en_US'
        self.headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
        self.group_id = 'mirconst-guid-0000-0011-missiongroup'
        self.session_id = '85cd7f3f-f2b7-11ea-ad20-0001299f16e3' #'a2f5b1e6-d558-11ea-a95c-0001299f04e5'
        
    # get the system information
    def get_system_info(self, mir_ip):
        result = requests.get(mir_ip + 'status', headers=self.headers)

        return result.json()

    # get all missions
    def get_all_missions(self, mir_ip):
        result = requests.get(mir_ip + 'missions', headers=self.headers)

        return result.json()

    # get missions
    def get_specific_mission(self, mir_ip, guid):
        result = requests.get(mir_ip + 'missions/' + guid, headers=self.headers)

        return result.json()

    # get actions of a missions
    def get_actions_of_mission(self, mir_ip, guid):
        result = requests.get(mir_ip + 'missions/' + guid + '/actions', headers=self.headers)

        return result.json()

    # get all maps infomation
    def get_maps(self, mir_ip):
        result = requests.get(mir_ip + 'maps', headers=self.headers)

        return result.json()

    def get_specific_maps(self, mir_ip, guid):
        result = requests.get(mir_ip + 'maps/' + guid, headers=self.headers)

        return result.json()

    def get_register(self, mir_ip):
        result = requests.get(mir_ip + 'registers', headers=self.headers)

        return result.json()


    # get specific map by the map name
    def get_map_positions(self, mir_ip, map_name):
        result = requests.get(mir_ip + 'maps/' + map_name + '/positions', headers=self.headers)

        return result.json()

        # get positions details

    def get_all_position(self, mir_ip):
        result = requests.get(mir_ip + 'positions', headers=self.headers)

        return result.json()

    # get positions details
    def get_specific_position(self, mir_ip, guid):
        result = requests.get(mir_ip + 'positions/' + guid, headers=self.headers)

        return result.json()

    # get a specific guid from the name of a position
    def get_position_guid(self, mir_ip, name):
        positions = self.get_all_position(mir_ip)
        for item in positions:
            if item['name'] == name:
                guid = item['guid']
                break

        return guid

    # post a new mission
    def post_mission(self, mir_ip, name):
        parameters = {"name": name, "hidden": False, "group_id": self.group_id, 'session_id': self.session_id}
        post_mission = requests.post(mir_ip + 'missions', json=parameters, headers=self.headers)
        print(post_mission)

        return post_mission.json()

    # post actions to mission
    def post_action_to_mission(self, mir_ip, mission_id, position_id, action_type):
        parameters = {'action_type': action_type, 'mission_id': mission_id,
        'parameters': [
        {'id': 'position', 'input_name': None, 'value': position_id},
        {'id': 'cart_entry_position', 'input_name': None, 'value': 'main'},
        {'id': 'main_or_entry_position', 'input_name': None, 'value': 'main'},
        {'id': 'marker_entry_position', 'input_name': None, 'value': 'entry'},
        {'id': 'retries', 'input_name': None, 'value': 10},
        {'id': 'distance_threshold', 'input_name': None, 'value': 0.1}],
        'priority': 1}

        result = requests.post(mir_ip + 'missions/' + mission_id + '/actions', json=parameters, headers=self.headers)

        return result.json()

    # post position
    def post_position(self, mir_ip, name):

        system_info = self.get_system_info(mir_ip)
        position = system_info['position']
        pos_x = position['x']
        pos_y = position['y']
        orientation = position['orientation']
        map_id = system_info['map_id']

        parameters = {"name": name, "pos_x": pos_x, "pos_y": pos_y, "orientation": orientation, "type_id": 0,
                      "map_id": map_id}
        post_position = requests.post(mir_ip + 'positions', json=parameters, headers=self.headers)

    def post_to_mission_queue(self, mir_ip, mission_id):
        mission_id = {"mission_id": mission_id}
        post_mission = requests.post(mir_ip + 'mission_queue', json=mission_id, headers=self.headers)

        return post_mission

    def get_mission_queue(self, mir_ip):
        result = requests.get(mir_ip + 'mission_queue', headers=self.headers)

        return result.json()

    def get_spe_mission_from_queue(self, mir_ip, queue_id):
        result = requests.get(mir_ip + 'mission_queue/' + queue_id, headers=self.headers)

        return result.json()

    # pause robot executing
    def put_state_to_pause(self, mir_ip):
        parameters = {"state_id": 4}
        requests.put(mir_ip + 'status', json=parameters, headers=self.headers)

    # start executing
    def put_state_to_execute(self, mir_ip):
        parameters = {"state_id": 3}
        requests.put(mir_ip + 'status', json=parameters, headers=self.headers)

    # start mir
    def put_state_to_start(self, mir_ip):
        parameters = {"state_id": 1}
        requests.put(mir_ip + 'status', json=parameters, headers=self.headers)

    # start shutdown
    def put_state_to_shutdown(self, mir_ip):
        parameters = {"state_id": 2}
        requests.put(mir_ip + 'status', json=parameters, headers=self.headers)

    # Abort Mission
    def put_state_to_abort(self, mir_ip):
        parameters = {"state_id": 6}
        requests.put(mir_ip + 'status', json=parameters, headers=self.headers)

    # get the details of a mission
    def get_mission_guid(self, mir_ip, name):
        missions = self.get_all_missions(mir_ip)
        for item in missions:
            if item['name'] == name:
                mission = self.get_specific_mission(mir_ip, item['guid'])
                break

        return mission['guid']

    # function for deleting missions by name
    def delete_mission(self, mir_ip, name):
        missions = self.get_all_missions()
        result = None
        for item in missions:
            if item['name'] == name:
                guid = item['guid']
                result = requests.delete(mir_ip + 'missions/' + guid, headers=self.headers)
                break

        if result == None:
            result = 'No mission named {0}'.format(name)

        return result

    # function for deleting positions by name
    def delete_position(self, mir_ip, name):
        positions = self.get_all_position(mir_ip)
        result = None
        for item in positions:
            if item['name'] == name:
                guid = item['guid']
                result = requests.delete(mir_ip + 'positions/' + guid, headers=self.headers)
                # break
        if result == None:
            result = 'No position named {0}'.format(name)

        return result

    # get details of a specific mission's actions
    def get_details_mission_actions(self, guid):
        actions = self.get_actions_of_mission(guid)
        len_actions = len(actions)
        i = 1
        for item in actions:
            guid = item['parameters'][0]['value']
            result = self.get_specific_position(guid)
            i = i + 1
            text = 'text'

        return text

    # create a mission with a certain name and return it's guid
    def create_mission(self, name):
        result = self.post_mission(name)
        print(result)
        mission_id = result['guid']

        return mission_id

    # create an action in a specific mission with a default action_type move
    def create_action(self, mission_id, position_name, action_type='move'):
        all_position = self.get_all_position()
        for item in all_position:
            if item['name'] == position_name:
                guid = item['guid']
                break
        # create an action with the specific type
        result = self.post_action_to_mission(mission_id, guid, action_type)

        return result, guid

    # calculate distance in meters between two points
    def cal_distance(self, origin, dist):

        return geodesic(origin, dist).meters

    # get the current position of the robot if accessible otherwise return None
    def get_current_position(self, mir_ip):
        sys_info = None
        tries = 0
        while tries < 10:
            try:
                sys_info = self.get_system_info(mir_ip)
                origin = (sys_info['position']['x'], sys_info['position']['y'], sys_info['position']['orientation'])
                return origin

            except KeyError:
                print('Retrying to get information!', 'Time: ', time.time())
                time.sleep(0.1)

        return None

    # get the position defined on the map with the shortest eucledian distance to the robot
    def get_nearest_position(self):
        origin = self.get_current_position()
        all_positions = self.get_all_position()
        best_distance = float("inf")
        cloest_location = ''
        for item in all_positions:
            if item['name'] != 'Config position':
                position = self.get_specific_position(item['guid'])
                dist = (position['pos_x'], position['pos_y'])
                temp_distance = self.cal_distance(origin, dist)
                if temp_distance < best_distance:
                    best_distance = temp_distance
                    cloest_location = item['name']

        return (best_distance, cloest_location)

    # check if the destination has been reached by comparing current position to a specific position
    def check_reach_des(self):
        origin = self.get_current_position()
        all_positions = self.get_all_position()
        best_distance = float("inf")
        cloest_location = ''
        for item in all_positions:
            if item['name'] != 'Config position':
                position = self.get_specific_position(item['guid'])
                temp_distance = math.sqrt(
                    math.pow(
                        origin[0] -
                        position['pos_x'],
                        2) +
                    math.pow(
                        origin[1] -
                        position['pos_y'],
                        2))
                if temp_distance < best_distance:
                    best_distance = temp_distance
                    cloest_location = item['name']

        return best_distance, cloest_location

    # get details of the mission that is currently being executed
    def get_exe_mission(self):
        exe_mission = self.get_mission_queue()
        mission_name = 'None'
        for item in exe_mission:
            if item['state'] == 'Executing':
                mission_gen = self.get_spe_mission_from_queue(str(item['id']))
                mission_detail = self.get_specific_mission(mission_gen['mission_id'])
                mission_name = mission_detail['name']

        return mission_name

    # Check if there are pending missions in the queue
    def get_pending_mission(self, mir_ip):
        mission_queue = self.get_mission_queue(mir_ip)
        for item in mission_queue:
            if item['state'] == 'Pending':
                return True

        return False

    # Check if a mission is done or not
    def get_mission_done_or_not(self,mir_ip, id):
        exe_mission = self.get_mission_queue(mir_ip)
        
        for item in exe_mission:
            if (item['id'] == id):
                print(item['state'])
            if (item['id'] == id) and (item['state'] == 'Done'):
                return True

        return False

    def get_mission_latest_mission_status(self,mir_ip):
        exe_mission = self.get_mission_queue(mir_ip)
        if exe_mission[-1]['state'] == 'Done':
            return True

        return False

    # move the mir robot with joystick using continuos velocity messages
    def move_mir(self, mir_ip, state_id, velocity, joystick_web_session_id):
        parameters = {"velocity":velocity}
        requests.put(mir_ip + 'status', json=parameters, headers=self.headers)

    # added so the position of the MiR can be changed
    def set_position(self, mir_ip, guid, x, y , orientation):
        parameters = {"pos_x": x, "pos_y": y, "orientation": orientation}
        response = requests.put(mir_ip + "positions/" + guid, headers=self.headers, json=parameters)

        return response.json()
    
    # change the state of the robot manually from (1 to 5)
    def chang_manual(self, mir_ip, state_id):
        parameters = {"state_id": state_id}
        requests.put(mir_ip + 'status', json=parameters, headers=self.headers)

    # Send mission to mir
    def set_mission(self, mir_ip, GUID):
        data = {"mission_id": GUID}
        response = requests.post(mir_ip + "mission_queue", headers=self.headers, json=data)

        return response.json()

    # Delete the mission queue
    def delete_mission_queue(self, mir_ip):
        response = requests.delete(mir_ip + "mission_queue/", headers=self.headers)
        
    # Delete a specific mission based upon mission_id
    def delete_specific_mission(self, mir_ip, guid):
        response = requests.delete(mir_ip + "mission_queue/" + guid, headers=self.headers)
        
        return response.json()
    
    def move_to_coordinate(self, mir_ip, x_dst, y_dst,ori):
        move_to_coordinate = {
        'mission_id': '0d0dedde-f982-11ed-a50a-20a7870098ae',
        'parameters': [
    	{'input_name': 'x', 'value': x_dst},
        {'input_name': 'y', 'value': y_dst},
        {"input_name": "ori", "value":ori}     
        ],	 
        'message': '',
        'priority': 1
        }
        response = requests.post(mir_ip + "mission_queue", headers=self.headers, json=move_to_coordinate)

    def move_to_pos(self, mir_ip, mission_id, pos_guid):
        move_to_pos = {
        'mission_id': mission_id,
        'parameters': [
    	{'input_name': 'pos', 'value': pos_guid}
        ],	 
        'message': '',
        'priority': 1
        }
        response = requests.post(mir_ip + "mission_queue", headers=self.headers, json=move_to_pos)
        # print(response)
    
    def set_desired_speed(self, mir_ip, speed):
        body = {'value': speed}
        response = requests.put(mir_ip + "settings/2078", headers=self.headers, json=body)

    def set_path_deviation(self, mir_ip, deviation):
        body = {'value': deviation}
        response = requests.put(mir_ip + "settings/2070", headers=self.headers, json=body)

    def set_maximum_distance_from_path(self, mir_ip, dist):
        body = {'value': dist}
        response = requests.put(mir_ip + "settings/1769", headers=self.headers, json=body)

    def set_waiting_for_obstacle(self, mir_ip, sec):
        body = {'value': sec}
        response = requests.put(mir_ip + "settings/2069", headers=self.headers, json=body)

    def post_pos_to_path(self, mir_ip, path_guid, pos_guid, pos_type):
        parameters = {
        "path_guide_guid": path_guid,
        "pos_guid": pos_guid,
        "pos_type": pos_type,
        "priority": 1
         }
        response = requests.post(mir_ip + 'path_guides/' + path_guid + '/positions', json = parameters, headers = self.headers)      
        
    def get_map_guid(self, mir_ip, name):
        result = self.get_maps(mir_ip)
        for item in result:
            if item['name'] == name:
                guid = item['guid']
                break

        return guid
    
    def get_all_path(self, mir_ip, map_id):
        response = requests.get(mir_ip + 'maps/' + map_id + '/path_guides', headers = self.headers)
        return response.json()
    
    def get_path_guid(self, mir_ip, map_id, name):
        response = self.get_all_path(mir_ip, map_id)
        for item in response:
            if item['name'] == name:
                guid = item['guid']
                break

        return guid
    