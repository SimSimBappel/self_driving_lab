import mir_api
import time
import os

move_to_parking = "969b1d2a-e512-11ee-aeef-000e8e984489"
dock_to_pl = "9c8d176a-0c39-11ee-b580-000e8e984489"

def main():
    mir = mir_api.MiR()
    mir_url = "http://192.168.100.140/api/v2.0.0/"

    mir.post_to_mission_queue(mir_url, dock_to_pl)
    time.sleep(1)
    while True:
        try:
            status = mir.get_system_info(mir_url)

            if status['mission_text'] == 'Waiting for new missions ...':
                break

            print("Waiting for robot to get into position")
            time.sleep(1)
        except KeyError:
            time.sleep(1)
            print("Connection error. Retry")
            continue

    print("Done")

    mir.post_to_mission_queue(mir_url, move_to_parking)


main()