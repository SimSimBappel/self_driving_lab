import mir_api
import time
import os

recharge_mission_id = "a3339ee8-1d04-11eb-b714-0001299f16e3"
#measurement_mission_id = "d7c236c8-bbf0-11ed-bbbc-20a7870098ae"
measurement_mission_id = "68c56b57-1f1c-11ee-8931-20a7870098ae"

charge_threshold = 35

measurements = [
    "ping 1.1.1.1 -c 60",
    "ping 8.8.8.8 -c 60",
]

def run_measurement(command):
    os.system(command)


def main():
    mir = mir_api.MiR()
    mir_url = "http://192.168.100.51/api/v2.0.0/"

    for measurement in measurements:
        status = mir.get_system_info(mir_url)

        if status['battery_percentage'] > charge_threshold:
            mir.delete_mission_queue(mir_url)
            mir.post_to_mission_queue(mir_url, measurement_mission_id)
            run_measurement(measurement)

        else:
            mir.post_to_mission_queue(mir_url, recharge_mission_id)
            while True:
                time.sleep(60)
                status = mir.get_system_info(mir_url)
                if status['battery_percentage'] > 80:
                    break

            mir.delete_mission_queue(mir_url)
            mir.post_to_mission_queue(mir_url, measurement_mission_id)
            run_measurement(measurement)

    mir
    mir.delete_mission_queue(mir_url)
    mir.post_to_mission_queue(mir_url, recharge_mission_id)


main()