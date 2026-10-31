"""PC-only actuator check. No RK traffic, detection claim, or existing ego control."""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import time

import carla
sys.path.insert(0,str(Path(__file__).resolve().parent/'Console_Application'))
from safe_arbiter import full_emergency_brake


def main():
    client=carla.Client('127.0.0.1',2000)
    client.set_timeout(4.)
    world=client.get_world()
    road=world.get_map()
    vehicle=None
    records=[]
    def speed():
        v=vehicle.get_velocity()
        return 3.6*math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
    try:
        bp=world.get_blueprint_library().find('vehicle.tesla.model3')
        bp.set_attribute('role_name','carla_aeb_brake_probe')
        for point in road.get_spawn_points():
            start=road.get_waypoint(point.location)
            ahead=start.next(15.)
            if start.is_junction or not ahead:
                continue
            dy=(ahead[0].transform.rotation.yaw-start.transform.rotation.yaw+180)%360-180
            if ahead[0].is_junction or abs(dy)>3:
                continue
            vehicle=world.try_spawn_actor(bp,point)
            if vehicle:
                break
        if vehicle is None:
            raise RuntimeError('No free straight spawn for temporary actuator check')
        actor_id=vehicle.id
        vehicle.apply_control(carla.VehicleControl(hand_brake=True,brake=1.))
        time.sleep(.5)
        begin=vehicle.get_location()
        maximum=0.
        vehicle.apply_control(carla.VehicleControl(throttle=.45))
        deadline=time.monotonic()+2.5
        while time.monotonic()<deadline:
            maximum=max(maximum,speed())
            if maximum>=10.:
                break
            time.sleep(.03)
        state=SimpleNamespace(vision_action='PC执行器检查（非RK识别）',turn_signal='off')
        backend=SimpleNamespace(vehicle=vehicle,_current_throttle=.45,_current_brake=0.,
            _current_steer=0.,_target_v_smooth=20.,_update_camera=lambda:None)
        requested=time.monotonic()
        full_emergency_brake(backend,state)
        deadline=requested+4
        while time.monotonic()<deadline:
            current=speed()
            ctrl=vehicle.get_control()
            records.append({'elapsed_s':round(time.monotonic()-requested,3),'speed_kmh':round(current,3),
                            'throttle':ctrl.throttle,'brake':ctrl.brake})
            if current<.3:
                break
            time.sleep(.03)
        end=vehicle.get_location()
        result={'scope':'PC-only CARLA actuator check; no RK perception/UDP was simulated',
            'actor_id':actor_id,'max_speed_kmh':round(maximum,3),'end_speed_kmh':round(speed(),3),
            'stop_elapsed_s':round(time.monotonic()-requested,3),'total_travel_m':round(begin.distance(end),3),
            'brake_lamp':bool(vehicle.get_light_state() & carla.VehicleLightState.Brake),
            'samples':records,'passed':maximum>=3 and speed()<.3 and records[-1]['throttle']==0 and records[-1]['brake']==1}
        output=Path(__file__).resolve().parent/'Results/PC_ACTUATOR_CHECK.json'
        output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({key:value for key,value in result.items() if key!='samples'},ensure_ascii=False),flush=True)
    finally:
        if vehicle is not None:
            vehicle.apply_control(carla.VehicleControl(throttle=0.,brake=1.,hand_brake=True))
            vehicle.destroy()


if __name__=='__main__':
    main()
