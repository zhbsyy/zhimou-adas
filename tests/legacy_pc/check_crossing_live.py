"""Bounded real CARLA crossing check. Run only with other consoles closed."""
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'PC_Windows'),str(ROOT/'PC_Windows/Console_Application'),'D:/CARLA_download/PythonAPI/carla']
from bridge.carla_backend import CarlaBackend
from bridge.kinematic_backend import VehicleState
from pedestrian_crossing import install,spawn_crossing


def main():
    install(CarlaBackend)
    backend=CarlaBackend(on_event=print)
    result={'passed':False,'scope':'actual CARLA crossing geometry and walker movement; ego remains parked'}
    try:
        assert backend.connect(),backend.get_status()
        backend.reset_to_default_spawn()
        assert backend.set_destination(98.8,82.9,'横穿走廊检查')
        path=backend.trajectory_waypoints
        assert spawn_crossing(backend,path[10]),'no validated crossing found'
        result['checks']=backend._scene_checks
        walker=backend._scene_walker
        start=walker.get_location()
        result['start']=[start.x,start.y,start.z]
        result['site']={k:[v.x,v.y,v.z] for k,v in backend._scene_site.items() if k in ('start','target','center')}
        result['samples']=[]
        # Exercise only walker update; never apply driving controls to the ego.
        state=VehicleState(gear='D',simulation_running=True)
        deadline=time.monotonic()+12
        while backend._scene_walker is not None and time.monotonic()<deadline:
            backend._update_pedestrian_scene(state,0.,30.)
            if backend._scene_walker is not None and (not result['samples'] or time.monotonic()-result['samples'][-1]['at']>.8):
                p=walker.get_location()
                result['samples'].append({'at':time.monotonic(),'xyz':[p.x,p.y,p.z],
                    'frame':backend.world.get_snapshot().frame,'alive':walker.is_alive})
            time.sleep(.05)
        result.update(passed=backend._scene_crossing_done,blocked=backend._scene_blocked,
                      crossing_started=backend._scene_crossing_started)
        assert result['passed'],'walker did not reach validated crossing end'
    finally:
        backend.disconnect()
        out=ROOT/'PC_Windows/Results'; out.mkdir(exist_ok=True)
        (out/'CROSSING_LIVE_CHECK.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':
    main()
