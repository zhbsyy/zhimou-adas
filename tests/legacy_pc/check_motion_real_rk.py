"""Actual CARLA + physical RK test, PC-selected routes, without cockpit UART.

Run only after the user console has exited. This does not certify Android
navigation: no serial frames, person detections or AEB reports are injected.
"""
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'PC_Windows'),str(ROOT/'PC_Windows/Console_Application'),
             'D:/CARLA_download/PythonAPI/carla']
from PyQt5 import QtWidgets
from bridge.serial_client import SerialClient
SerialClient.find_preferred_port=classmethod(lambda cls:None)
import bridge.arbiter
from pedestrian_crossing import install
from navigation_motion import install_collision_monitor
install(bridge.arbiter.CarlaBackend)
install_collision_monitor(bridge.arbiter.CarlaBackend)
from navigation_arbiter import NavigationArbiter
bridge.arbiter.SimulationArbiter=NavigationArbiter
from ui.dashboard import DashboardWindow
from navigation_session import NavigationSession


def main():
    app=QtWidgets.QApplication([])
    window=DashboardWindow()  # Not shown: software integration harness only.
    session=NavigationSession(window)
    a=window.arbiter
    a.on_log=lambda text:print(text,flush=True)
    a.carla.on_event=a.log
    result={'scope':__doc__.split('\n\n')[0], 'passed':False, 'routes':[]}
    def wait(predicate,seconds=12):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate():return
            time.sleep(.025)
        raise AssertionError('condition timeout: '+str(a.last_result)+' / health='+str(session.perception_reason(time.monotonic()))+' / checks='+str(getattr(a.carla,'_scene_checks',None)))
    try:
        session.prepare()
        wait(lambda:session.streamer and session.streamer.health()['ready'],30)
        for x,y in [(98.8,82.9)]:
            a.set_gear('P')
            wait(lambda:not a.intent.running)
            with a.operation_lock:
                a.carla.reset_to_default_spawn()
                a.state.cockpit_drive_mode=False
                a.state.system_mode='perception_debug'
                a.state.target_speed_kmh=40.
                a.state.speed_limit_kmh=40.
                a.arrived=False
                a.flow.reset()
            a.set_aeb_enabled(True)
            a.set_destination(x,y,'PC指定路线·真实RK控速检查',False)
            wait(lambda:a.carla.trajectory_waypoints and a.carla._scene_site is not None,20)
            wait(lambda:session.perception_reason(time.monotonic())=='',20)
            a.set_gear('D',False)
            wait(lambda:a.last_result.get('drive'),8)
            route={'destination':[x,y],'samples':[],'passed':False}
            initial_resumes=a.automatic_resumes
            result['routes'].append(route)
            started=time.monotonic();last_phase=None
            while time.monotonic()-started<110:
                app.processEvents()
                d=session.snapshot()
                d['elapsed_s']=round(time.monotonic()-started,3)
                route['samples'].append(d)
                phase=(d['result']['phase'],d['scene'].get('pedestrian_phase'))
                if phase!=last_phase:
                    print(json.dumps({'phase':phase,'speed':d['speed_kmh'],'entry':d['crossing_entry'],
                        'remaining':d['route_remaining_m'],'collisions':d['collisions']},ensure_ascii=False),flush=True)
                    last_phase=phase
                if d['collisions'] or d['result']['phase']=='fault':break
                if d['arrived'] and d['speed_kmh']<.3:break
                time.sleep(.1)
            rows=route['samples']
            emergency=[d for d in rows if d['rk_report'].get('aeb_triggered') and
                       d['rk_report'].get('evidence',{}).get('visual_person_matched')]
            stopped=any(d['speed_kmh']<.3 and d['result']['phase'] in ('braking','waiting_clear') for d in rows)
            resumed=rows[-1]['automatic_resumes']>initial_resumes
            route.update(real_rk_emergency=bool(emergency),stopped=stopped,resumed=resumed,
                arrived=rows[-1]['arrived'],collisions=rows[-1]['collisions'],entry=rows[-1]['crossing_entry'],
                max_speed_kmh=max(d['speed_kmh'] for d in rows))
            route['passed']=bool(emergency and stopped and resumed and route['arrived'] and not route['collisions']
                and 1/3<=route['entry'].get('route_fraction',0)<=2/3 and route['max_speed_kmh']<42)
            print('ROUTE_RESULT',json.dumps({k:v for k,v in route.items() if k!='samples'},ensure_ascii=False),flush=True)
            if route['collisions']:break
        result['passed']=len(result['routes'])==1 and all(r['passed'] for r in result['routes'])
    except Exception as exc:
        result['error']=repr(exc)
        print('CHECK_ERROR',repr(exc),flush=True)
    finally:
        session.close()
        a.close()
        out=ROOT/'PC_Windows/Results';out.mkdir(exist_ok=True)
        (out/'MOTION_REAL_RK.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print('MOTION_REAL_RK',json.dumps({k:v for k,v in result.items() if k!='routes'},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
