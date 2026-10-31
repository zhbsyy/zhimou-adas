"""Isolated PC integration: synthetic UART/RK inputs, real CARLA actuator.

No real RK/Android device is claimed. Never run alongside a user console:
the production arbiter exclusively owns UDP8888 for this short test.
"""
import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'PC_Windows'),str(ROOT/'PC_Windows/Console_Application'),'D:/CARLA_download/PythonAPI/carla']
from navigation_arbiter import NavigationArbiter
from pedestrian_crossing import install as install_crossing
from bridge.carla_backend import CarlaBackend
install_crossing(CarlaBackend)
from control_guard import SCHEMA
from bridge.protocol_5aa5 import crc16_ccitt


def nav_bytes(sequence):
    payload=struct.pack('<HHIIIHHBB',10001,1,100,153,60,0,20,1,0)
    body=struct.pack('<HHHH',1,1,sequence&65535,len(payload))+payload
    return b'\x5a\xa5'+body+struct.pack('<H',crc16_ccitt(body))+b'\xbb\xbb'


def wait_for(predicate,label,seconds=8.):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError(label)


def main():
    records=ROOT/'PC_Windows/Results'
    records.mkdir(exist_ok=True)
    arbiter=None
    stopped=threading.Event()
    tcp=socket.socket()
    tcp.bind(('127.0.0.1',0)); tcp.listen(1); tcp.settimeout(2.)
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    phases=[]
    telemetry=[]
    scenario={'emergency':False,'clear':False,'send':False,'healthy':True}
    def report():
        return {'schema':SCHEMA,'source':'carla','session_id':arbiter.protocol.session_id,
            'run_id':'test-run-000000000000000000000000','epoch':arbiter.epoch,
            'det_report_seq':report.seq,'aeb_triggered':scenario['emergency'],
            'brake_pressure':1. if scenario['emergency'] else 0.,'action':'PC测试输入（非RK实测）',
            'evidence':{'actor_id':42,'visual_person_matched':scenario['emergency'],'confirmed_hits':2,
                'capture_fresh':True,'valid_scene':True,'path_clear':scenario['clear'],'clear_actor_id':42}}
    report.seq=0
    def horizontal_speed():
        v=arbiter.carla.vehicle.get_velocity()
        return 3.6*(v.x*v.x+v.y*v.y)**.5
    def inputs():
        connection,_=tcp.accept()
        connection.settimeout(.02)
        buffer=bytearray(); sequence=0
        while not stopped.is_set():
            sequence+=1
            frame={'action':'SIMULATOR_STATE','gear':'D','simulation_running':True,
                'target_speed_kmh':20,'speed_limit_kmh':20,'destination_x':98.8,'destination_y':82.9,
                'destination_name':'PC协议回归目标','navigation_icon':1}
            try:
                connection.sendall(nav_bytes(sequence)+json.dumps(frame).encode()+b'\n')
                chunk=connection.recv(32768)
                buffer.extend(chunk)
                while b'\n' in buffer:
                    line,_,rest=buffer.partition(b'\n'); buffer[:]=rest
                    try:
                        telemetry.append(json.loads(line))
                    except ValueError:
                        pass
            except socket.timeout:
                pass
            except OSError:
                break
            if scenario['send']:
                report.seq+=1
                udp.sendto(json.dumps(report()).encode(),('127.0.0.1',8888))
            stopped.wait(.1)
        connection.close()
    def save_phase(name):
        position=arbiter.carla.vehicle.get_location()
        value={'name':name,'speed_kmh':arbiter.state.actual_speed_kmh,'gear':arbiter.state.gear,
            'horizontal_speed_kmh':horizontal_speed(),
            'result':dict(arbiter.last_result),'position':[position.x,position.y],
            'route_name':arbiter.carla.current_dest_name,'path_count':len(arbiter.carla.trajectory_waypoints)}
        phases.append(value)
        print(json.dumps(value,ensure_ascii=False),flush=True)
    try:
        arbiter=NavigationArbiter(on_log=lambda _:None)
        with arbiter.operation_lock:
            if not arbiter.connect_carla():
                raise RuntimeError(arbiter.carla.get_status())
        arbiter.protocol.board_ip='127.0.0.1'  # Test object only; production default stays the RK address.
        arbiter.protocol.select_run('test-run-000000000000000000000000')
        arbiter.session=SimpleNamespace(perception_reason=lambda _:'' if scenario['healthy'] else '测试感知中断')
        sender=threading.Thread(target=inputs,daemon=True); sender.start()
        if not arbiter.serial.connect('socket://127.0.0.1:'+str(tcp.getsockname()[1])):
            raise RuntimeError('test UART socket could not connect')
        origin=arbiter.carla.vehicle.get_location()
        wait_for(lambda:horizontal_speed()>5. and
                 abs(arbiter.carla.vehicle.get_location().x-origin.x)+abs(arbiter.carla.vehicle.get_location().y-origin.y)>.5,
                 'pure navigation without RK did not physically move forward',12.)
        save_phase('pure_navigation_no_RK')
        original_route=arbiter.carla.trajectory_waypoints
        scenario.update(send=True,emergency=True)
        time.sleep(.5)
        assert not arbiter.state.vision_aeb_active
        assert arbiter.last_result['drive']
        save_phase('RK_report_present_but_AEB_off')
        scenario['emergency']=False
        arbiter.set_aeb_enabled(True)
        wait_for(lambda:arbiter.aeb_enabled and not arbiter.intent.running,'switch did not pause')
        wait_for(lambda:arbiter.state.actual_speed_kmh<.3,'switch did not park')
        arbiter.set_gear('D')
        wait_for(lambda:arbiter.state.actual_speed_kmh>5.,'AEB-enabled navigation did not start',6.)
        scenario['emergency']=True
        wait_for(lambda:arbiter.last_result['brake'],'AEB packet did not trigger brake')
        wait_for(lambda:arbiter.state.actual_speed_kmh<.3,'AEB did not stop vehicle')
        save_phase('AEB_stop')
        scenario.update(emergency=False,clear=False)
        time.sleep(1.3)
        assert not arbiter.last_result['drive']
        save_phase('missing_detection_does_not_resume')
        scenario['clear']=True
        wait_for(lambda:arbiter.last_result['drive'],'positive clearance did not release',4.)
        wait_for(lambda:arbiter.state.actual_speed_kmh>3.,'vehicle did not accelerate after clearance',5.)
        assert arbiter.carla.trajectory_waypoints is original_route
        save_phase('resumed_original_navigation')
        scenario.update(emergency=True,clear=False)
        wait_for(lambda:arbiter.last_result['brake'],'second AEB did not trigger')
        arbiter.set_gear('P')
        wait_for(lambda:not arbiter.intent.running,'manual pause did not stick')
        scenario.update(emergency=False,clear=True)
        time.sleep(1.5)
        assert not arbiter.intent.running and not arbiter.last_result['drive']
        save_phase('manual_pause_blocks_auto_resume')
        arbiter.set_gear('D')
        wait_for(lambda:arbiter.last_result['drive'],'explicit resume did not run',4.)
        scenario['healthy']=False
        wait_for(lambda:arbiter.last_result['phase']=='fault','perception loss did not fault')
        scenario['healthy']=True
        time.sleep(.7)
        assert not arbiter.last_result['drive']
        save_phase('reconnect_does_not_silently_resume')
        result={'passed':True,'scope':'PC test fixtures over UART/UDP + actual CARLA; no physical Android/RK',
            'phases':phases,'received_android_telemetry_frames':len(telemetry),'control_error':arbiter.control_error}
        (records/'PC_INTEGRATION_CHECK.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    finally:
        stopped.set()
        if arbiter:
            arbiter.intent.pause()
            time.sleep(.1)
            arbiter.close()
        tcp.close(); udp.close()


if __name__=='__main__':
    main()
