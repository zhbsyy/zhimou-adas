"""RK visual pedestrian gate + explicitly labelled CARLA actor-truth range.

No stereo claim: four CARLA cameras face different directions. A projected
scene box is only used to associate an RK person detection with the demo walker.
Without a current matching detection, actor telemetry cannot trigger AEB.
"""
import json
import math
import socket
import threading
import time
import uuid

SCHEMA = 'carla_nav_aeb.control.v3'


def matching_person(dets, scene):
    expected = scene.get('bbox_front')
    if not expected or len(expected) != 4:
        return False
    try:
        x1, y1, x2, y2 = map(float, expected)
    except (TypeError,ValueError):
        return False
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)) or min(x2-x1, y2-y1) < 1:
        return False
    margin = max(12., .35*(x2-x1))
    for a, b, c, d, score, cid in dets:
        if not all(math.isfinite(float(v)) for v in (a, b, c, d, score, cid)):
            continue
        # Detector coordinates are restored to the full 1280x720 mosaic.
        if float(cid) != 0 or not .40 <= float(score) <= 1. or not (0 <= a < c <= 640 and 0 <= b < d <= 360):
            continue
        cx, cy = (float(a)+float(c))/2, (float(b)+float(d))/2
        inter = max(0, min(c, x2)-max(a, x1)) * max(0, min(d, y2)-max(b, y1))
        if inter > 0 and x1-margin <= cx <= x2+margin and y1-12 <= cy <= y2+12:
            return True
    return False


class AebPolicy:
    def __init__(self):
        self.hits = 0
        self.actor_id = None

    def evaluate(self, dets, telemetry, age_s):
        scene = telemetry.get('carla_aeb_scene', {})
        same_actor = scene.get('actor_id') == self.actor_id
        self.actor_id = scene.get('actor_id')
        if not same_actor:
            self.hits = 0
        try:
            speed = float(telemetry.get('vehicle', {}).get('speed', 0))
            distance, lateral = float(scene.get('forward_gap_m')), float(scene.get('lateral_m'))
            geometry_ok = all(math.isfinite(v) for v in (speed, distance, lateral))
            scene_age = float(scene.get('scene_age_ms',0))/1000
            geometry_ok = geometry_ok and math.isfinite(scene_age) and 0 <= scene_age+age_s <= .4
        except (TypeError, ValueError):
            speed, distance, lateral, geometry_ok = 0., 0., 0., False
        valid = (0 <= age_s <= .4 and scene.get('valid') is True and geometry_ok
                 and scene.get('range_source') == 'CARLA_actor_truth'
                 and telemetry.get('perception_source') == 'carla'
                 and telemetry.get('takeover_enabled') is True)
        matched = valid and matching_person(dets, scene)
        # Clearance needs positive geometry for the tracked actor, not merely
        # an empty detector result. Unknown other pedestrians in the projected
        # driving corridor veto release even after the demo actor has crossed.
        other_in_path=False
        for a,b,c,d,score,cid in dets:
            if cid!=0 or score<.4 or not (0<=a<c<=640 and 0<=b<d<=360):
                continue
            cx=(a+c)/2
            same=matching_person([[a,b,c,d,score,cid]],scene)
            if not same and 640*.3<=cx<=640*.7 and d>=360*.4:
                other_in_path=True
        positive_clear=scene.get('clear_state') in ('outside','completed')
        path_clear=bool(valid and positive_clear and not other_in_path)
        self.hits = self.hits + 1 if matched else 0
        corridor = matched and 0 < distance <= 30 and abs(lateral) <= 1.5
        ttc = distance/(speed/3.6) if corridor and speed > 1.8 else None
        emergency = bool(corridor and self.hits >= 2 and (distance < 3.5 or (ttc is not None and ttc < 2.0))
                         and telemetry.get('carla_aeb_enabled') is True)
        action = '紧急制动-行人' if emergency else ('前方有行人' if corridor else '正常巡航')
        return {'aeb_triggered': emergency, 'brake_pressure': 1.0 if emergency else 0.0,
            'action': action, 'vehicle_speed': None, 'steering_angle': 0.0, 'lane_change_state': 'idle',
            'stop_line_m': None, 'light_distance_m': None,
            'veto': ('RK检出行人；仿真前向间距%.1fm，TTC=%s' % (distance, '%.2fs' % ttc if ttc is not None else '--')) if corridor else '',
            'domain': 'ADAS', 'range_source': 'CARLA_actor_truth',
            'evidence': {'visual_person_matched': bool(matched), 'confirmed_hits': self.hits,
                'actor_id': scene.get('actor_id'), 'telemetry_age_ms': round(age_s*1000, 1),
                'forward_gap_m': round(distance, 2) if valid else None,
                'lateral_m': round(lateral, 2) if valid else None, 'ttc_s': round(ttc, 2) if ttc is not None else None,
                'carla_frame': scene.get('carla_frame'), 'valid_scene': bool(valid),
                'path_clear':path_clear,'clear_actor_id':scene.get('actor_id') if path_clear else None,
                'other_in_path_person':other_in_path,'clear_state':scene.get('clear_state')},
            'stamp': round(time.time(), 3)}


class CarlaAebLink:
    stats_sink = None

    def __init__(self, pc_ip='192.168.2.10', oib_port=8888, listen_port=8892, **unused):
        self.pc_ip, self.oib_port, self.listen_port = pc_ip, oib_port, listen_port
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.settimeout(.2)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.telemetry, self.received = {}, 0.
        self.policy, self.threads = AebPolicy(), []
        self.perception_source, self.vehicle_speed_kmh = 'carla', 0.
        self.count = 0
        self.run_id = uuid.uuid4().hex
        self.session_id = None
        self.last_state_seq = -1
        self.epoch=0

    def start(self):
        self.rx.bind(('0.0.0.0', self.listen_port))
        for function in (self.receive_loop, self.heartbeat_loop):
            thread = threading.Thread(target=function, daemon=True)
            thread.start()
            self.threads.append(thread)
        print('[CARLA AEB] visual person association + CARLA_actor_truth range; UDP8888/8892', flush=True)

    def send_raw_json(self, payload):
        self.tx.sendto(json.dumps(payload, ensure_ascii=False).encode(), (self.pc_ip, self.oib_port))

    def receive_loop(self):
        while not self.stop_event.is_set():
            try:
                blob, peer = self.rx.recvfrom(16384)
                if peer[0] != self.pc_ip:
                    continue
                payload = json.loads(blob)
                if not isinstance(payload,dict) or payload.get('type') != 'OIB_DOMAIN_STATE':
                    continue
                session_id = payload.get('carla_aeb_session')
                seq = payload.get('carla_aeb_state_seq')
                epoch=payload.get('carla_aeb_epoch')
                speed = float(payload.get('vehicle',{}).get('speed',0))
                if not isinstance(session_id,str) or len(session_id)!=32 or type(seq) is not int or type(epoch) is not int or not math.isfinite(speed):
                    continue
                with self.lock:
                    if session_id != self.session_id or epoch!=self.epoch:
                        self.last_state_seq = -1
                        self.policy = AebPolicy()
                    if seq <= self.last_state_seq:
                        continue
                    self.session_id,self.last_state_seq = session_id,seq
                    self.epoch=epoch
                    self.telemetry, self.received = payload, time.monotonic()
                    self.perception_source = payload.get('perception_source', 'carla')
                    self.vehicle_speed_kmh = speed
            except (socket.timeout, ValueError, TypeError,AttributeError):
                continue
            except OSError:
                break

    def heartbeat_loop(self):
        while not self.stop_event.is_set():
            try:
                self.send_raw_json({'message_type': 'heartbeat', 'domain': 'ADAS', 'stamp': time.time(),
                    'schema':SCHEMA,'source':'carla','run_id':self.run_id,'session_id':self.session_id,'epoch':self.epoch})
            except OSError:
                pass
            self.stop_event.wait(.5)

    def evaluate_and_report(self, dets, stereo_pairs=None, tl_state=None):
        with self.lock:
            telemetry, received, session_id, epoch = self.telemetry, self.received, self.session_id,self.epoch
        capture_age_ms = self.stats_sink.get('inf_age_ms', 9999.) if self.stats_sink is not None else 9999.
        capture_fresh = math.isfinite(capture_age_ms) and 0 <= capture_age_ms <= 500
        report = self.policy.evaluate(dets if capture_fresh else [], telemetry, time.monotonic()-received if received else 999.)
        report['evidence']['capture_age_ms'] = round(capture_age_ms, 1)
        report['evidence']['capture_fresh'] = capture_fresh
        self.count += 1
        report['det_report_seq'] = self.count
        report.update(schema=SCHEMA,source='carla',run_id=self.run_id,session_id=session_id,epoch=epoch)
        self.send_raw_json(report)
        if self.stats_sink is not None:
            self.stats_sink['carla_aeb'] = dict(report, enabled=True, policy='navigation_optional_aeb.v3')
        return report

    def stop(self):
        if self.stop_event.is_set():
            return
        try:
            self.count += 1
            self.send_raw_json({'aeb_triggered': False, 'brake_pressure': 0., 'action': '感知停止，测试暂停',
                'vehicle_speed': None, 'veto': 'CARLA AEB测试入口已停止', 'domain': 'ADAS',
                'schema':SCHEMA,'source':'carla','run_id':self.run_id,'session_id':self.session_id,
                'epoch':self.epoch,
                'det_report_seq':self.count,'evidence':{'valid_scene':False,'capture_fresh':False}})
        except OSError:
            pass
        self.stop_event.set()
        self.rx.close()
        for thread in self.threads:
            thread.join(1)
        self.tx.close()
