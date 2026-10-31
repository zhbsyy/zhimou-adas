"""Pure protocol validation for this isolated simulation; no Qt or CARLA calls."""
import math

SCHEMA = 'carla_nav_aeb.control.v3'


class ProtocolGate:
    def __init__(self, session_id, board_ip='192.168.2.3'):
        self.session_id, self.board_ip = session_id, board_ip
        self.run_id = None
        self.last_seq = 0
        self.epoch = 0

    def select_run(self, run_id):
        if not isinstance(run_id,str) or not 8 <= len(run_id) <= 64:
            return False
        if run_id != self.run_id:
            self.run_id, self.last_seq = run_id, 0
        return True

    def accept(self, payload, peer):
        if peer != self.board_ip or not isinstance(payload,dict):
            return None
        if payload.get('schema') != SCHEMA or payload.get('source') != 'carla':
            return None
        if payload.get('message_type') == 'heartbeat':
            return 'heartbeat'
        if payload.get('session_id') != self.session_id or payload.get('run_id') != self.run_id or not self.run_id:
            return None
        if payload.get('epoch')!=self.epoch:
            return None
        seq = payload.get('det_report_seq')
        if type(seq) is not int or seq <= self.last_seq:
            return None
        brake, emergency = payload.get('brake_pressure'), payload.get('aeb_triggered')
        if type(emergency) is not bool or type(brake) not in (int,float) or not math.isfinite(brake):
            return None
        if brake != (1. if emergency else 0.):
            return None
        evidence = payload.get('evidence')
        if not isinstance(evidence,dict):
            return None
        if emergency and not (evidence.get('visual_person_matched') is True
                              and evidence.get('capture_fresh') is True
                              and evidence.get('valid_scene') is True
                              and evidence.get('confirmed_hits',0) >= 2):
            return None
        self.last_seq = seq
        return 'control'


class ProgressWatch:
    def __init__(self):
        self.identity = None
        self.values, self.progressed = {}, {}

    def observe(self, stats, now):
        report = stats.get('carla_aeb',{})
        identity = (report.get('session_id'),report.get('run_id'))
        if identity != self.identity:
            self.identity, self.values, self.progressed = identity, {}, {}
        for key,value in [('det',report.get('det_report_seq')),
                          ('lane',stats.get('lane_count')),('jpeg',stats.get('stream_count'))]:
            if type(value) is not int or value < 0:
                continue
            old = self.values.get(key)
            if old is not None and value > old:
                self.progressed[key] = now
            elif old is not None and value < old:
                self.progressed.pop(key,None)
            self.values[key] = value

    def stale_reason(self, now,required=('det','lane','jpeg')):
        for key,label,limit in [('det','目标检测',1.),('lane','车道分割',1.5),('jpeg','视频发布',1.5)]:
            if key not in required:
                continue
            if key not in self.progressed or now-self.progressed[key] > limit:
                return label+'计数未持续更新'
        return ''
