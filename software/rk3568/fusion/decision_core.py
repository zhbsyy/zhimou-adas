"""Small RK-side rules. No OpenCV, RKNN, CARLA or actuator dependency."""
from collections import OrderedDict, deque
from copy import deepcopy
import math
import re
import threading
import time
import uuid


class CommandError(ValueError):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


class DecisionCore:
    def __init__(self, config=None, clock=time.monotonic, aeb_config=None):
        self.clock = clock
        self.config = {"signal_ttl_s": .75, "lane_ttl_s": .75,
                       "source_ttl_s": 1.0, "command_lease_s": 3.0,
                       **(config or {})}
        for key, value in self.config.items():
            if key in ("normal_driving_default", "red_only_stop"):
                if type(value) is not bool:
                    raise ValueError("Expected boolean rule setting: " + key)
                continue
            if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError("Invalid positive rule setting: " + key)
        from pedestrian_aeb import validate_config
        self.aeb_config = validate_config(aeb_config)
        self.lock = threading.RLock()
        self.events = deque(maxlen=128)
        self.event_seq = 0
        self.command_cache = OrderedDict()
        self.client_sequences = OrderedDict()
        self.context = {"scenario": "intersection", "lane": "straight"}
        self.context_revision = 0
        self.reset_source("not_started", evidence_mode="live")

    def _event(self, name, **details):
        self.event_seq += 1
        self.events.append({"event_seq": self.event_seq, "event": name, **details})

    def reset_source(self, name, evidence_mode="live"):
        with self.lock:
            self.epoch = uuid.uuid4().hex
            self.source = name
            self.evidence_mode = evidence_mode
            self.frame_seq = 0
            self.frame_at = None
            self.source_error = None
            self.channels = {"signal": None, "boundaries": None, "obstacles": None}
            self.aeb_latched = False
            self.intent = "move" if self.config.get("normal_driving_default") else "stop"
            self.intent_at = self.clock() if self.intent == "move" else None
            self.alarm = None
            self.last_ack = None
            self._event("source_reset", source=name, source_epoch=self.epoch)
            return self.epoch

    def note_frame(self, epoch, seq, received_mono):
        with self.lock:
            if epoch != self.epoch or type(seq) is not int or seq <= self.frame_seq:
                return False
            if not self._valid_stamp(received_mono):
                return False
            self.frame_seq, self.frame_at = seq, received_mono
            self.source_error = None
            return True

    def _valid_stamp(self, value):
        return (isinstance(value, (int, float)) and math.isfinite(value)
                and 0 <= value <= self.clock()+.01)

    def source_failed(self, message):
        with self.lock:
            self.source_error = str(message)[:160]
            self.channels = {"signal": None, "boundaries": None, "obstacles": None}
            if not (self.config.get("normal_driving_default") and self.intent == "move"):
                self.intent, self.intent_at = "stop", None
            self._event("source_error", reason=self.source_error)

    def publish(self, channel, epoch, seq, received_mono, result, context_revision=None):
        with self.lock:
            if channel not in self.channels or epoch != self.epoch:
                return False
            prior = self.channels[channel]
            if (type(seq) is not int or seq < 1 or seq > self.frame_seq or
                    (prior and seq <= prior["frame_seq"]) or not self._valid_stamp(received_mono)):
                return False
            # Signal selection belongs to the lane context at inference submission.
            if channel == "signal" and context_revision != self.context_revision:
                return False
            self.channels[channel] = {"frame_seq": seq, "received_mono": received_mono,
                                      "completed_mono": self.clock(), "result": deepcopy(result)}
            if channel == "obstacles" and 0 <= self.clock()-received_mono <= self.aeb_config['ttl_s']:
                if result.get('valid') and result.get('level') == 'emergency':
                    if not self.aeb_latched:
                        self._event('pedestrian_emergency_brake', frame_seq=seq, mode=result.get('mode'))
                    self.aeb_latched = True
                    self._brake_intent()
                elif result.get('valid') and result.get('level') == 'clear' and self.aeb_latched:
                    self.aeb_latched = False
                    self._brake_intent()
                    self._event('pedestrian_brake_released', frame_seq=seq,
                                resume_normal=bool(self.config.get("normal_driving_default") and self.intent == "move"))
            return True

    def _brake_intent(self):
        # Default drive is a stationary demonstration state, never throttle.
        # Hazards temporarily veto it; a deliberate stop remains latched.
        if self.config.get("normal_driving_default") and self.intent != "stop":
            self.intent, self.intent_at = "move", self.clock()
        else:
            self.intent, self.intent_at = "stop", None

    def context_snapshot(self):
        with self.lock:
            return {**self.context, "revision": self.context_revision}

    def _perception(self, now):
        age = None if self.frame_at is None else max(0, now-self.frame_at)
        live = not self.source_error and age is not None and age <= self.config["source_ttl_s"]
        output = {"source": self.source, "source_epoch": self.epoch,
                  "evidence_mode": self.evidence_mode, "frame_seq": self.frame_seq,
                  "source_valid": bool(live), "source_age_ms": None if age is None else round(age*1000, 2),
                  "source_error": self.source_error}
        for key, ttl in (("signal", "signal_ttl_s"), ("boundaries", "lane_ttl_s")):
            sample = self.channels[key]
            valid = bool(live and sample and 0 <= now-sample["received_mono"] <= self.config[ttl])
            if key == "signal":
                result = {"color": "unknown", "valid": False, "reason": "no_current_signal"}
                if valid:
                    result.update(deepcopy(sample["result"]))
            else:
                result = {side: {"kind": "unknown", "valid": False, "reason": "no_current_boundary"}
                          for side in ("left", "right")}
                if valid:
                    result.update(deepcopy(sample["result"]))
            output[key] = {"fresh": valid, "frame_seq": sample["frame_seq"] if sample else None,
                           "age_ms": round((now-sample["received_mono"])*1000, 2) if sample else None,
                           **result}
        sample=self.channels['obstacles']
        fresh=bool(live and sample and 0<=now-sample['received_mono']<=self.aeb_config['ttl_s'])
        output['obstacles']={'level':'unknown','valid':False,'distance_m':None,'ttc_s':None,
            'reason':'no_fresh_pedestrian_result', **(deepcopy(sample['result']) if fresh else {}),
            'fresh':fresh,'frame_seq':sample['frame_seq'] if sample else None,
            'age_ms':round((now-sample['received_mono'])*1000,2) if sample else None}
        return output

    def _aeb(self, p):
        if not self.aeb_config.get('enabled',False):
            return {'level':'disabled','enabled':False,'distance_m':None,'ttc_s':None}
        result=deepcopy(p['obstacles'])
        result.update(enabled=True,mode=self.aeb_config.get('mode','video_demo'))
        if self.aeb_latched:
            result.update(level='emergency',reason='pedestrian_brake_hold')
        return result

    def _permission(self, action, p):
        if action == "stop":
            return True, "STOP_REQUEST", "停止请求", None
        if not p["source_valid"]:
            return False, "SOURCE_UNAVAILABLE", "视频源无效或已过期", None
        aeb=self._aeb(p)
        if aeb['level']=='emergency':
            return False,'PEDESTRIAN_AEB','前方行人危险，紧急制动展示',None
        scenario = self.context["scenario"]
        signal = p["signal"]
        red_only = self.config.get("red_only_stop", False)
        if red_only and signal["fresh"] and signal.get("valid") and signal["color"] == "red":
            return False, "RED_LIGHT_STOP", "红灯禁止通行", "RED_LIGHT_MOVE_REQUEST"
        if scenario == "intersection":
            if not red_only and (not signal["fresh"] or not signal.get("valid")):
                return False, "SIGNAL_UNKNOWN", "相关信号灯未知或已过期", None
            if not red_only and signal["color"] == "red":
                return False, "RED_LIGHT_STOP", "红灯禁止通行", "RED_LIGHT_MOVE_REQUEST"
            if not red_only and signal["color"] != "green":
                return False, "SIGNAL_WAIT", "当前不是已确认绿灯，等待通行", None
            if action != "move":
                return False, "USE_INTERSECTION_MOVE", "路口模式请先选车道，再提交行驶请求", None
            if signal["fresh"] and signal.get("valid") and signal["color"] == "green":
                return True, "GREEN_LIGHT_GO", "绿灯允许模拟通行", None
            return True, "NO_RED_LIGHT_GO", "未检测到当前红灯，正常模拟行驶", None
        if action == "move":
            return True, "KEEP_STRAIGHT", "保持直行", None
        if action not in ("left", "right"):
            return False, "INVALID_MANEUVER", "无效动作", None
        boundary = p["boundaries"][action]
        if not p["boundaries"]["fresh"] or not boundary.get("valid"):
            return False, "BOUNDARY_UNKNOWN", "请求侧边界未知或已过期，暂不变道", None
        if boundary["kind"] == "solid":
            return False, "SOLID_LINE_REJECT", ("左" if action == "left" else "右")+"侧实线，禁止变道", "SOLID_LINE_CHANGE_REQUEST"
        if boundary["kind"] != "dashed":
            return False, "BOUNDARY_UNKNOWN", "请求侧边界不确定，暂不变道", None
        return True, "DASHED_LINE_ALLOW", ("左" if action == "left" else "右")+"侧虚线，允许模拟变道", None

    def _decision(self, p, now):
        auto_clear = self.config.get("alarm_auto_clear_s")
        if self.alarm and auto_clear and now-self.alarm.get("raised_mono", now) >= auto_clear:
            self.alarm = None
            if not self.config.get("normal_driving_default"):
                self.intent, self.intent_at = "stop", None
            self._event("alarm_display_finished", automatic=True)
        permitted, code, text, _ = self._permission(self.intent, p)
        default_forward = self.config.get("normal_driving_default") and self.intent == "move"
        if self.intent != "stop" and not default_forward and (not permitted or self.intent_at is None or
                                       now-self.intent_at > self.config["command_lease_s"]):
            if permitted:
                code, text = "REQUEST_EXPIRED", "动作展示时限结束"
            if self.config.get("normal_driving_default"):
                self.intent, self.intent_at = "move", now
                permitted, code, text, _ = self._permission(self.intent, p)
            else:
                self.intent, self.intent_at = "stop", None
        action = self.intent if permitted else "stop"
        arrow = "none"
        if action != "stop":
            arrow = self.context["lane"] if self.context["scenario"] == "intersection" else (
                "straight" if action == "move" else action)
        if self.alarm is not None:
            action, arrow = "stop", "none"
            code, text = self.alarm["code"], self.alarm["message"]
        aeb=self._aeb(p)
        aeb_alarm=None
        if aeb['level']=='emergency':
            self._brake_intent()
            action,arrow='stop','none'
            code,text='PEDESTRIAN_AEB','前方有行人！紧急制动（静止演示）'
            aeb_alarm={'code':code,'message':text,'automatic':True,'source_epoch':self.epoch}
        elif aeb['level'] in ('warning','slow') and self.alarm is None:
            if action=='stop':
                text+='；前方有行人'+('，正在减速（模拟）' if aeb['level']=='slow' else '')
            elif aeb['level']=='slow':
                code,text='PEDESTRIAN_SLOW_DOWN','前方有行人，正在减速（模拟）'
                if action!='stop': action='slow'
            else:
                code,text='PEDESTRIAN_WARNING','前方有行人，请注意（静止演示）'
        stopped = action == "stop"
        return {"action": action, "arrow": arrow, "reason_code": code, "message": text,
                "alarm": aeb_alarm or deepcopy(self.alarm), "simulation_only": True,
                "aeb": {**aeb,'brake_pressure':1.0 if aeb['level']=='emergency' else .3 if aeb['level']=='slow' else 0.0},
                "feedback": {"turn": "hazard" if self.alarm or aeb_alarm else arrow if arrow in ("left", "right") else "off",
                             "brake_lamp": stopped or aeb['level']=='slow', "wheel": arrow if arrow in ("left", "right") else "center",
                             "body_locked": True}}

    def snapshot(self):
        with self.lock:
            now = self.clock()
            p = self._perception(now)
            d = self._decision(p, now)
            return {"schema": "rk_decision.v1", "source_epoch": self.epoch,
                    "context": {**self.context, "revision": self.context_revision},
                    "perception": p, "decision": d, "last_ack": deepcopy(self.last_ack),
                    "event_seq": self.event_seq}

    def command(self, command):
        with self.lock:
            if not isinstance(command, dict):
                raise CommandError("Expected JSON object")
            allowed_fields = {"client_id", "request_id", "seq", "source_epoch", "action", "scenario", "lane"}
            if set(command)-allowed_fields:
                raise CommandError("Unknown command fields")
            for key in ("client_id", "request_id"):
                if not isinstance(command.get(key), str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", command[key]):
                    raise CommandError("Invalid " + key)
            seq = command.get("seq")
            if type(seq) is not int or not 1 <= seq <= 2**53-1:
                raise CommandError("seq must be a positive integer")
            client, request = command["client_id"], command["request_id"]
            if command.get("source_epoch") != self.epoch:
                raise CommandError("source_epoch changed; fetch /snapshot and resubmit deliberately", 409)
            cache_key = (client, request)
            cached = self.command_cache.get(cache_key)
            if cached:
                if command != cached[0]:
                    raise CommandError("request_id reused with different content", 409)
                return {"ack": {**cached[1], "duplicate": True}, "snapshot": self.snapshot()}
            if seq <= self.client_sequences.get(client, 0):
                raise CommandError("Out-of-order command sequence", 409)
            action = command.get("action")
            if action not in ("move", "stop", "left", "right", "clear_alarm", "set_context"):
                raise CommandError("Invalid action")
            if action == "set_context":
                scenario, lane = command.get("scenario"), command.get("lane")
                if scenario not in ("intersection", "lane_change") or lane not in ("left", "straight", "right"):
                    raise CommandError("set_context requires scenario and lane")
            elif "scenario" in command or "lane" in command:
                raise CommandError("Use set_context to change scenario/lane")
            now = self.clock()
            accepted, code, message = True, "OK", "请求已处理"
            if action == "set_context":
                self.context = {"scenario": scenario, "lane": lane}
                self.context_revision += 1
                self.channels["signal"] = None
                if not self.config.get("normal_driving_default"):
                    self.intent, self.intent_at = "stop", None
                elif self.intent != "stop":
                    self.intent, self.intent_at = "move", now
                self.alarm = None
                code, message = "CONTEXT_CHANGED", "场景/车道已更新，等待新感知结果"
            elif action == "stop":
                self.intent, self.intent_at = "stop", None
                if self.config.get("alarm_auto_clear_s"):
                    self.alarm = None
                code, message = "STOP_REQUEST", "停止请求"
            elif action == "clear_alarm":
                self.alarm = None
                if not self.config.get("normal_driving_default"):
                    self.intent, self.intent_at = "stop", None
                code, message = "ALARM_ACKNOWLEDGED", "报警已确认，按当前规则重新判断"
            elif self.alarm and not self.config.get("alarm_auto_clear_s"):
                accepted, code, message = False, "ALARM_LATCHED", "请先确认报警，车辆保持停止"
            else:
                accepted, code, message, alarm_code = self._permission(action, self._perception(now))
                if accepted:
                    self.intent, self.intent_at = action, now
                elif not self.config.get("normal_driving_default"):
                    self.intent, self.intent_at = "stop", None
                if accepted and self.config.get("alarm_auto_clear_s"):
                    self.alarm = None
                if alarm_code:
                    self.alarm = {"code": alarm_code, "message": message+"，收到违规动作请求",
                                  "request_id": request, "requested_action": action,
                                  "source_epoch": self.epoch, "raised_mono": now}
            ack = {"request_id": request, "client_id": client, "seq": seq, "accepted": accepted,
                   "reason_code": code, "message": message, "duplicate": False}
            self.last_ack = ack
            self.client_sequences[client] = seq
            self.client_sequences.move_to_end(client)
            while len(self.client_sequences) > 64:
                self.client_sequences.popitem(last=False)
            self.command_cache[cache_key] = (deepcopy(command), deepcopy(ack))
            while len(self.command_cache) > 128:
                self.command_cache.popitem(last=False)
            self._event("command", **ack)
            return {"ack": deepcopy(ack), "snapshot": self.snapshot()}

    def events_after(self, seq):
        with self.lock:
            return {"events": [deepcopy(e) for e in self.events if e["event_seq"] > seq],
                    "latest_event_seq": self.event_seq,
                    "truncated": bool(self.events and seq < self.events[0]["event_seq"]-1)}
