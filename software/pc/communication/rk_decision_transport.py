"""PC transport and freshness guard; rule decisions are supplied only by RK."""
from __future__ import annotations

import copy
import json
import math
import queue
import threading
import time
import urllib.error
import urllib.request
import uuid


def display_feedback(snapshot, local_age, max_age=.8):
    """Validate the RK envelope and bound local replay age, without inferring rules."""
    stop = {'turn':'off','brake_lamp':True,'wheel':'center','body_locked':True,'reaction':'hold'}
    if not isinstance(local_age,(int,float)) or not math.isfinite(local_age) or not 0 <= local_age <= max_age:
        return stop, 'RK结果中断，保持停止'
    if not isinstance(snapshot,dict) or snapshot.get('schema') != 'rk_decision.v1':
        return stop, 'RK返回格式不匹配，保持停止'
    p, d = snapshot.get('perception',{}), snapshot.get('decision',{})
    if p.get('evidence_mode') != 'live':
        return stop, '非实际感知结果，车辆反馈已阻止'
    if not p.get('source_valid'):
        return stop, 'RK图像源无效，保持停止'
    source_age = p.get('source_age_ms')
    if (type(source_age) not in (int,float) or not math.isfinite(source_age) or
            source_age < 0 or source_age+local_age*1000 > 1000):
        return stop, 'RK图像源过期，保持停止'
    f = d.get('feedback',{})
    if (d.get('simulation_only') is not True or f.get('body_locked') is not True or
            f.get('turn') not in ('off','left','right','hazard') or
            f.get('wheel') not in ('center','left','right') or type(f.get('brake_lamp')) is not bool):
        return stop, 'RK反馈字段无效，保持停止'
    expected = {'off':'center','left':'left','right':'right','hazard':'center'}[f['turn']]
    if f['wheel'] != expected:
        return stop, 'RK转向反馈字段冲突，保持停止'
    result=dict(f)
    aeb=d.get('aeb',{})
    result['reaction']='normal'
    if aeb.get('level')=='emergency':
        if d.get('action')!='stop' or f['turn'] not in ('off','hazard') or not f['brake_lamp']:
            return stop,'RK行人制动反馈字段冲突，保持停止'
        result['aeb_emergency']=True
        result.update(turn='off',wheel='center',brake_lamp=True,reaction='emergency')
    else:
        # Rule alarms request braking; they do not imply a vehicle breakdown.
        if result['turn']=='hazard':result.update(turn='off',wheel='center')
        if d.get('alarm'):
            result.update(brake_lamp=True,reaction='alarm')
        else:
            # Use the state in the accepted RK envelope, as the status banner
            # does. Do not briefly extinguish brake lamps on a summary TTL edge.
            if aeb.get('level') in ('warning','slow'):
                result.update(brake_lamp=True,reaction=aeb['level'])
            elif d.get('action')=='stop' and d.get('reason_code')=='STOP_REQUEST':
                ack=snapshot.get('last_ack') or {}
                explicit_stop=ack.get('accepted') is True and ack.get('reason_code')=='STOP_REQUEST'
                result.update(brake_lamp=explicit_stop,reaction='manual_stop' if explicit_stop else 'idle')
    # Missing pedestrian/range evidence is diagnostic, not an extra stop rule.
    # The existing source/transport guards and confirmed emergency remain above.
    return result, None


class DecisionTransport:
    """One bounded command slot, 5Hz polling, all network IO outside Qt thread."""
    def __init__(self, url, token=None, timeout=.8):
        self.url = url.rstrip('/')
        self.timeout, self.token = timeout, token
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.client_id = 'console_'+uuid.uuid4().hex[:16]
        self.seq = 0
        self.commands = queue.Queue(maxsize=1)
        self.events = queue.Queue(maxsize=32)
        self.lock = threading.Lock()
        self.snapshot = None
        self.received = 0.0
        self.error = '正在连接RK'
        self.pending = False
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run,daemon=True)

    def start(self):
        self.thread.start()

    def state(self):
        with self.lock:
            return self.snapshot, self.received, self.error, self.pending

    def request(self, action, epoch, scenario=None, lane=None):
        with self.lock:
            if self.pending or self.stop_event.is_set():
                raise ValueError('上一请求尚未完成')
            if not epoch:
                raise ValueError('请先等待RK连接')
            self.seq += 1
            command = {'client_id':self.client_id, 'request_id':uuid.uuid4().hex,
                       'seq':self.seq, 'source_epoch':epoch, 'action':action}
            if action == 'set_context':
                command.update(scenario=scenario,lane=lane)
            self.commands.put_nowait((command,time.monotonic()))
            self.pending = True
        return command

    def _event(self, value):
        try:
            self.events.put_nowait(value)
        except queue.Full:
            self.events.get_nowait()
            self.events.put_nowait(value)

    def _fetch(self, command=None):
        if command is None:
            req = urllib.request.Request(self.url+'/snapshot')
        else:
            headers = {'Content-Type':'application/json'}
            if self.token:
                headers['X-Decision-Token'] = self.token
            req = urllib.request.Request(self.url+'/command',data=json.dumps(command).encode(),
                                         headers=headers,method='POST')
        with self.opener.open(req,timeout=self.timeout) as response:
            payload = response.read(131073)
            if len(payload) > 131072:
                raise ValueError('RK回复过大')
            return json.loads(payload)

    def _save(self, snapshot):
        if not isinstance(snapshot,dict) or snapshot.get('schema') != 'rk_decision.v1':
            raise ValueError('RK回复schema不匹配')
        with self.lock:
            self.snapshot = snapshot
            self.received = time.monotonic()
            self.error = None

    def _run(self):
        while not self.stop_event.is_set():
            command = None
            try:
                command, created = self.commands.get(timeout=.15)
            except queue.Empty:
                pass
            try:
                if command is not None:
                    if time.monotonic()-created > .8:
                        raise ValueError('点击等待过久，已取消；请查看新结果后再次点击')
                    reply = self._fetch(command)
                    self._save(reply['snapshot'])
                    self._event({'command':command,'ack':reply['ack']})
                else:
                    self._save(self._fetch())
            except Exception as exc:
                message = str(exc)[:180]
                if isinstance(exc,urllib.error.HTTPError):
                    try:
                        message = exc.read(4096).decode('utf-8')[:180]
                    except Exception:
                        pass
                with self.lock:
                    self.error = message
                    self.received = 0.0
                if command is not None:
                    self._event({'command':command,'error':message})
            finally:
                if command is not None:
                    with self.lock:
                        self.pending = False
            self.stop_event.wait(.03)

    def close(self):
        self.stop_event.set()


class MjpegInput:
    """Latest decoded QImage only. No unbounded Qt signal queue or frame backlog."""
    def __init__(self,url):
        self.url = url
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest, self.seq, self.received = None,0,0.0
        self.error = '正在连接视频'
        self.response = None
        self.thread = threading.Thread(target=self._run,daemon=True)

    def start(self):
        self.thread.start()

    def state(self):
        with self.lock:
            return self.latest,self.seq,self.received,self.error

    def _run(self):
        from PyQt5.QtGui import QImage
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while not self.stop_event.is_set():
            try:
                with opener.open(self.url,timeout=1.2) as response:
                    self.response=response
                    buf=b''
                    while not self.stop_event.is_set():
                        chunk=response.read1(16384)
                        if not chunk:
                            raise RuntimeError('视频流已结束')
                        buf+=chunk
                        if len(buf)>3*1024*1024:
                            raise ValueError('视频帧缓冲超限')
                        while True:
                            a=buf.find(b'\xff\xd8')
                            b=buf.find(b'\xff\xd9',a+2) if a>=0 else -1
                            if b<0:
                                break
                            image=QImage.fromData(buf[a:b+2], 'JPG')
                            buf=buf[b+2:]
                            if image.isNull():
                                continue
                            with self.lock:
                                self.latest=image
                                self.seq+=1
                                self.received=time.monotonic()
                                self.error=None
            except Exception as exc:
                with self.lock:
                    self.error=str(exc)[:100]
                self.stop_event.wait(.8)
            finally:
                self.response=None

    def close(self):
        self.stop_event.set()
