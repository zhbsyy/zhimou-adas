"""Navigation intent and AEB stop/resume rules, independent of Qt and CARLA."""
from dataclasses import dataclass


@dataclass
class DriveIntent:
    gear: str = 'P'
    running: bool = False
    last_remote_gear: str | None = None
    last_remote_running: bool | None = None
    revision: int = 0

    def gear_request(self,gear):
        if gear not in ('P','N','D','R'):
            return False
        self.gear,self.running = gear,gear in ('D','R')
        self.revision += 1
        return True

    def run_request(self,running):
        self.running = bool(running)
        if running and self.gear not in ('D','R'):
            self.gear = 'D'
        self.revision += 1

    def pause(self):
        self.running = False
        self.gear = 'P'
        self.revision += 1

    def remote(self,frame):
        """Repeated Android status is a heartbeat, not a new resume command."""
        before=self.revision
        gear=frame.get('gear')
        run=frame.get('simulation_running')
        if isinstance(gear,str) and gear.upper() in ('P','N','D','R'):
            gear=gear.upper()
            if gear!=self.last_remote_gear:
                self.gear_request(gear)
            self.last_remote_gear=gear
        if type(run) is bool:
            if run!=self.last_remote_running:
                self.run_request(run)
            self.last_remote_running=run
        return self.revision!=before


class AebFlow:
    def __init__(self):
        self.phase='idle'
        self.actor=None
        self.clear_since=None
        self.clear_count=0
        self.last_clear_seq=None
        self.last_clear_at=0.
        self.fault=''
        self.has_run=False
        self.lag_since=None
        self.recovery_since=None
        self.recovery_seq=None
        self.recovery_count=0

    def reset_clear(self):
        self.clear_since,self.clear_count,self.last_clear_seq=None,0,None

    def explicit_command(self):
        self.fault=''
        self.has_run=False
        self.lag_since,self.recovery_since=None,None
        self.recovery_count=0

    def reset(self):
        self.__init__()

    def step(self,*,enabled,intent,reason,report,speed,now,transient=False):
        requested=intent.running and intent.gear in ('D','R')
        result={'drive':False,'brake':False,'phase':self.phase,'reason':'已暂停','resumed':False}
        if not enabled:
            self.phase='idle'
            self.reset_clear()
            self.actor=None
            self.lag_since,self.recovery_since=None,None
        evidence=report.get('evidence',{})
        if enabled and report.get('aeb_triggered') is True:
            self.phase='braking'
            self.actor=evidence.get('actor_id')
            self.reset_clear()
        if reason:
            if transient and enabled and requested and not self.fault:
                self.lag_since=now if self.lag_since is None else self.lag_since
                self.recovery_since,self.recovery_seq,self.recovery_count=None,None,0
                self.reset_clear()
                if now-self.lag_since<2.:
                    return dict(result,brake=True,phase='waiting_fresh',reason='感知短时等待：'+reason+'；保持制动')
            if self.has_run:
                self.fault=reason
            self.reset_clear()
            return dict(result,phase='fault' if self.fault else 'waiting',reason=self.fault or reason)
        if self.fault:
            return dict(result,phase='fault',reason=self.fault+'；请明确继续')
        if not requested:
            self.reset_clear()
            self.lag_since,self.recovery_since=None,None
            return dict(result,phase='paused',reason='手动暂停/驻车')
        if enabled and self.lag_since is not None:
            seq=report.get('det_report_seq')
            if seq!=self.recovery_seq:
                self.recovery_seq=seq
                self.recovery_count+=1
                self.recovery_since=now if self.recovery_since is None else self.recovery_since
            if self.recovery_count<3 or self.recovery_since is None or now-self.recovery_since<.5:
                return dict(result,brake=True,phase='recovering',reason='感知已恢复，确认连续新结果；保持制动')
            self.lag_since,self.recovery_since=None,None
            self.recovery_count=0
        if not enabled:
            self.has_run=True
            return dict(result,drive=True,phase='navigation',reason='按导航行驶（AEB关闭）')
        if self.phase in ('braking','waiting_clear'):
            if speed>.3:
                self.phase='braking'
                self.reset_clear()
            else:
                self.phase='waiting_clear'
                seq=report.get('det_report_seq')
                clear=(evidence.get('path_clear') is True and evidence.get('clear_actor_id')==self.actor
                       and self.actor is not None and not report.get('aeb_triggered'))
                if not clear or now-self.last_clear_at>.6:
                    self.reset_clear()
                if clear and seq!=self.last_clear_seq:
                    self.clear_since=now if self.clear_since is None else self.clear_since
                    self.clear_count+=1
                    self.last_clear_seq,self.last_clear_at=seq,now
                if self.clear_count>=5 and self.clear_since is not None and now-self.clear_since>=1.:
                    self.phase='idle'
                    self.reset_clear()
                    self.has_run=True
                    return dict(result,drive=True,phase='navigation',reason='行人已通过，恢复原导航',resumed=True)
            return dict(result,brake=True,phase=self.phase,
                reason='前方行人，紧急制动' if self.phase=='braking' else '车辆已停，确认行人通过')
        self.has_run=True
        return dict(result,drive=True,phase='navigation',reason='按导航行驶（AEB开启）')
