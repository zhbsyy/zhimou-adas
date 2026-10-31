"""One latest console with three startup profiles; control algorithms unchanged."""
from pathlib import Path
import argparse
import datetime
import json
import os
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from runtime_layout import materialize


def prepare(profile):
    sys.dont_write_bytecode=True
    pc=materialize('pc')
    cfg=json.loads((ROOT/'configs/pc.json').read_text(encoding='utf-8'))
    sys.path[:0]=[str(pc/'Console_Application'),str(pc/'Runtime_Support/CARLA_PythonAPI'),str(pc)]
    os.environ.update(CARLA_ROOT=cfg['carla_root'],
        ADAS_BOARD_STREAM_URL='http://%s:%s/stream'%(cfg['rk_ip'],cfg['four_camera_video_port'] if profile!='carla_navigation' else cfg['carla_video_port']),
        ADAS_RK_DECISION_URL='http://%s:%s'%(cfg['rk_ip'],cfg['decision_port']),
        ADAS_RK_DECISION_STREAM='http://%s:%s/stream'%(cfg['rk_ip'],cfg['decision_video_port']),
        ADAS_DECISION_RESULTS=str(ROOT/'results/pc/decision'))
    from bridge.serial_client import SerialClient
    from navigation_motion import preferred_cockpit_port,install_collision_monitor
    import serial.tools.list_ports
    SerialClient.find_preferred_port=classmethod(lambda cls:preferred_cockpit_port(serial.tools.list_ports.comports()))
    import bridge.arbiter
    from pedestrian_crossing import install as install_crossing
    install_crossing(bridge.arbiter.CarlaBackend);install_collision_monitor(bridge.arbiter.CarlaBackend)
    from navigation_arbiter import NavigationArbiter
    class ProfileArbiter(NavigationArbiter):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self._perception_source=self.state.perception_source='carla' if profile=='carla_navigation' else 'pcie'
    bridge.arbiter.SimulationArbiter=ProfileArbiter
    from ui.dashboard import DashboardWindow
    import navigation_session
    from navigation_session import install_console_view
    from video_bridge import install
    install(DashboardWindow);install_console_view(DashboardWindow)
    import video_bridge
    video_bridge.PCIE='http://%s:%s/stream'%(cfg['rk_ip'],cfg['four_camera_video_port'])
    video_bridge.PROCESSED='http://%s:%s/stream'%(cfg['rk_ip'],cfg['carla_video_port'])
    # NavigationSession uses ROOT only to locate its Results folder.
    navigation_session.ROOT=ROOT/'results/pc/navigation'
    import ui.rk_decision_panel as panel
    modules=[sys.modules[DashboardWindow.__module__],panel,video_bridge,navigation_session]
    assert all(Path(m.__file__).resolve().is_relative_to(pc.resolve()) for m in modules)
    return DashboardWindow,navigation_session.NavigationSession,{
        'profile':profile,'console_version':'navigation_reliability.v1.5','PC_panel_version':'stereo_console.v0.1.5',
        'RK_video':os.environ['ADAS_RK_DECISION_STREAM'] if profile=='decision' else os.environ['ADAS_BOARD_STREAM_URL'],
        'default_source':'carla' if profile=='carla_navigation' else 'pcie',
        'module_paths':{m.__name__:m.__file__ for m in modules},'CARLA_or_RK_started_in_check':False}


def main():
    p=argparse.ArgumentParser();p.add_argument('--profile',choices=('decision','four_camera','carla_navigation'),required=True);p.add_argument('--check',action='store_true');a=p.parse_args()
    Window,Session,info=prepare(a.profile)
    if a.check:print(json.dumps(info,ensure_ascii=False,indent=2));return
    logs=ROOT/'results/pc';logs.mkdir(parents=True,exist_ok=True)
    (logs/'navigation').mkdir(exist_ok=True)
    import msvcrt
    lock=open(logs/'console.lock','a+b')
    if lock.seek(0,2)==0:lock.write(b'0');lock.flush()
    lock.seek(0)
    try:msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    except OSError:return
    log=open(logs/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+a.profile+'.log'),'a',encoding='utf-8',buffering=1)
    sys.stdout=sys.stderr=log
    import faulthandler
    faulthandler.enable(file=log)
    print(json.dumps(info,ensure_ascii=False),flush=True)
    from PyQt5 import QtCore,QtWidgets
    app=QtWidgets.QApplication([])
    try:
        window=Window();session=Session(window)
    except Exception as error:
        import traceback
        traceback.print_exc()
        QtWidgets.QMessageBox.critical(None,'总控台启动未完成','先关闭其他总控台；8888和18090不能共用。\n'+str(error));return
    window.aeb_session=session
    original=window.closeEvent
    def close(event):session.close();original(event)
    window.closeEvent=close
    window.setWindowTitle(window.windowTitle()+' · Final submission 2 · '+a.profile)
    if a.profile=='decision':window._toggle_rk_decision()
    window.showMaximized()
    QtCore.QTimer.singleShot(350,lambda:(window.showMaximized(),window.raise_(),window.activateWindow()))
    app.exec_();lock.close()


if __name__=='__main__':main()
