"""Candidate-only camera attachment: actual frames required, partial failure cleaned."""
import threading
import time
import json
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

import carla
import cv2
import numpy as np
from bridge.carla_streamer import CarlaMultiViewStreamer


class ReliableStreamer(CarlaMultiViewStreamer):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.frame_counts = {}
        self.frame_received = {}
        self.attach_error = ''
        self.ready = False
        self._publisher = None
        self._published_at = 0.
        self._frame_ids = {}
        self._new_front = threading.Event()

    def attach_to_carla(self, carla_client=None):
        self.ready = False
        self.attach_error = ''
        try:
            self.carla_client = carla_client or carla.Client(self.host, self.port)
            self.carla_client.set_timeout(10.)
            world = self.carla_client.get_world()
            if world.get_settings().no_rendering_mode:
                raise RuntimeError('CARLA no_rendering_mode is enabled; RGB cameras cannot work')
            vehicles = [actor for actor in world.get_actors().filter('vehicle.*')
                        if actor.attributes.get('role_name') == 'ego_vehicle']
            if len(vehicles) != 1:
                raise RuntimeError('expected one ego_vehicle, found '+str(len(vehicles)))
            ego = vehicles[0]
            views = [('front',2.05,0.,1.75,0.),('right',0.,1.15,1.65,90.),
                     ('rear',-2.05,0.,1.75,180.),('left',0.,-1.15,1.65,-90.)]
            # Only cameras belonging to this isolated AEB session are replaced.
            for actor in world.get_actors().filter('sensor.camera.rgb'):
                if actor.attributes.get('role_name', '').startswith('carla_aeb_trial_'):
                    actor.destroy()
            for name, x, y, z, yaw in views:
                bp = world.get_blueprint_library().find('sensor.camera.rgb')
                for key, value in {'image_size_x':str(self.tile_w), 'image_size_y':str(self.tile_h),
                                   'fov':'60', 'sensor_tick':'.05',
                                   'role_name':'carla_aeb_trial_'+name}.items():
                    bp.set_attribute(key, value)
                cam = world.spawn_actor(bp, carla.Transform(carla.Location(x=x,y=y,z=z),carla.Rotation(yaw=yaw)),attach_to=ego)
                self.carla_cameras.append(cam)
                def callback(image, key=name):
                    bgr = np.frombuffer(image.raw_data,dtype=np.uint8).reshape(image.height,image.width,4)[:,:,:3].copy()
                    bgr.flags.writeable = False
                    with self._lock:
                        self._frames[key] = bgr
                        self.frame_received[key] = time.monotonic()
                        self.frame_counts[key] = self.frame_counts.get(key,0)+1
                        self._frame_ids[key] = image.frame
                    if key=='front':
                        self._new_front.set()
                cam.listen(callback)
                deadline = time.monotonic()+8
                while not self.frame_counts.get(name,0) and time.monotonic()<deadline:
                    time.sleep(.05)
                if not self.frame_counts.get(name,0):
                    raise RuntimeError(name+' camera has no image within 8 seconds')
                print('[AEB camera ready]',name,cam.id,flush=True)
            self.ready = True
            return True
        except Exception as exc:
            self.attach_error = str(exc)
            print('[AEB camera attach failed]',repr(exc),flush=True)
            self.stop()
            return False

    def health(self):
        now = time.monotonic()
        with self._lock:
            counts = dict(self.frame_counts)
            ages = {key:round((now-value)*1000,1) for key,value in self.frame_received.items()}
            frame_ids = dict(self._frame_ids)
        camera_ready = self.ready and len(ages)==4 and max(ages.values(),default=9999)<1000
        publisher_ready = self._publisher is not None and self._publisher.is_alive() and now-self._published_at<1.
        return {'ready':camera_ready and publisher_ready,'camera_ready':camera_ready,
                'counts':counts,'age_ms':ages,'carla_frames':frame_ids,'error':self.attach_error,
                'publisher_ready':publisher_ready,'seq':self._frame_seq,'fps':round(self._fps,1)}

    def get_latest_mosaic(self):
        # Callbacks replace owned immutable arrays. Snapshot references once;
        # copying each whole tile again before composing is unnecessary.
        with self._lock:
            frames = [self._frames.get(key) for key in ('front','right','rear','left')]
        if any(frame is None for frame in frames):
            return None
        canvas = np.empty((self.height,self.width,3),dtype=np.uint8)
        canvas[:self.tile_h,:self.tile_w] = frames[0]
        canvas[:self.tile_h,self.tile_w:] = frames[1]
        canvas[self.tile_h:,:self.tile_w] = frames[2]
        canvas[self.tile_h:,self.tile_w:] = frames[3]
        for label,x,y in [('FRONT',10,24),('RIGHT',self.tile_w+10,24),
                          ('REAR',10,self.tile_h+24),('LEFT',self.tile_w+10,self.tile_h+24)]:
            cv2.putText(canvas,label,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
        return canvas

    def _publish_loop(self):
        last_front = -1
        while self._running:
            arrived = self._new_front.wait(.1)
            self._new_front.clear()
            if not self._running:
                break
            front = self.frame_counts.get('front',0)
            if arrived and self.health()['camera_ready'] and front!=last_front:
                mosaic = self.get_latest_mosaic()
                ok,encoded = cv2.imencode('.jpg',mosaic,[cv2.IMWRITE_JPEG_QUALITY,self.quality])
                if ok:
                    now = time.monotonic()
                    with self._lock:
                        self._latest_jpeg = encoded.tobytes()
                        self._frame_seq += 1
                        if self._published_at:
                            self._fps = .9*self._fps+.1/max(now-self._published_at,1e-6)
                        self._published_at = now
                    last_front = front
            elif not self.health()['camera_ready']:
                with self._lock:
                    self._latest_jpeg = None

    def start_http_server(self):
        if self._running:
            return
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):
                pass
            def do_GET(self):
                self.connection.settimeout(2.)
                if self.path=='/status':
                    health = owner.health()
                    body = json.dumps(dict(health,cameras=len(owner.carla_cameras))).encode()
                    self.send_response(200)
                    self.send_header('Content-Type','application/json')
                    self.send_header('Content-Length',str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path in ('/frame','/snapshot.jpg'):
                    data = owner._latest_jpeg
                    if not owner.health()['ready'] or not data:
                        self.send_error(503,'Camera frames not ready')
                        return
                    self.send_response(200)
                    self.send_header('Content-Type','image/jpeg')
                    self.send_header('Content-Length',str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if self.path not in ('/','/stream'):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Cache-Control','no-store')
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=FRAME')
                self.end_headers()
                last = -1
                try:
                    while owner._running:
                        with owner._lock:
                            data,seq = owner._latest_jpeg,owner._frame_seq
                        if data and seq!=last:
                            self.wfile.write(b'--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n')
                            last = seq
                        time.sleep(.005)
                except (OSError,TimeoutError):
                    pass
        # Bind first. A busy port must not leave an orphan producer thread.
        self._http_server = ThreadingHTTPServer(('0.0.0.0',self.http_port),Handler)
        self._running = True
        self._publisher = threading.Thread(target=self._publish_loop,daemon=True)
        self._publisher.start()
        self._new_front.set()
        self._http_thread = threading.Thread(target=lambda:self._http_server.serve_forever(poll_interval=.1),daemon=True)
        self._http_thread.start()

    def stop(self):
        self.ready = False
        self._new_front.set()
        super().stop()
        if self._publisher:
            self._publisher.join(1.)
        with self._lock:
            self._frames.clear()
            self.frame_received.clear()
            self.frame_counts.clear()
            self._frame_ids.clear()
            self._latest_jpeg = None
