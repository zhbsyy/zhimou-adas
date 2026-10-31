"""Short independent camera check; destroys only actors created by this run."""
import json
from pathlib import Path
import threading
import time

import carla
import cv2
import numpy as np


def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.)
    world = client.get_world()
    owned, frames, counts = [], {}, {}
    lock = threading.Lock()
    output = Path(__file__).resolve().parent/'Results'
    output.mkdir(exist_ok=True)
    started = time.monotonic()
    try:
        bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
        bp.set_attribute('role_name', 'carla_aeb_camera_probe')
        ego = world.try_spawn_actor(bp, carla.Transform(carla.Location(x=-4, y=24.7, z=.6), carla.Rotation(yaw=.2)))
        if ego is None:
            raise RuntimeError('probe spawn occupied; no existing actor was changed')
        owned.append(ego)
        ego.apply_control(carla.VehicleControl(hand_brake=True, brake=1.))
        for name, x, y, z, yaw in [('front',2.05,0,1.75,0),('right',0,1.15,1.65,90),
                                   ('rear',-2.05,0,1.75,180),('left',0,-1.15,1.65,-90)]:
            bp = world.get_blueprint_library().find('sensor.camera.rgb')
            for key, value in {'image_size_x':'640','image_size_y':'360','fov':'60',
                               'sensor_tick':'.05','enable_postprocess_effects':'false',
                               'role_name':'carla_aeb_probe_'+name}.items():
                bp.set_attribute(key, value)
            camera = world.spawn_actor(bp, carla.Transform(carla.Location(x=x,y=y,z=z),carla.Rotation(yaw=yaw)), attach_to=ego)
            owned.append(camera)
            def on_image(image, key=name):
                frame = np.frombuffer(image.raw_data,dtype=np.uint8).reshape(image.height,image.width,4)[:,:,:3].copy()
                with lock:
                    frames[key] = frame
                    counts[key] = counts.get(key,0)+1
            camera.listen(on_image)
            end = time.monotonic()+8
            while name not in frames and time.monotonic()<end:
                time.sleep(.05)
            if name not in frames:
                raise RuntimeError(name+' camera returned no image')
            print('CAMERA_READY', name, camera.id, flush=True)
        time.sleep(1.)
        with lock:
            image = np.vstack((np.hstack((frames['front'],frames['right'])),np.hstack((frames['rear'],frames['left']))))
            cv2.imwrite(str(output/'camera_probe.jpg'),image)
            result = {'passed':all(counts.get(x,0)>1 for x in ('front','right','rear','left')),
                      'counts':dict(counts),'elapsed_s':round(time.monotonic()-started,2),'image':str(output/'camera_probe.jpg')}
        (output/'camera_probe.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print('CAMERA_PROBE',json.dumps(result),flush=True)
    finally:
        for actor in reversed(owned):
            try:
                if actor.type_id.startswith('sensor.'):
                    actor.stop()
                actor.destroy()
            except Exception as exc:
                print('CLEANUP_ERROR',str(exc),flush=True)


if __name__=='__main__':
    main()
