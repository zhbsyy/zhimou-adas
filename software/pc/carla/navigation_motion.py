"""PC simulation speed planning; independent of RK's visual AEB decision."""
import math
import time


def speed_plan(backend, enabled):
    if not enabled:
        return None, ''
    cap, reason = 40., '正常路段按座舱目标/道路限速行驶'
    path = backend.trajectory_waypoints
    i = min(getattr(backend, 'last_matched_idx', 0), len(path)-1)
    if path and i >= 0:
        cached=getattr(backend,'_motion_route_cache',(None,None))
        if cached[0] is not path:
            arc=[0.]
            for a,b in zip(path,path[1:]):
                arc.append(arc[-1]+a.transform.location.distance(b.transform.location))
            backend._motion_route_cache=(path,arc)
        else:
            arc=cached[1]
        for j in range(i,len(path)):
            distance=arc[j]-arc[i]
            if distance>65:break
            k=j
            while k<len(path)-1 and arc[k]-arc[j]<12:k+=1
            angle=abs((path[k].transform.rotation.yaw-path[j].transform.rotation.yaw+180)%360-180)
            if angle>=18:
                # Straight intersections do not imply a turn. The stopping
                # envelope reaches 15km/h before the actual curve.
                bend_cap=3.6*math.sqrt((15./3.6)**2+2*1.5*max(0.,distance-4.))
                if bend_cap<cap:
                    cap,reason=bend_cap,'弯道前渐进减速，弯中上限15km/h'
                break
    site = getattr(backend, '_scene_site', None)
    if site and not backend._scene_crossing_done:
        # Route arc length prevents a parallel street or bend from hiding the
        # braking approach. Never use this geometry to assert visual AEB.
        arc = site.get('route_arc', ())
        remaining = site.get('route_s', float('inf'))-(arc[i] if arc and 0 <= i < len(arc) else 0.)
        approach_cap=3.6*math.sqrt((12./3.6)**2+2*1.5*max(0.,remaining-18.))
        if approach_cap<cap:
            cap=approach_cap
            reason='横穿路段前渐进减速，接近时上限12km/h'
    return cap, reason


def longitudinal_command(actual, target, dt, throttle, brake):
    """Immediate throttle cut on overspeed; pedal ramps use elapsed seconds."""
    dt = max(.005, min(.2, dt))
    error = target-actual
    if target <= .5:
        wanted_throttle, wanted_brake = 0., min(1., .4+actual*.04)
    elif error < -.6:
        wanted_throttle, wanted_brake = 0., min(.7, .10+(-error-.6)*.07)
    elif error > .2:
        wanted_throttle = min(.5, .10+actual/250.+error*.035)
        wanted_brake = 0.
    else:
        wanted_throttle, wanted_brake = 0., 0.
    if wanted_brake > 0:
        return 0., min(wanted_brake, brake+2.5*dt)
    # Release the brake before accelerating; never wait for residual throttle
    # to ramp down before accepting a brake demand.
    if brake > .01:
        return 0., max(0., brake-3.*dt)
    if wanted_throttle <= throttle:
        return wanted_throttle, 0.
    return min(wanted_throttle, throttle+.55*dt), 0.


def preferred_cockpit_port(ports):
    # Explicit user hardware. No fallback to FTDI FPGA cables or a virtual UART.
    for port in ports:
        if port.device.upper() == 'COM12' and 'CH340' in (port.description or '').upper():
            return port.device
    return None


def install_collision_monitor(backend_class):
    if getattr(backend_class, '_collision_monitor_installed', False):
        return
    original_spawn = backend_class._spawn_vehicle
    original_reset = backend_class.reset_to_default_spawn
    original_disconnect = backend_class.disconnect

    def stop_sensor(backend):
        sensor = getattr(backend, '_navigation_collision_sensor', None)
        backend._navigation_collision_sensor = None
        if sensor is not None:
            sensor.stop()
            sensor.destroy()

    def spawn(backend):
        stop_sensor(backend)
        original_spawn(backend)
        backend._navigation_collisions = []
        import carla
        bp = backend.world.get_blueprint_library().find('sensor.other.collision')
        sensor = backend.world.spawn_actor(bp, carla.Transform(), attach_to=backend.vehicle)
        backend._navigation_collision_sensor = sensor
        def collision(event):
            other = event.other_actor
            v = event.normal_impulse
            impulse = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            if impulse < 1. and not other.type_id.startswith('walker.'):
                return
            backend._navigation_collisions.append({'frame':event.frame, 'actor_id':other.id,
                'type':other.type_id, 'impulse':round(impulse, 3), 'at':time.monotonic()})
            backend._navigation_collisions[:] = backend._navigation_collisions[-30:]
        sensor.listen(collision)

    def reset(backend):
        original_reset(backend)
        backend._navigation_collisions = []

    def disconnect(backend):
        stop_sensor(backend)
        original_disconnect(backend)

    backend_class._spawn_vehicle = spawn
    backend_class.reset_to_default_spawn = reset
    backend_class.disconnect = disconnect
    backend_class._collision_monitor_installed = True
