"""Validated short crossings in the current route's frame; no teleport-through-obstacle fallback."""
import math
import time

import carla


def angle_difference(a,b):
    return abs((a-b+180.)%360.-180.)


def route_candidates(backend,preferred,automatic=True):
    path=backend.trajectory_waypoints
    if not path:
        return []
    arc=route_arc(path)
    if automatic:
        wanted=min(range(len(path)),key=lambda i:abs(arc[i]-.5*arc[-1]))
    else:
        wanted=min(range(len(path)),key=lambda i:path[i].transform.location.distance(preferred.transform.location))
    indices=sorted(range(max(0,backend.last_matched_idx+7),max(0,len(path)-3)),key=lambda i:abs(i-wanted))
    result=[]
    for i in indices:
        wp=path[i]
        if automatic and not arc[-1]/3<=arc[i]<=2*arc[-1]/3:
            continue
        if not 2.5<=wp.lane_width<=5.5:
            continue
        # Need time to finish the turn, slow down and obtain multiple RK
        # detections. Three neighboring points (~5m) were not enough.
        before=i
        while before>0 and arc[i]-arc[before]<14:
            before-=1
        after=i
        while after<len(path)-1 and arc[after]-arc[i]<7:
            after+=1
        if arc[i]-arc[before]<12.5 or arc[after]-arc[i]<6:
            continue
        near=path[before:after+1]
        yaw=math.radians(wp.transform.rotation.yaw)
        center=wp.transform.location
        if any(angle_difference(p.transform.rotation.yaw,wp.transform.rotation.yaw)>10 or
               abs((p.transform.location.x-center.x)*math.sin(yaw)-(p.transform.location.y-center.y)*math.cos(yaw))>.65
               for p in near):
            continue
        if arc[i]-arc[backend.last_matched_idx]<18:
            continue
        if all(abs(i-j)>=3 for j,_ in result):
            result.append((i,wp))
        if len(result)>=12:
            break
    return result


def route_arc(path):
    result=[0.]
    for a,b in zip(path,path[1:]):
        result.append(result[-1]+a.transform.location.distance(b.transform.location))
    return result


def crossing_geometry(world,wp):
    reason='no_flat_corridor'
    # Shorten the crossing before a kerb rather than asking the capsule to
    # climb it. Endpoints still clear the ego lane with a pedestrian margin.
    for span in (max(2.4,min(3.3,wp.lane_width/2+.6)),2.2,2.0,1.9):
        site,reason=_geometry_for_span(world,wp,span)
        if site is not None:
            return site,''
    return None,reason


def _geometry_for_span(world,wp,span):
    center=wp.transform.location
    yaw=math.radians(wp.transform.rotation.yaw)
    right=(-math.sin(yaw),math.cos(yaw))
    forward=(math.cos(yaw),math.sin(yaw))
    start=carla.Location(center.x+right[0]*span,center.y+right[1]*span,center.z)
    target=carla.Location(center.x-right[0]*span,center.y-right[1]*span,center.z)
    allowed=(carla.CityObjectLabel.Roads,carla.CityObjectLabel.RoadLines,carla.CityObjectLabel.Sidewalks)
    # Check the full capsule-width corridor at torso and leg height, not only endpoints.
    for longitudinal in (-.35,0.,.35):
        for height in (.10,.45,1.2):
            a=carla.Location(start.x+forward[0]*longitudinal,start.y+forward[1]*longitudinal,center.z+height)
            b=carla.Location(target.x+forward[0]*longitudinal,target.y+forward[1]*longitudinal,center.z+height)
            if world.cast_ray(a,b):
                return None,'obstacle_in_crossing'
    points=[]
    for fraction in (0.,.25,.5,.75,1.):
        p=carla.Location(start.x+(target.x-start.x)*fraction,start.y+(target.y-start.y)*fraction,center.z+2.)
        ground=world.ground_projection(p,3.)
        if ground is None or ground.label not in allowed or abs(ground.location.z-center.z)>.08:
            return None,'ground_or_kerb_not_suitable'
        points.append(ground.location)
    return {'start':points[0],'target':points[-1],'center':center,'right':right,'span':span,
            'road_yaw':wp.transform.rotation.yaw,'walk_speed':1.0},''


def should_start_crossing(site,pose,target,speed_kmh,front_extent):
    """Finite entry window after the car has slowed and aligned with the lane."""
    if not 7.<=speed_kmh<=13.5 or angle_difference(pose.rotation.yaw,site['road_yaw'])>12:
        return False
    center=site.get('center',target)
    yaw=math.radians(pose.rotation.yaw)
    dx,dy=center.x-pose.location.x,center.y-pose.location.y
    forward=dx*math.cos(yaw)+dy*math.sin(yaw)-front_extent
    lateral=dx*math.sin(yaw)-dy*math.cos(yaw)
    return 9.<=forward<=10.5 and abs(lateral)<=.85


def spawn_crossing(backend,preferred,automatic=True):
    backend._clear_pedestrian_scene()
    backend._scene_site=None
    backend._scene_blocked=False
    checks=[]
    blueprints=backend.world.get_blueprint_library().filter('walker.pedestrian.*')
    if not blueprints:
        return False
    bp=blueprints[0]
    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name','navigation_validated_crossing')
    for index,wp in route_candidates(backend,preferred,automatic):
        site,reason=crossing_geometry(backend.world,wp)
        checks.append({'route_index':index,'reason':reason or 'clear'})
        if site is None:
            continue
        start,target=site['start'],site['target']
        yaw=math.degrees(math.atan2(target.y-start.y,target.x-start.x))
        walker=None
        for lift in (.8,1.,1.2):
            walker=backend.world.try_spawn_actor(bp,carla.Transform(carla.Location(start.x,start.y,start.z+lift),carla.Rotation(yaw=yaw)))
            if walker is not None:
                break
        if walker is None:
            checks[-1]['reason']='spawn_collision'
            continue
        clear_lateral=max(1.7,backend.vehicle.bounding_box.extent.y+walker.bounding_box.extent.y+.35)
        if site['span']-.2<clear_lateral:
            walker.destroy()
            checks[-1]['reason']='insufficient_vehicle_clearance'
            continue
        site['clear_lateral']=clear_lateral
        site['route_index']=index
        site['route_arc']=route_arc(backend.trajectory_waypoints)
        site['route_s']=site['route_arc'][index]
        site['route_fraction']=site['route_s']/site['route_arc'][-1]
        backend._scene_site=site
        backend._scene_walker=walker
        backend._scene_walk_dir=carla.Vector3D(-site['right'][0],-site['right'][1],0.)
        backend._scene_target=target
        backend._scene_crossing_started=False
        backend._scene_crossing_done=False
        backend._scene_ready_since=None
        backend._scene_entry={}
        backend._scene_missed=False
        backend._scene_move_probe=(time.monotonic(),walker.get_location())
        backend._scene_checks=checks
        backend._scene_log('【动态行人】已选择路线 %.0f%% 位置并检查横穿走廊；路线点 %d，横穿 %.1fm'%(100*site['route_fraction'],index,site['span']*2))
        return True
    backend._scene_checks=checks
    backend._scene_log('【动态行人】路线中段没有适合的横穿位置；未退回起点布置，本轮不计为AEB验收通过')
    return False


def update_crossing(backend,state,dist_traveled,actual_v_kmh):
    del dist_traveled
    walker=backend._scene_walker
    site=getattr(backend,'_scene_site',None)
    if walker is None or site is None:
        state.pedestrian_detected=False
        return
    now=time.monotonic()
    snapshot=backend.world.get_snapshot()
    walker_state,vehicle_state=snapshot.find(walker.id),snapshot.find(backend.vehicle.id)
    if walker_state is None or vehicle_state is None:
        return
    target=walker_state.get_transform().location
    pose=vehicle_state.get_transform()
    yaw=math.radians(pose.rotation.yaw)
    dx,dy=target.x-pose.location.x,target.y-pose.location.y
    forward=dx*math.cos(yaw)+dy*math.sin(yaw)
    lateral=dx*math.sin(yaw)-dy*math.cos(yaw)
    state.pedestrian_distance_m=math.hypot(dx,dy)
    state.pedestrian_lateral_m=lateral
    state.pedestrian_detected=0<forward<30 and abs(lateral)<2.2
    if not backend._scene_crossing_started:
        eligible=state.simulation_running and state.gear=='D' and should_start_crossing(
            site,pose,target,actual_v_kmh,backend.vehicle.bounding_box.extent.x)
        # Stable alignment/speed must precede the window. Entering the last
        # few metres at high speed must never launch a pedestrian.
        aligned=7<=actual_v_kmh<=13.5 and angle_difference(pose.rotation.yaw,site['road_yaw'])<=12
        if not aligned:
            backend._scene_ready_since=None
        elif backend._scene_ready_since is None:
            backend._scene_ready_since=now
        if eligible and backend._scene_ready_since is not None and now-backend._scene_ready_since>=.35:
            backend._scene_crossing_started=True
            backend._scene_entry={'speed_kmh':round(actual_v_kmh,3),'forward_gap_m':round(forward-backend.vehicle.bounding_box.extent.x,3),
                                  'carla_frame':snapshot.frame,'route_index':site['route_index'],'route_fraction':round(site['route_fraction'],4)}
            backend._scene_move_probe=(now,target)
            walker.apply_control(carla.WalkerControl(direction=backend._scene_walk_dir,speed=site['walk_speed']))
            backend._scene_log('【动态行人】低速且对准车道，开始横穿：'+str(backend._scene_entry))
        elif forward-backend.vehicle.bounding_box.extent.x<9 and aligned and not backend._scene_missed:
            backend._scene_missed=True
            backend._scene_log('【动态行人】已错过安全入场窗口；本轮不突然横穿，不记为AEB通过')
        return
    # Completion is measured in the fixed crossing frame, independent of car turns.
    remaining=(target.x-site['target'].x)*site['right'][0]+(target.y-site['target'].y)*site['right'][1]
    if remaining<=.2:
        walker.apply_control(carla.WalkerControl(speed=0.))
        backend._scene_crossing_done=True
        backend._scene_walker=None
        state.pedestrian_detected=False
        walker.destroy()
        backend._scene_log('【动态行人】已通过目标车道并到达横穿终点')
        return
    probe_at,probe_pos=backend._scene_move_probe
    if now-probe_at>=2.5:
        backend._scene_move_probe=(now,target)
        if target.distance(probe_pos)<.15:
            if not backend._scene_blocked:
                backend._scene_log('【动态行人】检测到受阻，未标记通过；请在驻停后重新生成')
            backend._scene_blocked=True


def install(backend_class):
    if getattr(backend_class,'_validated_crossing_installed',False):
        return
    original_clear=backend_class._clear_pedestrian_scene
    def clear(backend):
        backend._scene_revision=getattr(backend,'_scene_revision',0)+1
        backend._scene_site=None
        backend._scene_blocked=False
        backend._scene_ready_since=None
        backend._scene_entry={}
        backend._scene_missed=False
        original_clear(backend)
    backend_class._clear_pedestrian_scene=clear
    backend_class._spawn_dynamic_pedestrian=spawn_crossing
    backend_class._update_pedestrian_scene=update_crossing
    backend_class._validated_crossing_installed=True
