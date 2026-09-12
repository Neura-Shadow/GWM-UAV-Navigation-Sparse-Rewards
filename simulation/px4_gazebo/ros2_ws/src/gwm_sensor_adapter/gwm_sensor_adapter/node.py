"""Guarded ROS transport. No flight publishers, truth input, or planner."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import time
import numpy as np
from .depth import decode, classify, Reason
from .geometry import Calibration, OPTICAL_FRAME, EXTRINSIC_ID, optical_to_body
from .state import Health, NewestSlot, StateHistory, ProcessingSchedule
from .recording import RawRecorder
from .contracts import load_config, run_identity


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    for gate in ('GWM_ALLOW_OPTIONAL_RUNTIME','GWM_RUN_GAZEBO_PX4_TESTS','GWM_ALLOW_PX4_LAUNCH','GWM_P3_OWNED_NAMESPACE'):
        if os.environ.get(gate) != '1': raise ValueError('Missing '+gate)
    config = load_config(args.config)
    actual_run=run_identity(args.run_dir,args.run_id)
    contract=config['sample_evidence_contract']
    def metadata(value): return dict(run_id=actual_run,sample_evidence_contract=contract,**value)
    def write_metadata(name,value):
        path=args.run_dir/name
        temporary=path.with_suffix(path.suffix+'.tmp')
        with temporary.open('x') as stream:
            stream.write(json.dumps(metadata(value),allow_nan=False,indent=2)+'\n')
            stream.flush(); os.fsync(stream.fileno())
        os.link(temporary,path); temporary.unlink()
    write_metadata('sensor-startup.json',dict(config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
        adapter_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        recording_directory=str(args.run_dir),startup_monotonic_ns=time.monotonic_ns()))
    if os.environ.get('ROS_DOMAIN_ID') != '71' or {name for _,name in socket.if_nameindex()} != {'lo'}:
        raise ValueError('owned_private_domain_required')
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    from rosidl_runtime_py.convert import message_to_ordereddict
    qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

    class Adapter(Node):
        def __init__(self):
            super().__init__('p3_sensor_adapter', namespace='/gwm')
            self.set_parameters([Parameter('use_sim_time', value=True)])
            self.health = Health(config['source_age_sim_s'], config['stall_wall_s'])
            self.slot, self.history = NewestSlot(), StateHistory(config['state_capacity'], config['state_mismatch_s'])
            self.recorder = RawRecorder(args.run_dir, config['recorder_capacity'])
            self.calibration = None
            self.calibration_start = None
            self.received = self.processed = self.repeated = 0
            self.info_count = 0
            self.position_callbacks = 0
            self.info_first_sim = self.info_last_sim = None
            self.schedule = ProcessingSchedule(config['processing_hz'])
            self.last_health_wall = -float('inf')
            self.first_source = None
            self.errors = {}
            self.events = (args.run_dir/'sensor-events.jsonl').open('x', buffering=1)
            self.obs = self.create_publisher(String, config['observation_topic'], 8)
            self.hp = self.create_publisher(String, config['health_topic'], 8)
            self.create_subscription(Image, config['image_topic'], self.image, qos)
            self.create_subscription(CameraInfo, config['info_topic'], self.info, qos)
            if config['require_px4_state']:
                from px4_msgs.msg import VehicleLocalPosition
                from gwm_px4_control.sample_identity import IdentityTracker
                self.state_identities=IdentityTracker(actual_run)
                self.create_subscription(VehicleLocalPosition, '/px4_71/fmu/out/vehicle_local_position_v1', self.position, qos)
            self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def sim(self): return self.get_clock().now().nanoseconds / 1e9

        def emit(self, kind, value, publisher=None):
            event = metadata(dict(kind=kind, **value))
            encoded = json.dumps(event, allow_nan=False)
            self.events.write(encoded+'\n')
            if publisher: publisher.publish(String(data=encoded))

        def error(self, exc):
            reason = str(exc)
            self.errors[reason] = self.errors.get(reason,0)+1
            self.emit('error', dict(reason=reason, wall_s=time.monotonic(), sim_s=self.sim()))

        def info(self, msg):
            self.info_count += 1
            self.info_last_sim = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if self.info_first_sim is None: self.info_first_sim=self.info_last_sim
            try:
                raw = dict(message_to_ordereddict(msg))
                header = raw.pop('header')
                raw['frame_id'] = header['frame_id']
                candidate = Calibration(raw)
                if self.calibration and candidate.identity != self.calibration.identity:
                    self.calibration = None
                    self.health.fault = 'calibration_changed'
                    raise ValueError('calibration_changed')
                if not self.calibration and not self.health.fault:
                    self.calibration = candidate
                    self.calibration_start = header
                    self.emit('calibration', dict(info=raw, raw_header=header, identity=candidate.identity,
                        validity='static_within_owned_run_until_identity_change', receipt_wall_s=time.monotonic()))
            except ValueError as exc:
                self.calibration=None
                self.health.fault='calibration_unavailable'
                self.error(exc)

        def position(self, msg):
            self.position_callbacks += 1
            from gwm_px4_control.contracts import json_message
            raw_state=dict(message_to_ordereddict(msg))
            retained=json_message(raw_state)
            fields=retained['fields']
            state=dict(timestamp_us=int(msg.timestamp),timestamp_sample_us=int(msg.timestamp_sample),
                x=fields['x'],y=fields['y'],z=fields['z'],heading=fields['heading'],xy_valid=msg.xy_valid,z_valid=msg.z_valid,
                callback_id=actual_run+':sensor-position:'+str(self.position_callbacks),run_id=actual_run,
                receipt_monotonic_ns=time.monotonic_ns(),topic='/px4_71/fmu/out/vehicle_local_position_v1',
                message_type='px4_msgs/msg/VehicleLocalPosition',message_version=1,uorb_instance=0,
                instance_provenance='verified_single_EKF2_launch_and_pinned_non_selector_source',
                source='single_EKF2_non_selector_DDS',payload=retained)
            epoch=(msg.xy_reset_counter,msg.z_reset_counter,msg.heading_reset_counter,msg.vxy_reset_counter,msg.vz_reset_counter)
            try:
                state['source_reference']=self.state_identities.observe('vehicle_local_position',raw_state,
                    state['callback_id'],state['receipt_monotonic_ns'],version=1,instance=0)
                classification=self.history.add(int(msg.timestamp),epoch,state)
            except ValueError as exc:
                classification='rejected:'+str(exc)
                self.history.fault=str(exc); self.error(exc)
            self.emit('state_callback',dict(state=state,epoch=list(epoch),classification=classification))

        def image(self, msg):
            wall = time.monotonic()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            self.received += 1
            meta = metadata(dict(id=self.received,acquisition_sim_ns=int(msg.header.stamp.sec)*1000000000+int(msg.header.stamp.nanosec), acquisition_sim_s=stamp, receipt_wall_s=wall,
                        receipt_sim_s=self.sim(), width=msg.width,height=msg.height,encoding=msg.encoding,
                        step=msg.step,is_bigendian=msg.is_bigendian,raw_frame=msg.header.frame_id))
            raw = bytes(msg.data)
            try:
                self.recorder.submit(meta, raw)
                if (msg.width,msg.height,msg.encoding,msg.step,msg.is_bigendian)!=(640,480,'32FC1',2560,0) or len(raw)!=1228800:
                    raise ValueError('incompatible_native_image_layout')
                self.health.receive(stamp, wall)
                if self.first_source is None: self.first_source=stamp
                if msg.header.frame_id != config['raw_frame']: raise ValueError('transform_unavailable')
                self.slot.put((meta,raw))
            except (ValueError, RuntimeError) as exc:
                if str(exc) == 'repeated_timestamp': self.repeated += 1
                else: self.health.fault = str(exc)
                self.error(exc)

        def tick(self):
            sim, wall = self.sim(), time.monotonic()
            status = self.recorder.failure or self.health.status(sim,wall)
            if not self.calibration and status == 'fresh': status='calibration_unavailable'
            if wall-self.last_health_wall>=.1-1e-6:
                self.emit('health', dict(status=status, sim_s=sim, wall_s=wall,
                    last_acquisition_sim_s=self.health.stamp), self.hp)
                self.last_health_wall=wall
            try:
                if not self.schedule.due(sim): return
            except ValueError as exc:
                self.health.fault=str(exc); self.error(exc); return
            item = self.slot.take()
            if item is None: return
            meta,raw=item
            try:
                if status != 'fresh': raise ValueError(status)
                depth = decode(raw,meta['width'],meta['height'],meta['encoding'],meta['step'],meta['is_bigendian'])
                mask = classify(depth,config['near_m'],config['far_m'])
                counts = {r.name.lower():int(np.count_nonzero(mask==r)) for r in Reason}
                associated = self.history.match(meta['acquisition_sim_s'],native_ns=meta['acquisition_sim_ns']) if config['require_px4_state'] else None
                if config['require_px4_state'] and (associated is None or not associated['state']['xy_valid'] or not associated['state']['z_valid']):
                    raise ValueError('state_association_unavailable')
                samples=[]
                for v in (80,160,240,320,400):
                    for u in (80,160,240,320,400,480,560):
                        reason=Reason(int(mask[v,u])).name.lower()
                        d = float(depth[v,u]) if reason == 'valid' else None
                        optical = self.calibration.point(u,v,d) if d is not None else None
                        flu,frd = optical_to_body(optical,meta['raw_frame']) if optical else (None,None)
                        samples.append(dict(u=u,v=v,depth_m=d,reason=reason,optical_m=optical,body_flu_m=flu,body_frd_m=frd))
                finished_wall=time.monotonic(); finished_sim=self.sim()
                if finished_sim-meta['acquisition_sim_s']>config['source_age_sim_s']: raise ValueError('stale_after_processing')
                self.processed += 1
                self.emit('observation',dict(schema_version=2,observation_id=self.processed,
                    image=meta,processing_wall_s=wall,processing_sim_s=sim,source_age_sim_s=sim-meta['acquisition_sim_s'],
                    processing_return_wall_s=finished_wall,processing_return_sim_s=finished_sim,raw_frame=meta['raw_frame'],optical_frame=OPTICAL_FRAME,
                    calibration_id=self.calibration.identity,extrinsic_id=EXTRINSIC_ID,depth_semantics='optical_axis_Z_m',
                    counts=counts,samples=samples,state_association=associated,
                    state_status='matched' if associated else 'not_required_ground',
                    measurement_status='measured' if counts['valid'] else 'all_invalid_unknown',
                    overwritten_processing_frames=self.slot.overwritten,health='fresh'),self.obs)
                if self.first_source is not None and sim-self.first_source >= 5 and not (args.run_dir/'sensor-ready.json').exists():
                    write_metadata('sensor-ready.json',dict(sim_s=sim,wall_s=wall,calibration_id=self.calibration.identity))
            except ValueError as exc: self.error(exc)

    rclpy.init()
    node = Adapter()
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while not stopping: rclpy.spin_once(node, timeout_sec=.1)
    finally:
        result=dict(received=node.received,processed=node.processed,repeated=node.repeated,
                    overwritten_processing_frames=node.slot.overwritten,errors=node.errors,status='complete',
                    camera_info_received=node.info_count,camera_info_first_sim_s=node.info_first_sim,
                    camera_info_last_sim_s=node.info_last_sim)
        try: result['recorder']=node.recorder.close()
        except RuntimeError as exc: result.update(status='failed',failure=str(exc))
        node.events.close()
        write_metadata('sensor-result.json',result)
        node.destroy_node(); rclpy.shutdown()
    return 0 if result['status']=='complete' else 1


if __name__ == '__main__': raise SystemExit(main())
