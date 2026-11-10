"""Replay recorded sensors to inspect localization and optional global planning."""
import argparse
from collections import Counter
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tarfile
import time

import yaml


@contextmanager
def preserve_fastlio_logs(directory, output):
    """Restore FAST-LIO's known, unconditionally opened source debug logs."""
    names = ('pos_log.txt', 'mat_pre.txt', 'mat_out.txt', 'imu.txt', 'dbg.txt',
             'fast_lio_time_log.csv')
    state = dict(directory=str(directory) if directory else None, restored=False,
                 protected_files=[])
    snapshots = {}
    if directory is not None:
        directory = Path(directory)
        for name in names:
            path = directory / name
            if path.exists():
                info = path.stat()
                snapshots[name] = (path.read_bytes(), info)
            else:
                snapshots[name] = None
        with tarfile.open(Path(output) / 'fastlio_logs_before.tar.gz', 'w:gz') as archive:
            for name, snapshot in snapshots.items():
                if snapshot is not None:
                    archive.add(directory / name, arcname=name)
        state['protected_files'] = list(names)
    try:
        yield state
    finally:
        for name, snapshot in snapshots.items():
            path = directory / name
            if snapshot is None:
                path.unlink(missing_ok=True)
            else:
                data, info = snapshot
                path.write_bytes(data)
                path.chmod(info.st_mode)
                os.utime(path, ns=(info.st_atime_ns, info.st_mtime_ns))
                if hashlib.sha256(path.read_bytes()).digest() != hashlib.sha256(data).digest():
                    raise RuntimeError('Failed to restore original FAST-LIO log: ' + str(path))
                os.utime(path, ns=(info.st_atime_ns, info.st_mtime_ns))
        state['restored'] = directory is not None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('database')
    parser.add_argument('navigation_map')
    parser.add_argument('output', help='New results directory')
    parser.add_argument('--domain', type=int, default=114)
    parser.add_argument('--relocalization-params', type=Path,
                        help='Optional localization YAML for an isolated parameter comparison')
    parser.add_argument('--planning-cases', type=Path,
                        help='Optional fixed map-frame goals/checkpoints YAML; starts only the planner')
    parser.add_argument('--planner-params', type=Path,
                        help='Planner parameters before applying the same known-map overlay as offline_nav')
    parser.add_argument('--planning-timeout', type=float, default=5.0,
                        help='Wall-clock timeout per asynchronous planning request')
    parser.add_argument('--live-obstacle-topic', default='/terrain/obstacles_measured_global',
                        help='Same live global-planner obstacle input as offline_nav')
    parser.add_argument('--obstacle-evidence', action='store_true',
                        help='Save bounded original clouds, exact-stamp TF and collision-neighborhood evidence')
    parser.add_argument('--live-obstacle-freshness-timeout', type=float, default=2.0,
                        help='Shared global-planner and adapter freshness limit, seconds')
    parser.add_argument('--measured-obstacle-decay-time', type=float, default=.6,
                        help='Fresh measured-return retention in the terrain node, seconds (default: 0.6)')
    parser.add_argument('--fastlio-log-directory', type=Path,
                        help='Override source debug-log directory to preserve during replay')
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error('Use a private test domain from 1 to 232')
    if not math.isfinite(args.planning_timeout) or args.planning_timeout <= 0:
        parser.error('Planning timeout must be positive and finite')
    if not math.isfinite(args.measured_obstacle_decay_time) or args.measured_obstacle_decay_time <= 0:
        parser.error('Measured obstacle decay time must be positive and finite')
    if not math.isfinite(args.live_obstacle_freshness_timeout) or not 0 < args.live_obstacle_freshness_timeout <= 3600:
        parser.error('Live obstacle freshness timeout must be positive, finite and bounded')
    if args.obstacle_evidence and args.planning_cases is None:
        parser.error('--obstacle-evidence requires --planning-cases')
    bag, database, navigation = [Path(value).expanduser().resolve()
                                 for value in (args.bag, args.database, args.navigation_map)]
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    names = {item['topic_metadata']['name'] for item in metadata['topics_with_message_count']}
    if not {'/hesai/points', '/go2/imu', '/lf/sportmodestate'} <= names:
        parser.error('Bag must contain /hesai/points, /go2/imu and /lf/sportmodestate')
    from .map_io import load_map, map_identity, offline_planner_settings
    navigation_metadata, support, obstacles = load_map(navigation, database)
    planning_config = None
    if args.planning_cases is not None:
        from .planning_evaluation import load_planning_cases
        planning_config = load_planning_cases(args.planning_cases)
        expected = planning_config.get('expected_map_id')
        identity = map_identity(navigation_metadata, support, obstacles)
        if expected is not None and expected not in (identity['map_id'], identity['geometry_id']):
            parser.error('Planning cases belong to a different navigation map')
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for name in ('CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                 'ROS_DISCOVERY_SERVER', 'ROS_STATIC_PEERS', 'ROS_AUTOMATIC_DISCOVERY_RANGE'):
        os.environ.pop(name, None)
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    source_logs = args.fastlio_log_directory
    if source_logs is None:
        prefix = Path(get_package_prefix('fast_lio'))
        source_logs = next((base / 'src/FAST_LIO_ROS2/Log'
            for base in (prefix.parent.parent, prefix.parent)
            if (base / 'src/FAST_LIO_ROS2/Log').is_dir()), None)
    elif not source_logs.is_dir():
        parser.error('FAST-LIO source log directory does not exist')
    # FAST-LIO writes these files irrespective of ROS domain or its log-enable
    # flag. Refuse to race another local mapping process using the same source.
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            executable = Path(path.read_bytes().split(b'\0')[0].decode()).name
        except (OSError, UnicodeDecodeError):
            continue
        if executable == 'fastlio_mapping':
            raise RuntimeError('Another local FAST-LIO process is running; stop it before replay')
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TransformStamped
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.parameter import Parameter
    rclpy.init()
    node = rclpy.create_node('known_map_replay_probe', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    clock = [0]
    statuses, ready, reasons, alignments = [], [], [], []
    processes, logs = [], []
    cpu_samples, previous_cpu = [], {}
    previous_cpu_time = [None]
    planning = None
    if planning_config is not None:
        from .planning_evaluation import PlanningEvaluation
        planning = PlanningEvaluation(node, planning_config,
            int(metadata['starting_time']['nanoseconds_since_epoch']),
            timeout_s=args.planning_timeout,
            evidence_output=output / 'obstacle_evidence' if args.obstacle_evidence else None,
            live_obstacle_topic=args.live_obstacle_topic)
    clock_ticks, page_bytes = os.sysconf('SC_CLK_TCK'), os.sysconf('SC_PAGE_SIZE')

    def sample_cpu():
        # Follow only descendants of this probe's own child processes. Sum of
        # process RSS includes shared pages; CPU 100% means one logical core.
        sampled_at = time.monotonic()
        table = {}
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = path.read_text().rsplit(')', 1)[1].split()
                table[int(path.parent.name)] = (
                    int(fields[1]), int(fields[11]) + int(fields[12]),
                    int(fields[19]), int(fields[21]) * page_bytes / 1048576)
            except (OSError, ValueError, IndexError):
                continue
        elapsed = (sampled_at - previous_cpu_time[0]
                   if previous_cpu_time[0] is not None else 0)
        readings = []
        current_cpu = {}
        for name, process in processes:
            descendants = {process.pid}
            while True:
                new = {pid for pid, values in table.items() if values[0] in descendants}
                if new <= descendants:
                    break
                descendants.update(new)
            for pid in sorted(descendants & table.keys()):
                _, ticks, started, rss = table[pid]
                key = (pid, started)
                current_cpu[key] = ticks
                cpu = (100 * max(0, ticks - previous_cpu[key]) / clock_ticks / elapsed
                       if elapsed > 0 and key in previous_cpu else None)
                try:
                    command = (Path('/proc') / str(pid) / 'cmdline').read_bytes()
                    executable = Path(command.split(b'\0')[0].decode(errors='replace')).name
                except OSError:
                    executable = 'exited'
                readings.append(dict(group=name, pid=pid, executable=executable,
                                     cpu_percent=cpu, rss_mib=rss))
        cpu_samples.append(dict(observed_clock_ns=clock[0], elapsed_s=elapsed,
                                processes=readings))
        previous_cpu.clear()
        previous_cpu.update(current_cpu)
        previous_cpu_time[0] = sampled_at
    state = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def diagnostic(message):
        if message.status:
            statuses.append(dict({item.key: item.value for item in message.status[0].values},
                                 observed_clock_ns=clock[0]))
    def alignment(message):
        value = message.transform
        alignments.append(dict(stamp_ns=message.header.stamp.sec * 1000000000 +
            message.header.stamp.nanosec, translation=[value.translation.x,
            value.translation.y, value.translation.z], quaternion=[value.rotation.x,
            value.rotation.y, value.rotation.z, value.rotation.w]))
    subscriptions = [
        node.create_subscription(Clock, '/clock', lambda message: clock.__setitem__(
            0, message.clock.sec * 1000000000 + message.clock.nanosec), qos_profile_sensor_data),
        node.create_subscription(DiagnosticArray, '/relocalization/status', diagnostic, state),
        node.create_subscription(Bool, '/offline_localization/ready',
                                 lambda message: ready.append((clock[0], message.data)), state),
        node.create_subscription(String, '/offline_localization/status',
                                 lambda message: reasons.append((clock[0], message.data)), state),
        node.create_subscription(TransformStamped, '/relocalization/map_to_odom', alignment, 10)]
    commands = [
        ('fusion', ['ros2', 'launch', 'fast_lio_nav2', 'odom_fusion.launch.py',
                    'publish_map_odom_tf:=false', 'use_sim_time:=true', 'lidar_driver:=none',
                    'start_go2_imu_bridge:=false', 'start_go2_odom_bridge:=false',
                    'measured_obstacle_decay_time:=' + str(args.measured_obstacle_decay_time)]),
        ('raw_state', ['ros2', 'run', 'go2_odom_bridge', 'sportmodestate_odom_bridge', '--ros-args',
                       '-r', '__node:=go2_odom_bag_bridge', '-p', 'bag_path:=' + str(bag),
                       '-p', 'input_topic:=/lf/sportmodestate', '-p', 'velocity_frame:=body',
                       '-p', 'timestamp_source:=receive', '-p', 'use_sim_time:=true']),
        ('offline', ['ros2', 'launch', 'go2_offline_navigation', 'offline_nav.launch.py',
                     'database:=' + str(database), 'map_directory:=' + str(navigation),
                     'start_odometry:=false', 'start_navigation:=false',
                     'start_cmd_vel_bridge:=false', 'use_sim_time:=true',
                     'live_obstacle_freshness_timeout:=' + str(args.live_obstacle_freshness_timeout)]),
        ('starter', ['ros2', 'run', 'fast_lio_nav2', 'replay_starter.py', '--ros-args',
                     '-p', 'bag_path:=' + str(bag), '-p', 'use_sim_time:=true']),
        ('player', ['ros2', 'bag', 'play', str(bag), '--clock', '100', '--rate', '1',
                    '--start-paused', '--topics', '/hesai/points', '/go2/imu'])]
    if args.relocalization_params is not None:
        localization_params = args.relocalization_params.expanduser().resolve(strict=True)
        commands[2][1].append('relocalization_params_file:=' + str(localization_params))
    if planning is not None:
        source_parameters = args.planner_params or Path(get_package_share_directory(
            'go2_offline_navigation')) / 'config/nav2_mppi_params.yaml'
        parameters = yaml.safe_load(source_parameters.read_text())
        planner_parameters = parameters['planner_server']['ros__parameters']
        planner_parameters['GridBased'].update(offline_planner_settings(
            navigation_metadata, support, obstacles, planner_parameters['GridBased'],
            live_obstacle_topic=args.live_obstacle_topic,
            measured_obstacle_decay_time=args.measured_obstacle_decay_time,
            live_obstacle_freshness_timeout=args.live_obstacle_freshness_timeout))
        planner_parameters['use_sim_time'] = True
        parameters['global_costmap']['global_costmap']['ros__parameters']['use_sim_time'] = True
        parameter_file = output / 'planner_params.yaml'
        parameter_file.write_text(yaml.safe_dump(parameters))
        commands[3:3] = [
            ('planner', ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args',
                         '--params-file', str(parameter_file)]),
            ('planner_lifecycle', ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager',
                '--ros-args', '-r', '__node:=replay_planner_lifecycle_manager',
                '-p', 'use_sim_time:=true', '-p', 'autostart:=true',
                '-p', 'node_names:=[planner_server]'])]
    (output / 'commands.json').write_text(json.dumps(dict(
        commands={name: command for name, command in commands},
        measured_obstacle_decay_time_s=args.measured_obstacle_decay_time,
        obstacle_evidence_enabled=args.obstacle_evidence), ensure_ascii=False, indent=2)+'\n')
    log_preserver = preserve_fastlio_logs(source_logs, output)
    preserved_logs = log_preserver.__enter__()
    try:
        # Never share TF and simulated sensors with an already-running stack.
        end = time.monotonic() + 1
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        if any(name != node.get_name() for name in node.get_node_names()):
            raise RuntimeError('Test ROS domain already has nodes; choose another --domain')
        for name, command in commands:
            logfile = (output / (name + '.log')).open('w')
            logs.append(logfile)
            processes.append((name, subprocess.Popen(command, stdout=logfile,
                stderr=subprocess.STDOUT, start_new_session=True)))
        timeout = metadata['duration']['nanoseconds'] * 1e-9 + 25
        deadline, progress = time.monotonic() + timeout, time.monotonic()
        sample_cpu()
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if planning is not None:
                planning.tick(clock[0])
            if time.monotonic() - previous_cpu_time[0] >= 1.0:
                sample_cpu()
            if any(p.poll() is not None and name != 'player' for name, p in processes):
                raise RuntimeError('A replay process exited; inspect logs')
            if time.monotonic() - progress > 20:
                print('Confirmed measurements:', len({item.get('last_verified_stamp')
                    for item in statuses if item.get('valid') == 'true'}),
                    'ever ready:', any(value for _, value in ready), flush=True)
                progress = time.monotonic()
            if any(name == 'player' and p.poll() == 0 for name, p in processes):
                break
            if any(name == 'player' and p.poll() not in (None, 0) for name, p in processes):
                raise RuntimeError('Bag player failed')
        if not any(name == 'player' and p.poll() == 0 for name, p in processes):
            raise RuntimeError('Bag playback did not finish in time')
        if planning is not None:
            planning.finish(clock[0])
            grace = time.monotonic() + args.planning_timeout + 1.0
            while not planning.is_idle() and time.monotonic() < grace:
                rclpy.spin_once(node, timeout_sec=.05)
                planning.tick(clock[0])
        client = node.create_client(Trigger, '/replay_starter/finish')
        if client.wait_for_service(timeout_sec=3):
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 3
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.get_publishers_info_by_topic('/api/sport/request'):
            raise RuntimeError('Unexpected physical command publisher in test domain')
        forbidden = {'controller_server', 'bt_navigator', 'behavior_server', 'velocity_smoother',
                     'go2_cmd_vel_bridge', 'offline_cmd_vel_bridge'} & set(node.get_node_names())
        if forbidden:
            raise RuntimeError('Unexpected control nodes in replay: ' + str(sorted(forbidden)))
        planning_report = None
        if planning is not None:
            from .planning_evaluation import write_planning_reports
            planning.write_evidence()
            planning_report = write_planning_reports(output, planning.summary(), output / 'planner.log')
        report = dict(input_bag=str(bag), database=str(database),
            unique_confirmed_matches=len({item.get('last_verified_stamp') for item in statuses
                                           if item.get('valid') == 'true'}),
            ever_navigation_ready=any(value for _, value in ready),
            reason_counts=dict(Counter(item.get('reason') for item in statuses)),
            physical_api_publishers=0, hardware_runs=0, domain=args.domain,
            controllers_started=0, command_bridges_started=0,
            planning=planning_report,
            source_log_preservation=preserved_logs,
            statuses=statuses, ready_samples=ready, reason_samples=reasons, alignments=alignments,
            cpu_samples=cpu_samples)
        (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({key: value for key, value in report.items() if key not in
                         ('statuses', 'ready_samples', 'reason_samples', 'alignments',
                          'cpu_samples', 'planning', 'source_log_preservation')}, ensure_ascii=False), flush=True)
    except BaseException as error:
        failure = dict(input_bag=str(bag), database=str(database), domain=args.domain,
            exception_type=type(error).__name__, error=str(error), hardware_runs=0,
            planning=planning.summary() if planning is not None else None,
            statuses=statuses, ready_samples=ready, reason_samples=reasons,
            source_log_preservation=preserved_logs,
            child_return_codes={name: process.poll() for name, process in processes})
        (output / 'failure_report.json').write_text(json.dumps(
            failure, ensure_ascii=False, indent=2) + '\n')
        raise
    finally:
        try:
            for _, process in processes:
                if process.poll() is None:
                    try:
                        process.send_signal(signal.SIGINT)
                    except ProcessLookupError:
                        pass
            for _, process in processes:
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()
            for logfile in logs:
                logfile.close()
            if planning is not None:
                planning.write_evidence()
                planning.close()
            node.destroy_node()
            rclpy.shutdown()
        finally:
            log_preserver.__exit__(None, None, None)
            for name in ('report.json', 'failure_report.json'):
                path = output / name
                if path.exists():
                    saved = json.loads(path.read_text())
                    saved['source_log_preservation'] = preserved_logs
                    if planning is not None and isinstance(saved.get('planning'), dict):
                        saved['planning']['obstacle_evidence'] = planning.evidence_report
                    path.write_text(json.dumps(saved, ensure_ascii=False, indent=2) + '\n')
