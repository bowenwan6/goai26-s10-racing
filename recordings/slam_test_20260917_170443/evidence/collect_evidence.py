"""Collect generated read-only audit results and explicitly scoped configuration files."""
import hashlib
import json
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parent
ssh = ['ssh', '-S', '/tmp/s10-rosbag-session.pODJ7o/106.sock', '-o', 'BatchMode=yes',
       '-o', 'ConnectTimeout=8', 'user@10.21.33.106']
cmd = "bash -c 'source /opt/ros/jazzy/setup.bash; exec nice -n 10 python3 - /var/opt/robot/data/slam_test_20260917_170443'"
r = subprocess.run(ssh + [cmd], input=(root/'audit_bag.py').read_bytes(), capture_output=True, timeout=300)
(root/'audit_stderr.txt').write_bytes(r.stderr)
r.check_returncode()
# Native ROS libraries may print an informational line; retain it separately.
start = r.stdout.index(b'{')
(root/'audit_stdout_prefix.txt').write_bytes(r.stdout[:start])
value = json.loads(r.stdout[start:])
(root/'bag_audit.json').write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
print(json.dumps({t: {'count': s['count'], 'hz': s['source_hz'], 'max_gap': s['source_interval_s']['max']}
                  for t,s in value['topics'].items()}, ensure_ascii=False), flush=True)
cfg = root/'configuration_snapshot_after_recording'
cfg.mkdir(exist_ok=True)
sources = ['/var/opt/robot/conf/rslidar/config.yaml', '/var/opt/robot/conf/yesense_node/yesense.yaml',
           '/var/opt/robot/conf/slam/params.yaml', '/opt/robot/share/slam/conf/params.yaml',
           '/var/opt/robot/conf/robot_hardware_info.toml', '/opt/robot/fastdds.xml']
manifest = []
for source in sources:
    name = source.lstrip('/').replace('/', '__')
    result = subprocess.run(ssh + ['cat '+source], capture_output=True, timeout=30)
    if result.returncode:
        manifest.append(dict(source=source, error=result.stderr.decode(errors='replace')))
    else:
        (cfg/name).write_bytes(result.stdout)
        manifest.append(dict(source=source, local=name, bytes=len(result.stdout),
                             sha256=hashlib.sha256(result.stdout).hexdigest()))
(cfg/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
print('EVIDENCE_COLLECTED', flush=True)
