"""Copy exact vendor message definitions from the trusted mapping sensor board.

Run on Orin as the mapping user. Credentials remain in the existing mapping config.
Only the dependency closure needed by capture is copied, never firmware binaries.
"""
import hashlib
import json
from pathlib import Path
import re
import paramiko


def main():
    home = Path.home()
    config = json.loads((home/'.config/s10-mapping-web/config.json').read_text())
    client = paramiko.SSHClient()
    client.load_system_host_keys(str(home/'.ssh/known_hosts'))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(config['robot_host'], username=config['robot_user'],
                   password=config['robot_password'], look_for_keys=False, allow_agent=False, timeout=8)
    root = home/'s10_gait_capture/vendor_ws/src/drdds'
    (root/'msg').mkdir(parents=True, exist_ok=True)
    copied = {}
    dependencies = {'builtin_interfaces'}
    with client.open_sftp() as sftp:
        def fetch(name):
            if name in copied:
                return
            if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
                raise ValueError('Invalid message name')
            remote = '/opt/ros/jazzy/share/drdds/msg/'+name+'.msg'
            with sftp.open(remote, 'rb') as stream:
                raw = stream.read()
            copied[name] = hashlib.sha256(raw).hexdigest()
            (root/'msg'/f'{name}.msg').write_bytes(raw)
            for line in raw.decode().splitlines():
                line = line.split('#',1)[0].strip()
                if not line:
                    continue
                field_type = line.split()[0].split('[',1)[0]
                if '/' in field_type:
                    package, local = field_type.split('/',1)
                    if package == 'drdds':
                        fetch(local)
                    else:
                        dependencies.add(package)
                elif field_type[0].isupper():
                    fetch(field_type)
        for name in ('MotionInfo','JointsData','JointsDataCmd','Gait','LocationStatus','Steer'):
            fetch(name)
    client.close()
    cmake = ['cmake_minimum_required(VERSION 3.8)', 'project(drdds)',
             'find_package(ament_cmake REQUIRED)', 'find_package(rosidl_default_generators REQUIRED)']
    cmake += [f'find_package({dep} REQUIRED)' for dep in sorted(dependencies)]
    cmake += ['rosidl_generate_interfaces(${PROJECT_NAME}']
    cmake += [f'  "msg/{name}.msg"' for name in sorted(copied)]
    cmake += ['  DEPENDENCIES '+' '.join(sorted(dependencies))+')',
              'ament_export_dependencies(rosidl_default_runtime)', 'ament_package()']
    (root/'CMakeLists.txt').write_text('\n'.join(cmake)+'\n')
    package = '''<?xml version="1.0"?><package format="3"><name>drdds</name><version>0.0.0</version>
<description>Exact installed vendor interfaces for read-only capture</description>
<maintainer email="local@example.invalid">S10 local capture</maintainer><license>Vendor supplied</license>
<buildtool_depend>ament_cmake</buildtool_depend><build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend><member_of_group>rosidl_interface_packages</member_of_group>
'''+''.join(f'<depend>{dep}</depend>' for dep in sorted(dependencies))+'''
<export><build_type>ament_cmake</build_type></export></package>'''
    (root/'package.xml').write_text(package)
    (root.parent.parent/'VENDOR_SOURCE.json').write_text(json.dumps({
        'source_host': config['robot_host'], 'source_directory': '/opt/ros/jazzy/share/drdds/msg',
        'sha256': copied}, indent=2))
    print(f'Copied {len(copied)} exact vendor definitions; dependencies: {sorted(dependencies)}')


if __name__ == '__main__':
    main()
