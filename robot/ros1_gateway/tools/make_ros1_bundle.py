#!/usr/bin/env python3
"""Build a relocatable, user-space ROS 1 (ROS-O "one") bundle for the 48 AGX.

Runs inside the builder image (same Ubuntu 24.04 arm64 as the AGX). Copies the
files of every package the AGX lacks (list from comparing dpkg databases) into
<stage>/sysroot and rewrites absolute paths so the tree works from --prefix
without root, without apt and without touching /opt or /usr on the AGX.

Usage (inside the builder container):
  python3 make_ros1_bundle.py missing-on-agx.txt /out/ros1-bundle.tar.gz \
      --prefix /home/golai/ros1_gateway/ros1
"""
import argparse
import os
import re
import shutil
import subprocess
import tarfile
from pathlib import Path

# Pulled in by the image but never needed on the robot.
SKIP = {'default-libmysqlclient-dev', 'libmysqlclient-dev', 'python3-colcon-mixin', 'python3-vcstool',
        'unminimize', 'python3-nose', 'libldap-dev', 'libldap2-dev', 'libsctp-dev'}
LIBDIR = '/usr/lib/aarch64-linux-gnu'


def package_files(pkg):
    out = subprocess.run(['dpkg', '-L', pkg], capture_output=True, text=True, check=True).stdout
    return [p for p in out.splitlines() if p and (os.path.isfile(p) or os.path.islink(p))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('missing')
    ap.add_argument('output')
    ap.add_argument('--prefix', default='/home/golai/ros1_gateway/ros1')
    args = ap.parse_args()
    stage = Path('/tmp/ros1_bundle_stage')
    shutil.rmtree(stage, ignore_errors=True)
    sysroot = stage / 'sysroot'
    pkgs = [p.strip() for p in open(args.missing) if p.strip() and p.strip() not in SKIP]
    copied = 0
    for pkg in pkgs:
        for f in package_files(pkg):
            dst = sysroot / f.lstrip('/')
            dst.parent.mkdir(parents=True, exist_ok=True)
            if os.path.islink(f):
                if not dst.exists() and not dst.is_symlink():
                    os.symlink(os.readlink(f), dst)
            else:
                shutil.copy2(f, dst)
            copied += 1
    # -dev symlinks (libfoo.so -> libfoo.so.1.83.0) whose target lives in a runtime
    # package the AGX already has: copy the real file so linking works in-bundle.
    for link in list((sysroot / LIBDIR.lstrip('/')).glob('*')):
        if link.is_symlink() and not link.exists():
            real = Path(LIBDIR) / os.readlink(link)
            if real.exists():
                shutil.copy2(real.resolve(), link.parent / os.readlink(link))
    new_root = f'{args.prefix}/sysroot'
    rewritten = 0
    ros_root = sysroot / 'opt/ros/one'
    lib_pattern = re.compile(re.escape(LIBDIR) + r'/(lib[A-Za-z0-9_.+-]+)')
    for path in ros_root.rglob('*'):
        if not path.is_file() or path.is_symlink():
            continue
        try:
            text = path.read_text()
        except (UnicodeDecodeError, PermissionError):
            continue  # binaries keep their paths; the env script sets search paths
        new = text.replace('/opt/ros/one', f'{new_root}/opt/ros/one')
        if path.suffix in ('.pc', '.cmake'):
            new = lib_pattern.sub(
                lambda m: f'{new_root}{LIBDIR}/{m.group(1)}'
                if (sysroot / LIBDIR.lstrip('/') / m.group(1)).exists() else m.group(0), new)
        if new != text:
            path.write_text(new)
            rewritten += 1
    shutil.copy2(Path(__file__).with_name('ros1_env.sh'), stage / 'ros1_env.sh')
    (stage / 'PACKAGES.txt').write_text('\n'.join(pkgs) + '\n')
    with tarfile.open(args.output, 'w:gz') as tar:
        tar.add(stage, arcname='ros1')
    print(f'packages={len(pkgs)} files={copied} rewritten={rewritten} output={args.output}')


if __name__ == '__main__':
    main()
