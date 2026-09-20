"""Copy closed AGX recordings without changing remote files."""
import json
from pathlib import Path
import shlex
import tarfile
import time
import paramiko

ROOT = Path(__file__).resolve().parent
REMOTE = '/home/xwy/s10_gait_data'

def connect():
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.connect('10.21.33.102', username='ysc',
                   key_filename=str(Path.home()/'.ssh/s10_agx_ed25519'), timeout=10)
    return client

if __name__ == '__main__':
    client = connect()
    code = """import json,pathlib
r=pathlib.Path('/home/xwy/s10_gait_data')
items=[]
for p in sorted(r.glob('**/manifest.json')):
 m=json.loads(p.read_text()); files=[{'path':str(f.relative_to(r)), 'size':f.stat().st_size,'mtime':f.stat().st_mtime_ns} for f in p.parent.rglob('*') if f.is_file()]
 items.append({'path':str(p.parent.relative_to(r)),'manifest':m,'files':files})
print(json.dumps(items))
"""
    stdin, stdout, stderr = client.exec_command('cd /tmp && sudo -n -u xwy python3 -c '+shlex.quote(code))
    items = json.loads(stdout.read())
    if stdout.channel.recv_exit_status():
        raise RuntimeError(stderr.read().decode())
    ROOT.mkdir(exist_ok=True)
    (ROOT/'remote_inventory.json').write_text(json.dumps(items, indent=2, ensure_ascii=False),encoding='utf-8')
    selected = [x for x in items if x['manifest']['status'] != 'recording']
    print(json.dumps([{'path':x['path'],'status':x['manifest']['status'],'terrain':x['manifest']['terrain'],'duration':x['manifest'].get('duration_s'),'bytes':sum(f['size'] for f in x['files'])} for x in items],ensure_ascii=False),flush=True)
    names = [x['path'] for x in sorted(selected,key=lambda x:sum(f['size'] for f in x['files']))]
    command = 'cd /tmp && sudo -n -u xwy tar -C '+shlex.quote(REMOTE)+' -cf - -- '+' '.join(map(shlex.quote,names))
    stdin, stdout, stderr = client.exec_command(command)
    target=ROOT/'raw'; target.mkdir(exist_ok=True)
    start=time.monotonic(); total=0
    with tarfile.open(fileobj=stdout,mode='r|') as archive:
        for member in archive:
            path=(target/member.name).resolve()
            if not path.is_relative_to(target.resolve()): raise ValueError(member.name)
            if member.isdir(): path.mkdir(parents=True,exist_ok=True)
            elif member.isfile():
                path.parent.mkdir(parents=True,exist_ok=True)
                print('COPY '+member.name+' '+str(member.size),flush=True)
                with archive.extractfile(member) as source, path.with_suffix(path.suffix+'.partial').open('wb') as dest:
                    while block:=source.read(4*1024*1024):
                        dest.write(block);total+=len(block)
                path.with_suffix(path.suffix+'.partial').replace(path)
                print(f'DONE {total/1e9:.3f} GB, {total/(time.monotonic()-start)/1e6:.1f} MB/s',flush=True)
            else: raise ValueError('Unexpected archive member '+member.name)
    rc=stdout.channel.recv_exit_status()
    if rc: raise RuntimeError(stderr.read().decode())
    checks=[]
    for item in selected:
        for f in item['files']:
            p=target/f['path']; checks.append(p.is_file() and p.stat().st_size==f['size'])
    assert checks and all(checks), 'Copy size mismatch'
    (ROOT/'copy_result.json').write_text(json.dumps({'files':len(checks),'bytes':total,'all_sizes_match':True,'remote':REMOTE},indent=2))
    print('COPY COMPLETE',flush=True)
    client.close()
