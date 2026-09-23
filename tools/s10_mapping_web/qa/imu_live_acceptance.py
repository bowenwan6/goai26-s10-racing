"""Read-only deployed-app checks via an explicit loopback SSH tunnel.

Never starts recording or robot actions. Only POST is normal login.
Optional --record downloads an existing diagnostic, without changing it.
"""
import argparse
import getpass
import hashlib
import io
import json
import time
import zipfile
from pathlib import Path
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import HTTPCookieProcessor, ProxyHandler, Request, build_opener

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--url',required=True)
p.add_argument('--output',required=True,type=Path)
p.add_argument('--record')
a=p.parse_args()
assert urlsplit(a.url).hostname=='127.0.0.1', 'Use an explicit local SSH tunnel'
a.output.mkdir(parents=True,exist_ok=False)
client=build_opener(ProxyHandler({}),HTTPCookieProcessor())
checks=[]
def request(path,body=None):
    started=time.monotonic()
    try:
        r=client.open(Request(a.url+path,data=json.dumps(body).encode() if body else None,
            headers={'Content-Type':'application/json'}),timeout=120)
    except HTTPError as exc:r=exc
    with r:raw=r.read();status=r.code
    checks.append(dict(path=path,status=status,seconds=round(time.monotonic()-started,3),bytes=len(raw)))
    return status,raw
assert request('/phone/imu/live')[0]==401
assert request('/phone/login',dict(username='golai',password=getpass.getpass('Existing app password: ')))[0]==200
for path in ['/','/imu-check','/imu_diag.js','/field','/field.js','/localization','/heightmap','/phone/field/health','/phone/imu/list']:
    assert request(path)[0]==200,path
for path in ['/phone/imu/live?cursor=-1','/phone/imu/live?cmd=move','/phone/imu/report?id=../passwd','/phone/imu/list?x=1']:
    assert request(path)[0]==400,path
samples=[]
for _ in range(12):
    code,raw=request('/phone/imu/live');assert code==200
    d=json.loads(raw);d.pop('csrf',None);d.pop('rows',None);samples.append(d)
    time.sleep(1)
code,raw=request('/phone/field/live');assert code==200
field=json.loads(raw)
for key in ['cloud','aligned_cloud']:
    if isinstance(field.get(key),dict):field[key].pop('points',None)
if a.record:
    assert len(a.record)==32 and all(c in '0123456789abcdef' for c in a.record)
    code,raw=request('/phone/imu/report?id='+a.record);assert code==200
    report=json.loads(raw)
    (a.output/'diagnostic-report.json').write_bytes(raw)
    code,raw=request('/phone/imu/download?id='+a.record);assert code==200
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        assert z.testzip() is None
        assert json.loads(z.read('report.json'))==report
        rows=[json.loads(line) for line in z.read('samples.jsonl').splitlines()]
        assert len(rows)==sum(report['topic_counts'].values())
        assert all('cdr_b64' in row for row in rows if row['topic'] in ('imu','odom'))
    (a.output/'diagnostic.zip').write_bytes(raw)
    checks.append(dict(zip_sha256=hashlib.sha256(raw).hexdigest(),rows=len(rows)))
result=dict(checks=checks,live_samples=samples,field_live=field,record_started_by_this_test=False)
(a.output/'acceptance.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
last=samples[-1]
assert last['online'] and not last['error'] and last['latest']['imu']['age']+last['transport_age']<1
print(json.dumps(dict(checks=len(checks),queue_dropped=last['queue_dropped'],latest_age=last['latest']['imu']['age'],
                     field_error=field.get('error'),output=str(a.output)),ensure_ascii=False))
