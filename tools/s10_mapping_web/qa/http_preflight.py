"""Explicit authenticated deployment smoke check. No map/motion/record action.

Run only against an authorized robot app; --selfcheck creates one diagnostic job.
Credentials and cookies stay in memory and never enter the report.
"""
import argparse,getpass,hashlib,http.cookiejar,json,time,urllib.request
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--selfcheck',action='store_true');a=p.parse_args()
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    def call(path,data=None,csrf=None):
        headers={}
        if data is not None:headers['Content-Type']='application/json'
        if csrf:headers['X-CSRF-Token']=csrf
        req=urllib.request.Request(a.base+path,data=json.dumps(data).encode() if data is not None else None,headers=headers)
        with opener.open(req,timeout=30) as r:
            raw=r.read();return r.status,raw
    def js(path,data=None,csrf=None):return json.loads(call(path,data,csrf)[1])
    password=getpass.getpass('Robot web password: ')
    call('/phone/login',dict(username='golai',password=password));del password
    report={'target':'0914_fr_v3-20260914-142008','time_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'pages':{}}
    for path in ['/','/field','/field.js','/localization','/heightmap']:
        status,raw=call(path);report['pages'][path]={'status':status,'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest()}
    health=js('/phone/field/health');csrf=health.pop('csrf',None);report['health']=health
    def compact(s):return {k:({x:y for x,y in v.items() if x not in ('points','covariance')} if isinstance(v,dict) else v) for k,v in s.items()}
    report['before']=compact(js('/phone/field/live'))
    listing=js('/phone/field/list');report['eligibility_api']=listing.get('eligible_checks')
    assert not any(j['state'] in ('QUEUED','RUNNING') for j in listing['jobs']), 'existing active job; did not submit'
    latest_session=listing['jobs'][0]['session_id'] if listing['jobs'] else None
    tick=time.monotonic()
    dashboard=js('/phone/field/dashboard'+('?session_id='+latest_session if latest_session else ''))
    overview=dashboard['listing'].get('overview') or {}
    report['dashboard']={'seconds':time.monotonic()-tick,'ui_contract':dashboard['health'].get('ui_contract'),
                         'live_error':dashboard.get('live_error'),'current_map':(dashboard.get('live') or {}).get('map_name'),
                         'target':(dashboard['listing'].get('session') or {}).get('target'),
                         'saved_waypoint_count':overview.get('saved_waypoint_count'),
                         'last_report_passed':((overview.get('report') or {}).get('result') or {}).get('passed'),
                         'active_job':dashboard['listing'].get('active_job')}
    assert report['dashboard']['ui_contract']==2, 'dashboard contract mismatch'
    if a.selfcheck:
        import secrets
        req=dict(action='selfcheck',key=secrets.token_hex(24),params=dict(target_map=report['target']))
        job=js('/phone/field/submit',req,csrf);deadline=time.monotonic()+55
        while job['state'] in ('QUEUED','RUNNING') and time.monotonic()<deadline:
            time.sleep(1);job=js('/phone/field/job?job_id='+job['id'])
        result=job.get('result') or {}
        report['selfcheck']={k:job.get(k) for k in ('id','session_id','state','error')}
        report['selfcheck']['result']={k:compact(v) if k=='snapshot' else v for k,v in result.items()}
        preview=js('/phone/field/preview?session_id='+job['session_id'])
        report['preview']={k:v for k,v in preview.items() if k!='points'}
        report['preview']['display_points']=len(preview.get('points',[]))
        if job.get('artifacts'):
            artifact=job['artifacts'][0];status,raw=call('/phone/field/download?artifact_id='+artifact['id'])
            report['download']={'status':status,'bytes':len(raw),'sha256_matches':hashlib.sha256(raw).hexdigest()==artifact['sha256']}
    report['after']=compact(js('/phone/field/live'))
    report['map_and_localization_unchanged']=all(report['before'].get(k)==report['after'].get(k) for k in ('map_identity','invocation','boot_id'))
    a.out.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    a.out.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('before','after')},ensure_ascii=False,indent=2))
    if a.selfcheck and (report['selfcheck']['state'] != 'SUCCEEDED'
                        or report['selfcheck']['result'].get('passed') is not True):
        raise SystemExit(2)  # Job completion alone is not a quality pass.

if __name__=='__main__':main()
