// LOCAL ONLY. Real static pages/login, explicitly synthetic navigation responses.
const assert=require('node:assert/strict'),path=require('node:path'),fs=require('node:fs');
const {spawn}=require('node:child_process');
const {chromium}=require(process.env.S10_QA_PLAYWRIGHT||'/Users/xxxwbwxxx/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const here=path.resolve(__dirname,'..'),output=process.env.S10_QA_OUTPUT||path.join(__dirname,'native-nav-evidence');fs.mkdirSync(output,{recursive:true});
const code=path.resolve(here,'../..');
const py=spawn(process.env.S10_QA_PYTHON||'python3',['-u','-c',`import hashlib,server as w
from http.server import ThreadingHTTPServer
w.config={'login_hash':hashlib.sha256(b'demo:native-local').hexdigest()}
def forbidden(*a,**k): raise RuntimeError('LOCAL UI QA: robot/SSH forbidden')
w.connect=forbidden
h=ThreadingHTTPServer(('127.0.0.1',0),w.Handler)
print(h.server_port,flush=True)
h.serve_forever()`],{cwd:here});
const modes=['observe','flat','stairs','start','b','start_b'];
const routes=['start','b','start_b'].map(id=>{const raw=JSON.parse(fs.readFileSync(path.join(code,'native_transfer/config',id+'.draft.json')));return {id,label:id,points:raw.waypoints,modes:raw.waypoints.map(p=>p.kind).filter((k,i,a)=>i===0||k!==a[i-1])};});
(async()=>{
 const port=await new Promise((resolve,reject)=>{py.stdout.once('data',b=>resolve(Number(b.toString().trim())));py.once('error',reject);});
 const origin='http://127.0.0.1:'+port;
 const browser=await chromium.launch({headless:true});
 const context=await browser.newContext({viewport:{width:390,height:844},isMobile:true,hasTouch:true});
 const page=await context.newPage(),errors=[],external=[],writes=[];
 let offline=false,current=null,active=null;
 const reasons=['当前有 2 个其他速度指令来源；需先确认并释放控制权。'];
 let blockers=Object.fromEntries(modes.map(k=>[k,k==='observe'?[]:k==='flat'||k==='stairs'?[...reasons]:[...reasons,'原厂 policy 调用与实际速度响应尚未完成实机验收。']]));
 const snap=()=>({wall_time:Date.now()/1000,gait:4097,motion_state:17,hes:0,measured_velocity:[0,0,0],nav_cmd_publishers:2,state:'disarmed',accepted_gaits:[],observer:true,native_publishers_created:false});
 page.on('pageerror',e=>errors.push(e.message));
 await context.route('**/*',async route=>{
  const req=route.request(),url=new URL(req.url());
  if(url.origin!==origin){external.push(req.url());return route.abort();}
  if(!url.pathname.startsWith('/phone/native/'))return route.continue();
  if(req.method()==='POST'){
    const r=req.postDataJSON();writes.push(r);
    if(r.action==='submit'){current={id:'a'.repeat(32),key:r.key,kind:r.kind,label:r.kind==='observe'?'实机检查（模拟）':'原地普通模式（模拟）',state:r.kind==='observe'?'observing':'running',created:Date.now()/1000,message:'本机合成测试，不是实机状态',snapshot:snap()};active=current.id;}
    if(r.action==='cancel'){current.state='cancelled';current.message='已收到停止回执（合成测试）';active=null;}
    return route.fulfill({json:current,status:202});
  }
  if(offline)return route.fulfill({status:503,json:{detail:'测试网络中断'}});
  if(url.pathname.endsWith('report'))return route.fulfill({json:{physical_stop_verified:false,run:current}});
  return route.fulfill({json:{online:true,active_id:active,current,history:current?[current]:[],blockers,field_busy:false,field_error:null,routes,map_id:'0914_fr_v3-20260914-142008',policy_accepted:false,csrf:'LOCAL-ONLY'}});
 });
 try{
  await page.goto(origin+'/');await page.locator('#nativeNavigation').click();
  await page.locator('#login').waitFor({state:'visible'});await page.fill('#username','demo');await page.fill('#password','native-local');await page.locator('#login button').click();
  await page.locator('#observe').waitFor();await page.waitForFunction(()=>!document.getElementById('observe').disabled);
  assert.equal(page.url(),origin+'/native-nav');assert.equal(await page.evaluate(()=>isSecureContext),true); // loopback; randomUUID is not used.
  for(const id of ['supervisor','clear','position'])await page.locator('#'+id).check();
  assert(await page.locator('#startTest').isDisabled(),'onsite checkboxes must not override native publisher conflict');
  assert.equal(writes.length,0,'opening and checking boxes must not submit');
  for(const selector of ['#observe','#startTest','#stop','#kind'])assert((await page.locator(selector).boundingBox()).height>=48,selector+' touch target');
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'390px overflow');
  await page.screenshot({path:path.join(output,'phone-blocked.png'),fullPage:true});
  await page.locator('#observe').click();await page.waitForFunction(()=>document.getElementById('state').textContent.includes('实机检查'));
  assert(await page.locator('#observe').isDisabled());assert(!(await page.locator('#stop').isDisabled()));
  assert.equal(writes.filter(r=>r.action==='submit').length,1);
  assert.equal(writes[0].kind,'observe');assert.deepEqual(writes[0].onsite,{});
  // A second phone (or refreshed page) may observe/cancel but does not renew motion.
  const before=writes.length;await page.reload();await page.waitForTimeout(1800);
  assert.equal(writes.length,before,'reload must never submit or heartbeat on behalf of old page');
  offline=true;await page.waitForFunction(()=>document.getElementById('connection').textContent.includes('连接中断'));
  assert(await page.locator('#startTest').isDisabled());assert(!(await page.locator('#stop').isDisabled()),'stop remains available after read disconnect');
  await page.locator('#stop').click();assert.equal(writes.filter(r=>r.action==='cancel').length,1);
  offline=false;await page.waitForFunction(()=>document.getElementById('state').textContent.includes('已结束'));
  await page.locator('#kind').selectOption('start_b');await page.locator('#routePlot').waitFor({state:'visible'});
  assert((await page.textContent('#modes')).includes('普通模式 → 楼梯模式'));
  assert.equal(await page.locator('#routePlot line').count(),17);
  await page.screenshot({path:path.join(output,'phone-route.png'),fullPage:true});
  await page.setViewportSize({width:320,height:740});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'320px overflow');
  await page.screenshot({path:path.join(output,'phone-320.png'),fullPage:true});
  await page.setViewportSize({width:1280,height:900});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'desktop overflow');
  await page.screenshot({path:path.join(output,'desktop.png'),fullPage:true});
  // Explicit start owns a private heartbeat; a hidden/reloaded page never resumes it.
  blockers.flat=[];await page.locator('#kind').selectOption('flat');await page.waitForTimeout(1600);
  for(const id of ['supervisor','clear','position'])await page.locator('#'+id).check();
  await page.locator('#startTest').click();await page.waitForTimeout(1600);
  assert(writes.some(r=>r.action==='heartbeat'),'explicitly started test must keep a lease');
  await page.locator('#stop').click();
  assert.deepEqual(errors,[]);assert.deepEqual(external,[]);
  console.log(JSON.stringify({passed:true,robot_io:false,checks:['login-return','blocked-no-bypass','48px-targets','390/320/1280-no-overflow','observe','single-submit','reload-no-replay','offline-stop','route-modes','explicit-heartbeat','no-external-assets','no-js-errors'],screenshots:output}));
 }finally{await context.close();await browser.close();py.kill();}
})().catch(e=>{console.error(e);py.kill();process.exitCode=1;});
