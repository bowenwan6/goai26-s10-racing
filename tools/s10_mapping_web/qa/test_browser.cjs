// Independent browser checks against local DEMO ONLY server (never real robot).
const assert=require('node:assert/strict');
const path=require('node:path');
const {chromium}=require(process.env.S10_QA_PLAYWRIGHT||'/Users/xxxwbwxxx/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const origin='http://s10-phone.invalid:18080';
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--host-resolver-rules=MAP s10-phone.invalid 127.0.0.1','--no-proxy-server']});
 const context=await browser.newContext({viewport:{width:390,height:844},deviceScaleFactor:1,isMobile:true,hasTouch:true});
 let posts=0,networkDown=false;const errors=[],external=[];
 // Synthetic non-secure LAN origin is relayed only to loopback by this fixture;
 // no external DNS, proxy or robot network is used. Browser secure-context rules
 // still apply to the non-localhost HTTP origin.
 await context.route('**/*',async route=>{
  const u=route.request().url();
  if(!u.startsWith(origin)){external.push(u);return route.abort();}
  if(networkDown)return route.abort('internetdisconnected');
  const response=await route.fetch({url:'http://127.0.0.1:18080'+u.slice(origin.length)});
  await route.fulfill({response});
 });
 const page=await context.newPage();
 page.on('pageerror',e=>errors.push(e.message));
 page.on('request',r=>{if(r.method()==='POST'&&r.url().includes('/phone/field/submit'))posts++;});
 try{
  await page.goto(origin+'/field');
  await page.locator('#login').waitFor({state:'visible'});
  await page.fill('#username','demo');await page.fill('#password','field-demo-only');
  await page.locator('#login button[type=submit]').click();
  await page.locator('#demo').waitFor({state:'visible'});
  assert.equal(await page.evaluate(()=>isSecureContext),false,'must test actual insecure HTTP, not localhost secure exception');
  assert.equal(await page.evaluate(()=>typeof crypto.randomUUID),'undefined');
  assert.equal(await page.evaluate(()=>typeof navigator.serviceWorker),'undefined');
  console.log('BROWSER insecure HTTP login + demo banner PASS');
  await page.waitForFunction(()=>!document.getElementById('selfcheck').disabled);
  const before=posts;
  await page.evaluate(()=>{document.getElementById('selfcheck').click();document.getElementById('selfcheck').click();});
  await page.waitForFunction(()=>document.querySelector('#jobs .job')&& !document.getElementById('localize').disabled);
  assert.equal(posts-before,1,'double click must make one POST');
  await page.click('#preview');
  await page.waitForFunction(()=>document.getElementById('notice').textContent.includes('抽样已加载'));
  await page.click('#side');await page.click('#zoomIn');await page.click('#top');
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=document.documentElement.clientWidth),true,'phone width must not overflow (mobile innerWidth expands with overflow)');
  await page.screenshot({path:path.join(__dirname,'browser-phone.png'),fullPage:true});
  console.log('BROWSER local map, XY/XZ, controls, 390px and double click PASS');
  await page.check('#stationary');await page.check('#remote');
  await page.click('#localize');
  await page.waitForFunction(()=>document.querySelector('#jobs').textContent.includes('RUNNING'));
  const startedPosts=posts;
  networkDown=true;await context.setOffline(true);
  await page.waitForTimeout(3500);
  assert(!await page.locator('#liveQuality').evaluate(e=>e.classList.contains('good')),'offline must clear old green state');
  networkDown=false;await context.setOffline(false);
  await page.reload();
  await page.waitForFunction(()=>document.getElementById('demo')&&!document.getElementById('demo').hidden);
  await page.waitForFunction(()=>document.querySelector('#jobs').textContent.includes('localization_check'),{},{timeout:10000});
  assert.equal(posts,startedPosts,'reconnect and refresh must not replay POST');
  await page.waitForFunction(()=>[...document.querySelectorAll('#jobs .job')].some(e=>e.textContent.includes('localization_check')&&e.textContent.includes('SUCCEEDED')),{},{timeout:40000});
  console.log('BROWSER closed network + refresh restored same 30sec job without replay PASS');
  await page.click('#preview');
  await page.waitForFunction(()=>document.getElementById('notice').textContent.includes('抽样已加载'));
  await page.check('#human');await page.click('#confirm');
  await page.waitForFunction(()=>[...document.querySelectorAll('#jobs .job')].some(e=>e.textContent.includes('confirm_overlay')&&e.textContent.includes('SUCCEEDED')),{},{timeout:10000});
  await page.fill('#pointName','<img src=x onerror=alert(1)>');await page.fill('#floor','1F');await page.check('#stationary');
  await page.waitForFunction(()=>!document.getElementById('waypoint').disabled);
  await page.click('#waypoint');
  await page.waitForFunction(()=>[...document.querySelectorAll('#jobs .job')].some(e=>e.textContent.includes('waypoint')&&e.textContent.includes('SUCCEEDED')),{},{timeout:12000});
  assert.equal(await page.locator('#detail img').count(),0);
  assert((await page.locator('#detail').textContent()).includes('onerror'),'user label should render as text');
  await page.waitForFunction(()=>!document.getElementById('finish').disabled);await page.click('#finish');
  await page.waitForFunction(()=>[...document.querySelectorAll('#jobs .job')].some(e=>e.textContent.includes('finish')&&e.textContent.includes('SUCCEEDED')),{},{timeout:12000});
  await page.waitForFunction(()=>{const t=document.getElementById('detail').textContent;return t.includes('"action": "finish"')&&t.includes('"state": "SUCCEEDED"');},{},{timeout:5000});
  assert((await page.locator('#detail').textContent()).includes('"passed": false'),'demo incomplete results may never pretend acceptance');
  assert(await page.locator('#downloads a').count()>0);
  console.log('BROWSER confirmation, fresh draft, escaped text, explicit failed-quality report/download PASS');
  await page.evaluate(()=>localStorage.setItem('s10-field-session','0'.repeat(32)));
  await page.reload();
  await page.waitForFunction(()=>document.getElementById('selfcheck')&&!document.getElementById('selfcheck').disabled,{},{timeout:10000});
  console.log('BROWSER nonexistent saved session is recoverable PASS');
  await page.route('**/phone/field/list',async route=>{
   const response=await route.fetch({url:'http://127.0.0.1:18080/phone/field/list'}),data=await response.json();
   for(const j of data.jobs)if(j.action==='selfcheck'&&j.result)j.result.checks.push({name:'候选地图',state:'pass',detail:'1209_01_F-20260912-175844:sha256:'+ 'abc123'.repeat(11)});
   await route.fulfill({response,json:data});
  });
  await page.waitForTimeout(2500);
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=document.documentElement.clientWidth),true,'real-length map hash must wrap on phone');
  console.log('BROWSER long real-style map identity phone width PASS');
  assert.deepEqual(errors,[]);assert.deepEqual(external,[]);
  console.log('BROWSER_ALL_PASS');
 }finally{await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
