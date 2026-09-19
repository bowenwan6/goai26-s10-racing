'use strict';
// Real local Engine + FakeAdapter; every network request relayed ONLY to loopback.
const assert=require('node:assert/strict');
const path=require('node:path');
const fs=require('node:fs');
const {chromium}=require(process.env.S10_QA_PLAYWRIGHT||'<home>/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const port=Number(process.argv[2]);assert(port>0&&port<65536);
const local=`http://127.0.0.1:${port}`,origin='http://s10-phone.invalid';
const lost=process.argv[3]==='lost';
const out=path.resolve(__dirname,'../../../artifacts/field-usability-20260916');
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--no-proxy-server']});
 const context=await browser.newContext({viewport:{width:390,height:844},deviceScaleFactor:1,isMobile:true,hasTouch:true});
 let offline=false,posts=0;const errors=[],external=[];
 await context.route('**/*',async route=>{
  const u=route.request().url();if(!u.startsWith(origin+'/')){external.push(u);return route.abort();}
  if(offline)return route.abort('internetdisconnected');
  // Reproduce robot dashboard round trips slower than the former 2.5s UI TTL.
  if(u.includes('/phone/field/dashboard'))await new Promise(resolve=>setTimeout(resolve,3000));
  try{const response=await route.fetch({url:local+u.slice(origin.length)});await route.fulfill({response});}
  catch(e){if(!offline)errors.push(e.message);await route.abort();}
 });
 const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));
 page.on('request',r=>{if(r.method()==='POST'&&r.url().endsWith('/phone/field/submit'))posts++;});
 const ready=async id=>page.waitForFunction(id=>!document.getElementById(id).disabled,id,{timeout:12000});
 const text=async(id,fragment,timeout=12000)=>page.waitForFunction(([id,f])=>document.getElementById(id).textContent.includes(f),[id,fragment],{timeout});
 try{
  await page.goto(origin+'/field');await page.fill('#username','demo');await page.fill('#password','revisit-demo-only');
  await page.locator('#login button[type=submit]').click();await page.locator('#demo').waitFor({state:'visible'});
  assert.equal(await page.evaluate(()=>isSecureContext),false);
  await ready('selfcheck');await page.click('#selfcheck');await text('currentIdentity','0914_fr_v3');
  await page.check('#stationary');await page.check('#remote');await ready('localize');await page.click('#localize');
  if(lost){
    await text('jobs','30秒定位检查完成，但质量未通过',50000);await text('localizationDiagnostic','Code 3');
    assert.equal(await page.locator('#confirm').isDisabled(),true);
    assert.equal(await page.locator('#waypoint').isDisabled(),true);
    await ready('localize'); // listing/active-job snapshot may straddle task completion
    assert.equal(await page.locator('#localize').isDisabled(),false,'failed diagnosis must be repeatable');
    await page.locator('#localizationDiagnostic').screenshot({path:path.join(out,'phone-code3-diagnostic.png')});
    assert.deepEqual(errors,[]);assert.deepEqual(external,[]);
    fs.writeFileSync(path.join(out,'browser-lost-qa.json'),JSON.stringify({passed:true,demo:true,checks:['Code3 allows diagnostic collection','failed quality cannot confirm or save WP','3-second dashboard transport','specific status and age explanation']},null,2));
    console.log('LOST_LOCALIZATION_BROWSER_PASS');return;
  }
  await text('jobs','静止采样通过',42000);
  await page.click('#preview');await text('notice','抽样已加载');await page.check('#human');await ready('confirm');await page.click('#confirm');
  await text('jobs','人工叠合确认成功');
  await page.fill('#pointName','WP0');await page.fill('#floor','1F平台');
  await page.fill('#markerNote','胶带十字；同一机身参考、朝向与站高');await ready('waypoint');await page.click('#waypoint');
  await text('wpCount','已保存 1 条');
  await page.fill('#revisitNote','回到胶带十字；同一机身位置、朝向、站高');
  assert.equal(await page.locator('#revisit').isDisabled(),true);
  await page.check('#revisitPhysical');await ready('revisit');
  await page.waitForTimeout(4500);
  assert.equal(await page.locator('#revisitPhysical').isChecked(),true,'3s refresh must not clear per-attempt confirmation');
  assert.equal(await page.locator('#revisitMode').inputValue(),'nearby');
  const before=posts;await page.evaluate(()=>{document.getElementById('revisit').click();document.getElementById('revisit').click();});
  await text('busyTitle','5秒');assert.equal(posts-before,1);
  assert.equal(await page.locator('#revisit').isDisabled(),true);assert.equal(await page.locator('#waypoint').isDisabled(),true);
  await page.locator('#revisitCard').screenshot({path:path.join(out,'phone-revisit-running.png')});
  offline=true;await page.waitForTimeout(3000);offline=false;await page.reload();
  await text('revisitCount','已保存 1 次',20000);assert.equal(posts,before+1,'reload cannot replay POST');
  assert.equal(await page.locator('#revisitPhysical').isChecked(),false);
  await text('revisitResult','不是绝对精度');await text('wpCount','已保存 1 条');
  await page.locator('#revisitCard').screenshot({path:path.join(out,'phone-revisit-result.png')});
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=document.documentElement.clientWidth),true,'390px no overflow');
  await page.locator('#revisitHistory button').first().click();await text('downloads','revisit-xy.svg');
  await ready('finish');await page.click('#finish');await text('downloads','waypoint-revisits.json');
  const href=await page.locator('#downloads a').filter({hasText:'waypoint-revisits.json'}).getAttribute('href');
  const report=await page.evaluate(async u=>await (await fetch(u)).json(),href);
  assert.equal(report.trials.length,1);assert.equal(report.demo,true);assert.equal(report.absolute_accuracy_verified,false);
  assert.equal(report.trials[0].result.source_waypoint_unchanged,true);
  assert.deepEqual(errors,[]);assert.deepEqual(external,[]);
  fs.writeFileSync(path.join(out,'browser-qa.json'),JSON.stringify({passed:true,demo:true,width:390,posts,external,errors,checks:['insecure HTTP login','actual 30s check and 3s waypoint','actual 5s revisit','double-click single POST','offline reload persistence','original WP immutable','physical checkbox resets','SVG and session JSON export','no overflow']},null,2));
  console.log('REVISIT_BROWSER_ALL_PASS');
 }finally{await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
