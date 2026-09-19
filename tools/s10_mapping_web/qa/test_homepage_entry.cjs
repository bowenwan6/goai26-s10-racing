// Homepage-entry regression: LOCAL DEMO ONLY, no robot/ROS/SSH connection.
const assert=require('node:assert/strict');
const path=require('node:path');
const {chromium}=require(process.env.S10_QA_PLAYWRIGHT||'/Users/xxxwbwxxx/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const origin='http://s10-phone.invalid:18080';
(async()=>{
 const browser=await chromium.launch({headless:true});
 const context=await browser.newContext({viewport:{width:390,height:844},deviceScaleFactor:1,isMobile:true,hasTouch:true});
 const external=[],errors=[],writes=[];
 await context.route('**/*',async route=>{
  const u=route.request().url();
  if(!u.startsWith(origin)){external.push(u);return route.abort();}
  const response=await route.fetch({url:'http://127.0.0.1:18080'+u.slice(origin.length)});
  return route.fulfill({response});
 });
 const page=await context.newPage();
 page.on('pageerror',e=>errors.push(e.message));
 page.on('request',r=>{if(r.method()==='POST'&&!r.url().endsWith('/phone/login'))writes.push(r.url());});
 try{
  await page.goto(origin+'/');
  await page.locator('#login').waitFor({state:'visible'});
  const entry=page.locator('#fieldAssistant');await entry.waitFor({state:'visible'});
  assert.equal(await entry.getAttribute('href'),'/field');
  assert((await entry.textContent()).includes('现场助手'));
  assert((await entry.textContent()).includes('验图'));
  const rect=await entry.boundingBox();assert(rect.height>=46);assert(rect.y+rect.height<844,'entry must be on first phone screen');
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=document.documentElement.clientWidth),'homepage overflow');
  assert.equal(await page.evaluate(()=>isSecureContext),false);
  for(const selector of ['a[href="/localization"]','a[href="/heightmap"]','#start','#save','#maps'])assert(await page.locator(selector).count()>0);
  await page.screenshot({path:path.join(__dirname,'homepage-entry-phone.png'),fullPage:false});
  console.log('ENTRY visible before login, first-screen >=46px, strict390px, old controls PASS');
  await entry.click();await page.waitForURL(origin+'/field');
  await page.locator('#login').waitFor({state:'visible'});
  assert.equal(await page.locator('#demo').count(),0,'unauthenticated field content must not leak');
  assert.equal(await page.evaluate(async()=>{const r=await fetch('/phone/field/health');return r.status;}),401);
  await page.fill('#username','demo');await page.fill('#password','field-demo-only');
  await page.locator('#login button[type=submit]').click();
  await page.locator('#demo').waitFor({state:'visible'});
  assert.equal(page.url(),origin+'/field','login must return to field rather than homepage');
  assert(await page.locator('#selfcheck').count()>0);
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=document.documentElement.clientWidth),'field overflow');
  console.log('ENTRY unauthenticated click -> login -> field without URL loss PASS');
  await page.goto(origin+'/');await page.locator('#fieldAssistant').click();
  await page.locator('#demo').waitFor({state:'visible'});assert.equal(page.url(),origin+'/field');
  assert.equal(await page.locator('#login').count(),0);
  console.log('ENTRY already-authenticated homepage click opens field directly PASS');
  for(const [url,title] of [['/localization','定位'],['/heightmap','高度图']]){
   const response=await page.goto(origin+url);assert.equal(response.status(),200);
   const text=await page.textContent('body');assert(text.includes(title),url+' old content retained');
  }
  await page.goto(origin+'/');assert(await page.locator('#start').count()>0);assert(await page.locator('#save').count()>0);
  const response=await page.goto(origin+'/field-not-found');assert.equal(response.status(),404);
  assert((await page.textContent('body')).includes('没有这个页面'));
  assert.deepEqual(writes,[],'navigation test must not submit field, map or recording actions');
  assert.deepEqual(external,[]);assert.deepEqual(errors,[]);
  console.log('ENTRY old pages and explicit404 retained; no write actions or external requests PASS');
  console.log('HOMEPAGE_ENTRY_ALL_PASS');
 }finally{await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
