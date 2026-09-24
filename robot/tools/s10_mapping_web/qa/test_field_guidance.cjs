'use strict';
const assert=require('node:assert/strict');
const G=require('../field.js');
let count=0;
function test(name,fn){fn();count++;console.log('PASS '+name);}
function healthy(){
  const b={robot_id:'robot',boot_id:'boot',map_identity:'v3-sha',invocation:'loc'};
  const check={id:'check',action:'localization_check',state:'SUCCEEDED',updated:990,result:{passed:true,binding:b}};
  return {connected:true,contract:2,demo:false,fresh:true,features:['field_workflow_v2'],now:1000,stationary:true,remote:true,human:true,name:'WP01',floor:'1F',cloudReady:true,
    session:{target:'v3',binding:b,eligible_check:'check',approved_check:'check',approved_at:995},
    overview:{selfcheck:{state:'SUCCEEDED',result:{passed:true}},eligible_job:check},
    live:{...b,map_name:'v3',mapping_active:false,localization_active:true,status:{fresh:true,code:0,mode:'全局'},pose:{frame:'map',xyz:[1,2,3],age:.1,stamp_age_s:.1},navigation:{idle:true}}};
}
test('fresh approved same map can record and save draft',()=>{const g=G.workflow(healthy());assert.equal(g.gates.record,'');assert.equal(g.gates.waypoint,'');});
test('old indoor map blocks every downstream data action',()=>{const x=healthy();x.live.map_name='indoor';const g=G.workflow(x);for(const k of ['localize','confirm','record','waypoint'])assert.ok(g.gates[k]);assert.match(g.title,/尚未加载/);});
test('selfcheck succeeded but passed false blocks load',()=>{const x=healthy();x.overview.selfcheck.result.passed=false;assert.ok(G.workflow(x).gates.load);});
test('map file identity change cannot borrow current map name',()=>{const x=healthy();x.live.map_identity='changed-sha';assert.ok(G.workflow(x).gates.waypoint);});
test('reboot and localization restart invalidate UI continuation',()=>{for(const key of ['boot_id','invocation']){const x=healthy();x.live[key]='new';assert.ok(G.workflow(x).gates.record);}});
test('loss and stale live evidence block data but keep failure export',()=>{for(const type of ['loss','stale','missing']){const x=healthy();if(type==='loss')x.live.status.code=3;if(type==='stale')x.fresh=false;if(type==='missing')x.live=null;const g=G.workflow(x);assert.ok(g.gates.record);assert.ok(g.gates.waypoint);assert.equal(g.gates.finish,'');}});
test('revoked eligibility cannot use persisted old approval',()=>{const x=healthy();x.session.eligible_check=null;x.overview.eligible_job=null;assert.ok(G.workflow(x).gates.waypoint);});
test('expired approval requires a new check',()=>{const x=healthy();x.session.approved_at=-1000;assert.ok(G.workflow(x).gates.record);});
test('static check only can confirm but not record',()=>{const x=healthy();x.session.approved_check=null;const g=G.workflow(x);assert.equal(g.gates.confirm,'');assert.ok(g.gates.record);});
test('old static check and unreviewed cloud cannot confirm',()=>{for(const old of [true,false]){const x=healthy();if(old)x.overview.eligible_job.updated=600;else x.cloudReady=false;assert.ok(G.workflow(x).gates.confirm);}});
test('human stop and policy-exited declarations do not bypass nonzero feedback',()=>{const x=healthy();x.live.map_name='old';x.live.navigation.idle=false;assert.ok(G.workflow(x).gates.load);});
test('missing waypoint name floor or stationary declaration blocks save',()=>{for(const key of ['name','floor','stationary']){const x=healthy();x[key]=key==='stationary'?false:'';assert.ok(G.workflow(x).gates.waypoint);}});
test('unavailable or old backend fails closed',()=>{for(const key of ['connected','contract']){const x=healthy();x[key]=false;assert.ok(G.workflow(x).gates.load);assert.ok(G.workflow(x).gates.record);}});
test('finish succeeded is not an acceptance pass',()=>{const r=G.result({action:'finish',state:'SUCCEEDED',result:{passed:false,recordings:[],valid_waypoint_count:0,saved_waypoint_count:3,draft_waypoint_count:3,reasons:['没有合格录制']}});assert.match(r.title,/验收未通过/);assert.match(r.detail,/草稿 3 条/);assert.equal(r.tone,'bad');});
test('failed point explicitly has no saved coordinates',()=>{assert.match(G.result({action:'waypoint',state:'FAILED',error:'wrong map'}).title,/未保存坐标/);});
test('draft saved is distinct from navigation readiness',()=>{const r=G.result({action:'waypoint',state:'SUCCEEDED',result:{name:'WP01',floor:'1F',draft:true,pose:{xyz:[1,2,3]}}});assert.match(r.title,/草稿保存成功/);assert.match(r.detail,/不可直接导航/);});
test('queued task is not falsely described as started',()=>{assert.match(G.result({action:'record',state:'QUEUED'}).title,/尚未开始/);});
test('record completed with bad quality is not a pass',()=>{assert.match(G.result({action:'record',state:'SUCCEEDED',result:{passed:false}}).title,/质量未通过/);});
test('real sample progress uses backend samples rather than a browser countdown',()=>{const p=G.progress({action:'localization_check',state:'RUNNING',duration_seconds:30,stage:'采样 5/30 秒'});assert.equal(p.value,5);assert.equal(p.max,30);assert.match(p.title,/停稳/);});
test('record duration does not create a fake remaining-time estimate',()=>{const p=G.progress({action:'record',state:'RUNNING',duration_seconds:10,stage:'正在106本地录制10秒'});assert.equal(p.value,null);assert.match(p.title,/10秒/);});
test('3-second sample completion remains busy while saving',()=>{const p=G.progress({action:'waypoint',state:'RUNNING',duration_seconds:3,stage:'采样结束，正在检查数据与保存结果'});assert.equal(p.value,null);assert.match(p.title,/继续等待/);});
test('queued phase cannot claim elapsed sampling and terminal phase hides progress',()=>{assert.equal(G.progress({action:'waypoint',state:'QUEUED',duration_seconds:3}).value,null);assert.equal(G.progress({state:'FAILED'}),null);});
function revisitReady(){const x=healthy();return {...x,features:['waypoint_revisit_v1'],revisitPoint:{binding:x.session.binding},revisitPhysical:true,revisitNote:'胶带十字，同一机身参考'};}
test('revisit requires capable backend and fresh per-attempt physical confirmation',()=>{
  assert.equal(G.workflow(revisitReady()).gates.revisit,'');
  for(const key of ['features','revisitPoint','revisitPhysical','remote','stationary']){
    const x=revisitReady();x[key]=key==='features'?[]:null;assert.ok(G.workflow(x).gates.revisit,key);
  }
});
test('revisit rejects wrong map and revoked approval but can compare old run after fresh approval',()=>{
  let x=revisitReady();x.revisitPoint={binding:{...x.session.binding,boot_id:'old'}};assert.equal(G.workflow(x).gates.revisit,'');
  x.revisitPoint.binding.map_identity='other-map';assert.ok(G.workflow(x).gates.revisit);
  x=revisitReady();x.session.eligible_check=null;assert.ok(G.workflow(x).gates.revisit);
});
test('5-second revisit exposes progress and does not assert absolute accuracy',()=>{
  const p=G.progress({action:'waypoint_revisit',state:'RUNNING',duration_seconds:5,stage:'采样 2/5 秒'});assert.equal(p.max,5);assert.equal(p.value,2);
  const r=G.result({action:'waypoint_revisit',state:'SUCCEEDED',result:{source_name:'WP0',metrics:{horizontal_m:.05,vertical_m:-.01,yaw_deg:2},within_reference:true,reference_verified:false}});
  assert.match(r.detail,/5.0 cm/);assert.match(r.detail,/-1.0 cm/);assert.match(r.detail,/不是绝对精度/);assert.equal(r.tone,'unknown');
});
test('fresh Code3 can run diagnosis without unlocking confirmation or WP',()=>{
  const x=healthy();x.live.status={fresh:true,code:3,mode:'局部',label:'定位丢失'};
  const g=G.workflow(x);assert.equal(g.gates.localize,'');assert.ok(g.gates.confirm);assert.ok(g.gates.waypoint);
  assert.match(g.detail,/Code 3/);assert.match(g.detail,/0.100 秒/);assert.doesNotMatch(g.detail,/位姿接收\/源时间未通过/);
});
test('stale pose can be diagnosed but cannot be approved',()=>{
  const x=healthy();x.live.pose.age=1;const g=G.workflow(x);assert.equal(g.gates.localize,'');assert.ok(g.gates.confirm);
  assert.match(g.detail,/0.5秒数据检查/);
});
test('3s transport does not expire UI but 8s or invalid age does',()=>{
  assert(G.liveFresh(3100));assert(G.liveFresh(7900));assert(!G.liveFresh(8000));assert(!G.liveFresh(NaN));assert(!G.liveFresh(-1));
});
test('optional revisit note and different heading do not block coordinate comparison',()=>{
  const x=revisitReady();x.revisitNote='';x.live.pose.quaternion=[0,0,1,0];assert.equal(G.workflow(x).gates.revisit,'');
});
test('recording selfcheck failures do not prevent read-only localization diagnosis or saved draft',()=>{
  const x=healthy();x.overview.selfcheck.result.passed=false;
  const g=G.workflow(x);assert.equal(g.gates.localize,'');assert.equal(g.gates.waypoint,'');assert.ok(g.gates.record);
});
console.log(`FIELD_GUIDANCE_CHECK_OK ${count}`);
