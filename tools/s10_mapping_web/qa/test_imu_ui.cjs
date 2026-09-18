'use strict';
const fs=require('node:fs'), vm=require('node:vm'), assert=require('node:assert/strict'), path=require('node:path');
const nodes={}, noop=()=>{};
const ctx2d=new Proxy({}, {get:()=>noop,set:()=>true});
function element(id){return nodes[id]??=( {textContent:'',value:id==='window'?'30':id==='seconds'?'180':'auto',checked:false,clientWidth:300,clientHeight:200,getContext:()=>ctx2d,addEventListener:noop,replaceChildren:noop,append:noop} );}
let payload={epoch:'one',rows:[],online:true,board_mono:1,transport_age:.1,latest:{imu:{age:.01},odom:{age:.01,values:[1,2,3],frame:'map'}},error:'等待连接'};
const context=vm.createContext({document:{getElementById:element,createElement:()=>element('dynamic')},devicePixelRatio:1,
    performance:{now:()=>1000},AbortSignal:{timeout:noop},window:{addEventListener:noop},setTimeout:noop,setInterval:noop,
    fetch:async()=>({ok:true,status:200,json:async()=>payload}),console});
let source=fs.readFileSync(path.join(__dirname,'../imu_diag.js'),'utf8');
source=source.replace(/poll\(\);history\(\);\s*$/,'');
vm.runInContext(source,context);
(async()=>{
  await vm.runInContext('poll()',context);assert.equal(nodes.error.textContent,'等待连接');
  payload={...payload,error:null};await vm.runInContext('poll()',context);assert.equal(nodes.error.textContent,'');
  nodes.error.textContent='操作失败，请核对记录';await vm.runInContext('poll()',context);assert.equal(nodes.error.textContent,'操作失败，请核对记录');
  assert.equal(nodes.start.disabled,true);
  nodes.stationary.checked=nodes.remote.checked=true;vm.runInContext('render()',context);assert.equal(nodes.start.disabled,false);
  payload={...payload,transport_age:3};await vm.runInContext('poll()',context);
  assert.equal(nodes.start.disabled,true);assert.match(nodes.position.textContent,/过期/);
  payload={...payload,transport_age:.01,zero:{state:'IDLE',guard_reason:null},latest:{...payload.latest,imu:{age:.01,values:[.001,.002,.003,0,0,9.81]},body_motion:{age:.01,values:[.01,-.017,.001],motion_state:17}}};
  await vm.runInContext('poll()',context);assert.equal(nodes.zeroStart.disabled,true);
  nodes.zeroStationary.checked=nodes.zeroRemote.checked=true;vm.runInContext('render()',context);assert.equal(nodes.zeroStart.disabled,false);
  payload={...payload,zero:{state:'SAMPLING',id:'abc',elapsed_s:4}};await vm.runInContext('poll()',context);
  assert.equal(nodes.zeroStart.disabled,true);assert.equal(nodes.start.disabled,true);assert.match(nodes.zeroStatus.textContent,/4.0 \/ 13/);
  payload={...payload,zero:{state:'READY',id:'abc',remaining_s:120,gyro_relative:[.0001,.0002,.0003],motion_relative:null,motion_reason:'冻结，不复零'}};
  await vm.runInContext('poll()',context);assert.match(nodes.zeroGyroRelative.textContent,/0.000100/);assert.match(nodes.zeroBodyRelative.textContent,/未复零/);assert.match(nodes.zeroMotionReason.textContent,/冻结/);
  payload={...payload,transport_age:5};await vm.runInContext('poll()',context);assert.match(nodes.zeroGyroRelative.textContent,/未生效/);
  payload={...payload,transport_age:.01,zero:{state:'EXPIRED',id:'abc',reason:'移动后失效'}};await vm.runInContext('poll()',context);assert.equal(nodes.zeroStationary.checked,false);assert.match(nodes.zeroStatus.textContent,/失效/);
  console.log('IMU_UI_CHECK_OK 6: reconnect error, action error, confirmation gate, fresh start, stale start, stale odometry');
  console.log('ZERO_UI_CHECK_OK: per-attempt consent, 13s lock/progress, no frozen zero, stale hiding, movement invalidation');
})().catch(e=>{console.error(e);process.exitCode=1});
