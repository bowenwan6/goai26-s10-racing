'use strict';
const $=id=>document.getElementById(id), colors=['#c84639','#158160','#315de0'];
let latest=null, cursor=0,epoch=null,rows=[],busy=false,csrf='',received=0,oldActive=null,lastStreamError='',oldZeroState=null;
const num=x=>typeof x==='number'&&Number.isFinite(x),fmt=(x,n=3)=>num(x)?x.toFixed(n):'不可用';
async function get(path){const r=await fetch(path,{cache:'no-store',signal:AbortSignal.timeout(10000)});if(r.status===401)throw Error('请返回首页登录，再进入IMU诊断。');const d=await r.json();if(!r.ok)throw Error(d.detail||'读取失败');return d;}
const travel=()=>Math.max(0,latest?.transport_age||0)+(performance.now()-received)/1000;
function fresh(){return !!latest?.online && performance.now()-received<2000 && num(latest?.latest?.imu?.age)&&latest.latest.imu.age+travel()<1;}
function zeroRender(){
  const z=latest?.zero,state=z?.state,working=['SAMPLING','VALIDATING'].includes(state),isFresh=fresh();
  const remain=(z?.remaining_s||0)-travel(),ready=state==='READY'&&isFresh&&!z?.guard_reason&&remain>0;
  const triplet=v=>Array.isArray(v)&&v.length===3&&v.every(num)?v.map(x=>fmt(x,6)).join(' / '):'不可用';
  $('zeroStart').disabled=busy||!isFresh||!z||!!latest?.active||working||!!z.guard_reason||!$('zeroStationary').checked||!$('zeroRemote').checked;
  $('zeroClear').disabled=busy||!latest?.online||!z?.id||!['SAMPLING','VALIDATING','READY'].includes(state);
  $('zeroExport').disabled=!z?.id||working;
  $('zeroStationary').disabled=busy||working;$('zeroRemote').disabled=busy||working;
  $('zeroProgress').value=working?z.elapsed_s||0:state==='READY'?13:0;
  const titles={IDLE:'尚未复零',SAMPLING:'正在采样',VALIDATING:'正在独立验证',READY:'本次显示参考已建立',FAILED:'未能建立零点',EXPIRED:'本次零点已失效',CLEARED:'零点已清除'};
  $('zeroStatus').textContent=!latest?.online?'连接中断／连接中：相对值已隐藏，请先恢复连接核对任务状态':
    !z?'复零服务未就绪，请等待连接或确认已部署新版':
    `${titles[state]||state}：${working?`${fmt(z.elapsed_s||0,1)} / 13 秒，继续保持不动。` : ready?`剩余 ${Math.max(0,Math.floor(remain))} 秒。` : ''}${z.reason||''}${!working&&state!=='READY'&&z.guard_reason?'；当前阻断：'+z.guard_reason:''}`;
  if(state==='READY'&&!ready)$('zeroStatus').textContent='参考暂不可用／已过期：相对值已隐藏。'+(z.guard_reason||'请等待新鲜数据或重新复零');
  $('zeroGyroRaw').textContent=isFresh?triplet(latest.latest.imu.values?.slice(0,3)):'数据过期／不可用';
  $('zeroGyroRelative').textContent=ready?triplet(z.gyro_relative):'未生效（不补0）';
  const b=latest?.latest?.body_motion,bodyFresh=latest?.online&&num(b?.age)&&b.age+travel()<.7;
  $('zeroBodyRaw').textContent=bodyFresh?triplet(b.values)+` · state=${b.motion_state}`:'数据过期／未接入';
  $('zeroBodyRelative').textContent=ready&&bodyFresh&&z.motion_relative?triplet(z.motion_relative):'未复零（不补0）';
  $('zeroMotionReason').textContent=z?.motion_reason||'本体速度需要新鲜、持续更新的 state=17 数据；Idle／冻结值不会被强行归零。下方所有曲线仍显示原值。';
}
function draw(id,topic,axes,reference){
  const c=$(id),ctx=c.getContext('2d'),w=c.clientWidth,h=c.clientHeight,dpr=Math.min(devicePixelRatio||1,2);c.width=w*dpr;c.height=h*dpr;ctx.scale(dpr,dpr);
  const end=(latest?.board_mono||0)+travel(),span=Number($('window').value),start=end-span;
  const data=rows.filter(r=>r.topic===topic&&r.t>=start&&r.t<=end);let lo=-reference,hi=reference;
  if($('scale').value==='auto'){const values=data.flatMap(r=>axes.flatMap(i=>[r.min[i],r.max[i]])).filter(num);if(values.length){lo=Math.min(0,...values);hi=Math.max(0,...values);const pad=Math.max((hi-lo)*.12,reference*.005);lo-=pad;hi+=pad;}}
  const x=t=>50+(t-start)/span*(w-62),y=v=>h-24-(v-lo)/(hi-lo)*(h-40);ctx.fillStyle='#526b7a';ctx.font='11px system-ui';ctx.strokeStyle='#dce5eb';ctx.lineWidth=1;
  const digits=Math.min(6,Math.max(3,Math.ceil(-Math.log10(hi-lo))+1));
  for(let i=0;i<=4;i++){const v=lo+(hi-lo)*i/4;ctx.beginPath();ctx.moveTo(50,y(v));ctx.lineTo(w-12,y(v));ctx.stroke();ctx.fillText(v.toFixed(digits),1,y(v)+4);}
  ctx.fillText(`−${span}s`,50,h-5);ctx.fillText('现在',w-38,h-5);
  axes.forEach((axis,j)=>{ctx.strokeStyle=colors[j%3];ctx.lineWidth=1;ctx.setLineDash(j>=3?[5,4]:[]);ctx.beginPath();let prev=null;
    for(const r of data){const v=r.mean[axis];if(!num(v)){prev=null;continue;}if(prev&&!r.discontinuity&&r.t-prev.t<(topic==='motion'?1.8:.4)){ctx.lineTo(x(r.t),y(v));}else ctx.moveTo(x(r.t),y(v));prev=r;}ctx.stroke();ctx.setLineDash([]);ctx.globalAlpha=.35;
    for(const r of data){if(!num(r.min[axis])||!num(r.max[axis]))continue;ctx.beginPath();ctx.moveTo(x(r.t),y(r.min[axis]));ctx.lineTo(x(r.t),y(r.max[axis]));ctx.stroke();}ctx.globalAlpha=1;});
  if(!data.length){ctx.fillStyle='#647985';ctx.fillText('暂无有效数据；不会补零或绘制假曲线',60,h/2);}
}
function render(){
  const active=latest?.active,isFresh=fresh();$('connection').textContent=isFresh?'收到新鲜IMU数据':'数据未就绪／已过期';
  $('age').textContent=latest?.online?fmt(latest?.latest?.imu?.age+travel(),2)+' 秒':'连接中断';$('drops').textContent=latest?.queue_dropped??'—';
  const tail=rows.filter(r=>r.topic==='imu'&&r.t>=(latest?.board_mono||0)-10);const seconds=tail.length>1?tail.at(-1).t-tail[0].t+.1:0;
  $('hz').textContent=!isFresh?'数据已过期':seconds>=1?fmt(tail.reduce((n,r)=>n+r.n,0)/seconds,1)+' Hz':'积累样本中';
  $('start').disabled=busy||!isFresh||!!active||['SAMPLING','VALIDATING'].includes(latest?.zero?.state)||!$('stationary').checked||!$('remote').checked;
  for(const id of ['stop','stillEvent','moveEvent'])$(id).disabled=busy||!active||!latest?.online;
  $('seconds').disabled=busy||!!active;$('progress').max=active?.seconds||Number($('seconds').value);$('progress').value=active?.elapsed_s||0;
  $('progressText').textContent=active?`采集中：${Math.min(Math.floor(active.elapsed_s||0),active.seconds)} / ${active.seconds} 秒 · 已记录 ${active.samples} 条 · ${active.id.slice(0,8)}`:busy?'正在提交，请勿重复点击…':'当前无采集任务。查看曲线不改变机器人状态。';
  const imu=latest?.latest?.imu,q=imu?.quaternion,cv=imu?.orientation_covariance;
  if(isFresh&&q?.length===4&&q.every(num)&&cv?.[0]!==-1&&Math.abs(q.reduce((s,v)=>s+v*v,0)-1)<.05){
    const [x,y,z,w]=q;const r=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),p=Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x)))),a=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    $('attitude').textContent=`IMU姿态（非校准证明）：roll ${fmt(r*180/Math.PI,1)}° / pitch ${fmt(p*180/Math.PI,1)}° / yaw ${fmt(a*180/Math.PI,1)}°；协方差为0表示未知。`;
  }else $('attitude').textContent='姿态不可用、未收到或已过期；不把无效四元数当作零姿态。';
  const od=latest?.latest?.odom;$('position').textContent=latest?.online&&num(od?.age)&&od.age+travel()<1?`/ODOM位置：${od.values.map(v=>fmt(v)).join(' / ')}；坐标系 ${od.frame||'未知'}（融合值，不是现场真值；不证明全局定位有效）`:'位姿未接入或已过期。';
  draw('gyro','imu',[0,1,2],.05);draw('accel','imu',[3,4,5],12);draw('motion','motion',[0,1,2,3,4,5],.05);
  zeroRender();
}
async function history(){try{const d=await get('/phone/imu/list');$('history').replaceChildren();for(const s of d.sessions){const b=document.createElement('button');b.textContent=`${new Date(s.started_wall*1000).toLocaleString()} · ${s.state} · ${s.samples}条 · ${s.id.slice(0,8)}`;b.onclick=()=>report(s.id);$('history').append(b);}if(!d.sessions.length)$('history').textContent='尚无诊断记录';}catch(e){$('error').textContent=e.message;}}
function reportSummary(d){
  const s=d.session||{},states={COMPLETED:'采集时限完成（不等于IMU验收通过）',PARTIAL:'提前结束／部分采集',INTERRUPTED:'进程重启中断',FAILED:'采集失败',RUNNING:'仍在采集'};
  const lines=[states[s.state]||s.state||'状态未知',`已保存 ${s.samples||0} 条；队列丢弃 ${s.queue_dropped||0} 条`,'校准状态：未验证；真实静止状态：未确定。'];
  if(!d.windows)return lines.concat(d.conclusion||'尚未生成报告').join('\n');
  lines.push(`无效分量 ${d.invalid_components}；时间异常标记 ${d.time_warnings}`,`未采到的话题：${(d.missing_topics||[]).join(' / ')||'无（仅表示收到，不证明健康）'}`);
  for(const [topic,t] of Object.entries(d.timing||{}))lines.push(`${topic}：${d.topic_counts[topic]}条；最大接收间隔 ${fmt(t.max_receive_gap_s,3)}秒；末尾缺测 ${fmt(t.end_missing_s,2)}秒`);
  const ws=d.windows.filter(w=>w.topic==='imu');if(ws.length){lines.push('\nIMU：首／末10秒窗口均值；末窗口标准差');const a=ws[0],b=ws.at(-1);['wx','wy','wz','ax','ay','az'].forEach((name,i)=>lines.push(`${name}：${fmt(a.axes[i]?.mean,6)} → ${fmt(b.axes[i]?.mean,6)}；波动 ${fmt(b.axes[i]?.std,6)}`));}
  lines.push('\n均值变化不单独证明零偏故障；需结合人工停稳、命令、姿态与单位。加速度可能包含重力，不以三轴为0作为目标。');return lines.join('\n');
}
async function report(id){try{const d=await get('/phone/imu/report?id='+encodeURIComponent(id));$('selection').textContent='诊断 '+id;$('report').textContent=reportSummary(d);$('reportRaw').textContent=JSON.stringify(d,null,2);$('download').href='/phone/imu/download?id='+encodeURIComponent(id);$('download').hidden=!d.windows;}catch(e){$('error').textContent=e.message;}}
async function submit(body){busy=true;render();try{const r=await fetch('/phone/imu/submit',{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:JSON.stringify(body),signal:AbortSignal.timeout(20000)});const d=await r.json();if(!r.ok)throw Error(d.detail||'提交失败');$('error').textContent='';if(body.action==='start'){localStorage.removeItem('imuPendingStart');latest.active=d;}if(body.action==='stop'){latest.active=null;await history();await report(d.id);}if(body.action==='zero_start'||body.action==='zero_clear'){latest.zero=d;localStorage.removeItem('imuPendingZero');$('zeroStationary').checked=$('zeroRemote').checked=false;}}catch(e){if(body.action==='zero_start'){$('zeroStationary').checked=$('zeroRemote').checked=false;}$('error').textContent=e.message+'；先核对任务状态。开始请求重试会复用编号，不重复采集。';}finally{busy=false;render();}}
$('start').onclick=()=>{if($('start').disabled)return;let request;try{request=JSON.parse(localStorage.getItem('imuPendingStart')||'null');}catch{}if(!request){request={action:'start',key:Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join(''),seconds:Number($('seconds').value),stationary:true,remote_ready:true};localStorage.setItem('imuPendingStart',JSON.stringify(request));}submit(request);};
$('stop').onclick=()=>latest?.active&&submit({action:'stop',id:latest.active.id});
for(const [id,label] of [['stillEvent','停稳'],['moveEvent','移动']])$(id).onclick=()=>latest?.active&&submit({action:'event',id:latest.active.id,label});
for(const id of ['stationary','remote','seconds','window','scale','zeroStationary','zeroRemote'])$(id).addEventListener('change',render);$('refresh').onclick=history;
$('zeroStart').onclick=()=>{if($('zeroStart').disabled)return;let request;try{request=JSON.parse(localStorage.getItem('imuPendingZero')||'null');}catch{}if(request?.epoch!==latest?.epoch)request=null;if(!request){request={action:'zero_start',key:Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join(''),epoch:latest.epoch,stationary:true,remote_ready:true};localStorage.setItem('imuPendingZero',JSON.stringify(request));}submit(request);};
$('zeroClear').onclick=()=>{if(!$('zeroClear').disabled)submit({action:'zero_clear',id:latest.zero.id});};
$('zeroExport').onclick=()=>{if($('zeroExport').disabled)return;const b=new Blob([JSON.stringify(latest.zero,null,2)],{type:'application/json'}),u=URL.createObjectURL(b),a=document.createElement('a');a.href=u;a.download='display-reference-'+latest.zero.id+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(u),1000);};
async function poll(){try{const d=await get('/phone/imu/live?cursor='+cursor);if(epoch!==d.epoch){rows=[];cursor=0;epoch=d.epoch;$('zeroStationary').checked=$('zeroRemote').checked=false;}for(const r of d.rows||[])if(r.seq>cursor){rows.push(r);cursor=r.seq;}rows=rows.filter(r=>r.t>=(d.board_mono||0)-125).slice(-3600);latest=d;csrf=d.csrf||csrf;received=performance.now();if(oldZeroState!==d.zero?.state&&['EXPIRED','FAILED','CLEARED'].includes(d.zero?.state)){$('zeroStationary').checked=$('zeroRemote').checked=false;}oldZeroState=d.zero?.state||null;if(d.zero?.id){try{if(JSON.parse(localStorage.getItem('imuPendingZero')||'null')?.key===d.zero.id)localStorage.removeItem('imuPendingZero');}catch{}}if(d.error){lastStreamError=d.error;$('error').textContent=d.error;}else if($('error').textContent===lastStreamError){$('error').textContent='';lastStreamError='';}if(oldActive&&!d.active){await history();await report(oldActive);}oldActive=d.active?.id||null;}catch(e){if(latest)latest.online=false;$('zeroStationary').checked=$('zeroRemote').checked=false;lastStreamError=e.message;$('error').textContent=e.message;}render();setTimeout(poll,250);}
window.addEventListener('resize',render);setInterval(render,500);poll();history();
