'use strict';
const $=id=>document.getElementById(id);
let data=null,csrf='',busy=false,connected=false,received=0,owned=null,heartPending=false,stopping=false,connectionError=false;
const states={starting:'正在准备',observing:'正在实机检查 · 不运动',running:'测试进行中',stopping:'正在结束测试',completed:'本次采集／测试已结束',cancelled:'本次测试已结束',failed:'本次测试未通过',interrupted:'上次测试曾中断'};
const modeNames={flat:'普通模式',stairs:'楼梯模式'};
const gaitName=n=>({4097:'普通模式',4098:'高台模式',4099:'楼梯模式',12290:'导航普通模式',12291:'导航楼梯模式'}[n]||(n==null?'尚未读取':'未知模式 · '+n));
const uuid=()=>Array.from(crypto.getRandomValues(new Uint8Array(16)),n=>n.toString(16).padStart(2,'0')).join('');
const text=(id,value)=>{$(id).textContent=value;};
async function api(path,body){
  const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),10000);
  try{const r=await fetch(path,{method:body?'POST':'GET',headers:body?{'Content-Type':'application/json','X-CSRF-Token':csrf}:{},body:body?JSON.stringify(body):undefined,cache:'no-store',signal:abort.signal});
    let j;try{j=await r.json();}catch{throw Error('响应不完整，请查询当前记录，避免重复启动。');}
    if(r.status===401||r.status===403){connected=false;throw Error('登录已过期，请返回首页重新登录；当前运动测试将因心跳中断请求停止。');}
    if(!r.ok)throw Error(j.detail||'请求未完成');return j;
  }finally{clearTimeout(timer);}
}
function chosen(){return $('kind').value;}
function drawRoute(){
  const kind=chosen(),stationary=['flat','stairs'].includes(kind),route=data?.routes.find(r=>r.id===(stationary?'start':kind));
  text('scope',stationary?'只发零速度并核对原厂模式回执，约 12 秒。站姿可能变化。':kind==='start'?'从 Start 行走至 B 区入口。限速 0.20 m/s。':kind==='b'?'从 B 区入口到上方平台，包含三段楼梯。':'从 Start 经平地和三段楼梯，到 B 区上方平台结束。');
  text('modes',stationary?modeNames[kind]+' · 零速度':(route?.modes||[]).map(m=>modeNames[m]).join(' → '));
  text('positionLabel',stationary?'机器人站稳，适合原地切换模式':kind==='b'?'机器人位于 v3 的 B 区入口，朝向与路线一致':'机器人位于 v3 的 Start，朝向与路线一致');
  text('startTest',stationary?'开始原地模式测试':'开始所选路线测试');
  $('routePlot').hidden=stationary;$('routeHint').hidden=stationary;
  if(!route||stationary)return;
  text('routeHint',route.points.length+' 个航点 · 平地上限 0.20 m/s · 楼梯上限 0.15 m/s');
  const svg=$('routePlot');svg.replaceChildren();
  const pts=route.points.map(p=>p.position),xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]),minX=Math.min(...xs),minY=Math.min(...ys),spanX=Math.max(...xs)-minX||1,spanY=Math.max(...ys)-minY||1,scale=Math.min(260/spanX,130/spanY),project=p=>[45+(p[0]-minX)*scale,155-(p[1]-minY)*scale];
  const make=(name,attrs,content)=>{const el=document.createElementNS('http://www.w3.org/2000/svg',name);for(const [k,v] of Object.entries(attrs))el.setAttribute(k,String(v));if(content)el.textContent=content;svg.append(el);};
  for(let i=1;i<pts.length;i++){const a=project(pts[i-1]),b=project(pts[i]);make('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:route.points[i].kind==='stairs'?'#aa660e':'#166b54','stroke-width':4});}
  for(const [p,label] of [[pts[0],kind==='b'?'B 入口':'Start'],[pts.at(-1),kind==='start'?'B 入口':'B 上平台']]){const [x,y]=project(p);make('circle',{cx:x,cy:y,r:5,fill:'#203e30'});make('text',{x:Math.min(280,x+8),y:y-10},label);}
}
function render(){
  const live=connected&&Date.now()-received<5000,current=data?.current,active=Boolean(data?.active_id),kind=chosen(),reasons=[...(data?.blockers[kind]||[])];
  if(data?.field_busy)reasons.unshift('现场助手正在执行任务，请等待完成。');
  if(data?.field_error)reasons.unshift(data.field_error);
  $('connection').dataset.tone=live?(data.policy_accepted?'good':'pending'):'bad';text('connection',live?'已连接 48 号测试服务 · '+(data.policy_accepted?'已有原厂调用验收记录':'原厂行走调用仍待实机验收'):'连接中断或状态尚未更新；启动已禁用。请用遥控器核对，勿重复提交。');
  text('state',current?.kind==='observe'&&current?.state==='completed'?'检查完成 · 尚未运动':current?states[current.state]||'状态待核对':'尚未开始');text('statusDetail',current?.message||'先运行实机检查，查看当前模式、反馈和控制权。');
  const snap=current?.snapshot,age=snap?((data.server_time||received/1000)-snap.wall_time+(Date.now()-received)/1000):null;
  const context=snap?.map_context;
  text('mapStatus',context?'当前地图：'+(context.map_id||'未读取')+' · '+(context.global_mode?'全局定位':'尚未确认全局定位'):'当前地图与全局定位：等待实机检查。');
  text('gait',gaitName(snap?.gait));text('speed',Number.isFinite(snap?.measured_velocity?.[0])?snap.measured_velocity[0].toFixed(3)+' m/s':'—');
  text('age',age==null?'尚无读数':age<0?'时间异常':age<3&&live?'刚刚更新':Math.floor(age)+' 秒前 · 历史');
  const route=data?.routes.find(r=>r.id===current?.kind),index=snap?.target,progress=route&&Number.isInteger(index)?Math.min(route.points.length,Math.max(0,index)):0;
  const percent=route?Math.round(progress/route.points.length*100):0;$('bar').style.width=percent+'%';$('progress').setAttribute('aria-valuenow',percent);
  text('progressText',route?`已通过 ${progress} / ${route.points.length} 个航点 · ${live&&active?'采集中':'历史记录'}`:'原地切换和实机检查没有行走路线进度。');
  $('blockers').replaceChildren();for(const value of reasons){const li=document.createElement('li');li.textContent=value;$('blockers').append(li);}
  text('readyTitle',reasons.length?'以下条件未满足，暂不能启动所选测试：':'技术预检暂无阻止项；启动时将再次检查实时反馈。');
  const confirmed=['supervisor','clear','position'].every(k=>$(k).checked);
  $('observe').disabled=!live||busy||active||Boolean(data?.field_busy||data?.field_error);
  $('kind').disabled=busy||active;
  $('startTest').disabled=!live||busy||active||reasons.length>0||!confirmed;
  text('startHelp',active?'已有测试正在运行，需等待结束或停止。':reasons.length?'先完成上方条件；现场勾选不会绕过这些检查。':!confirmed?'请逐项确认三项现场条件。':'点击后可能改变站姿或开始行走，请保持本页在线。');
  // Stop stays usable after a transport failure using the last known run id.
  $('stop').disabled=!active||stopping;text('stopHint',stopping?'正在提交停止；若没有及时回执，请立即用遥控器接管。':'软件停止不是遥控器急停。现场接管优先。');
  text('diagnostics',JSON.stringify({map_id:data?.map_id,run:current},null,2));
  const history=$('history');history.replaceChildren();
  for(const r of (data?.history||[]).slice(0,5)){const div=document.createElement('div');div.className='history-item';const p=document.createElement('p');p.textContent=`${r.label} · ${states[r.state]||r.state}`;const small=document.createElement('small');small.textContent=new Date(r.created*1000).toLocaleString('zh-CN');const br=document.createElement('br');const a=document.createElement('a');a.href='/phone/native/report?id='+r.id;a.download='navigation-'+r.id+'.json';a.textContent='下载本次结果（JSON）';div.append(p,small,br,a);history.append(div);}
  if(!data?.history.length)text('history','尚无测试记录。');
}
async function refresh(){
  try{data=await api('/phone/native/status');csrf=data.csrf;connected=true;received=Date.now();
    if(connectionError){text('message','');connectionError=false;}
    if(owned&&data.current?.key===owned.key){owned.id=data.current.id;if(!data.active_id)owned=null;}
    drawRoute();
  }catch(e){connected=false;connectionError=true;text('message',e.name==='AbortError'?'读取超时；启动已禁用，稍后自动重连。':e.message);}
  render();setTimeout(refresh,1500);
}
async function submit(kind){
  if(busy)return;busy=true;text('message','正在提交；结果未确认前请勿重复点击。');render();
  const key=uuid(),owner=uuid();owned=kind==='observe'?null:{key,owner,id:null};
  try{const row=await api('/phone/native/submit',{action:'submit',kind,key,owner,onsite:kind==='observe'?{}:{supervisor:$('supervisor').checked,clear:$('clear').checked,position:$('position').checked}});
    if(owned)owned.id=row.id;
    if(data){data.current=row;data.active_id=row.id;data.history=[row,...data.history.filter(r=>r.id!==row.id)];}
    text('message',kind==='observe'?'检查已受理，正在读取实机状态。':'测试请求已受理，等待控制程序核对；请保持页面在线。');
  }catch(e){text('message',e.name==='AbortError'?'未确认此次请求结果。请查看当前记录；不会自动重发。':e.message);}
  finally{busy=false;for(const k of ['supervisor','clear','position'])$(k).checked=false;render();}
}
$('observe').onclick=()=>submit('observe');$('startTest').onclick=()=>submit(chosen());
$('kind').onchange=()=>{for(const k of ['supervisor','clear','position'])$(k).checked=false;drawRoute();render();};
for(const k of ['supervisor','clear','position'])$(k).onchange=render;
$('stop').onclick=async()=>{if(!data?.active_id||stopping)return;stopping=true;owned=null;render();try{const r=await api('/phone/native/submit',{action:'cancel',id:data.active_id});text('message',r.message||'已提交停止，请观察现场停稳。');}catch(e){text('message','停止结果未确认，请立即用遥控器接管。'+e.message);}finally{stopping=false;render();}};
setInterval(async()=>{if(!owned?.id||heartPending||document.hidden)return;heartPending=true;try{await api('/phone/native/submit',{action:'heartbeat',id:owned.id,owner:owned.owner});}catch{owned=null;text('message','测试心跳中断，控制程序将请求停止；请用遥控器核对。');}finally{heartPending=false;}},800);
document.addEventListener('visibilitychange',()=>{if(document.hidden)owned=null;});
setInterval(render,1000);drawRoute();refresh();
