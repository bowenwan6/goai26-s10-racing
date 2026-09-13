'use strict';
(() => {
  const $ = id => document.getElementById(id);
  let csrf='', session=null, jobs=[], connected=false, busy=false, posting=false, selected=null;
  let live=null, liveAt=0, preview=null, previewLoading=false, checkId=null;
  let view='xy', scale=10, center=[0,0], drag=null;
  const actionIds=['selfcheck','recheck','load','localize','confirm','record','waypoint','finish'];
  function remember(value) { try {localStorage.setItem('s10-field-session',value);} catch (_) {} }
  function remembered() { try {return localStorage.getItem('s10-field-session');} catch (_) {return null;} }
  function nonce() { // getRandomValues works on a private-IP HTTP origin; randomUUID need not.
    const bytes=new Uint8Array(24); crypto.getRandomValues(bytes);
    return Array.from(bytes,v=>v.toString(16).padStart(2,'0')).join('');
  }
  async function api(path, options) {
    const controller=new AbortController(), timeout=setTimeout(()=>controller.abort(),15000);
    try {
      const response=await fetch(path,{...options,cache:'no-store',signal:controller.signal});
      const data=await response.json();
      if(!response.ok) throw Error(data.detail||`HTTP ${response.status}`);
      return data;
    } finally {clearTimeout(timeout);}
  }
  function notice(text) {$('notice').textContent=text;}
  function buttons() {actionIds.forEach(id=>$(id).disabled=!connected||busy||posting||(id!=='selfcheck'&&!session));}
  function safety() {
    if(!$('stationary').checked) throw Error('请先用手柄停稳并勾选确认。');
    return {stationary:true};
  }
  async function submit(action,params={}) {
    if(!connected||busy||posting) return;
    let body;
    try {body={action,key:nonce(),params}; if(action!=='selfcheck'||params.target_map===undefined)body.session_id=session.id;}
    catch(e){notice('无法创建安全请求：'+e.message);return;}
    posting=true;buttons();
    try {
      const job=await api('/phone/field/submit',{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:JSON.stringify(body)});
      remember(job.session_id);session=await api('/phone/field/session?session_id='+job.session_id);
      selected=job.id;notice('任务已登记在机器人上：'+job.id+'。锁屏/断线不取消已接受任务。');
      if(action==='load_map'){$('human').checked=false;preview=null;checkId=null;}
      await refreshJobs();
    } catch(e) {
      notice('未确认此次请求结果：'+e.message+'。不会自动重试；先查看下方任务记录确认是否已接受。');
      await refreshJobs().catch(()=>{});
    } finally {posting=false;buttons();}
  }
  async function restore(id) {
    if(!id)return;
    session=await api('/phone/field/session?session_id='+id);remember(id);
    $('target').value=session.target;$('sessionId').textContent='会话 '+session.id;
    preview=null;checkId=null;$('human').checked=false;
    await refreshJobs();buttons();
  }
  async function refreshJobs() {
    const data=await api('/phone/field/list');
    const sessions=new Map();data.jobs.forEach(j=>{if(!sessions.has(j.session_id))sessions.set(j.session_id,j);});
    const options=[];sessions.forEach((j,id)=>{const option=document.createElement('option');option.value=id;option.textContent=new Date(j.created*1000).toLocaleString()+' · '+id.slice(0,8);options.push(option);});
    $('sessionList').replaceChildren(...options);if(session)$('sessionList').value=session.id;
    busy=data.jobs.some(j=>['QUEUED','RUNNING'].includes(j.state));
    if(!session&&data.jobs.length){const saved=remembered();await restore(sessions.has(saved)?saved:data.jobs[0].session_id);return;}
    jobs=data.jobs.filter(j=>session&&j.session_id===session.id);
    checkId=jobs.find(j=>j.action==='localization_check'&&j.state==='SUCCEEDED'&&j.result?.passed)?.id||null;
    const rows=jobs.map(j=>{
      const row=document.createElement('div');row.className='job';
      const label=document.createElement('div');label.textContent=`${new Date(j.created*1000).toLocaleTimeString()} · ${j.action} · ${j.state}`;
      const stage=document.createElement('div');stage.className='sub';stage.textContent=j.error||j.stage;
      const button=document.createElement('button');button.textContent='查看结果 / 下载';button.onclick=()=>showJob(j.id);
      row.append(label,stage,button);return row;
    });
    $('jobs').replaceChildren(...rows);if(!rows.length)$('jobs').textContent='当前会话暂无任务';
    if(selected)await showJob(selected);
    const check=jobs.find(j=>j.action==='selfcheck'&&j.result);
    if(check){$('checks').replaceChildren(...(check.result.checks||[]).map(c=>{const d=document.createElement('div');d.className='check '+(c.state==='pass'?'good':c.state==='fail'?'bad':'unknown');d.textContent=`${c.name} · ${c.state} — ${c.detail}`;return d;}));}
    buttons();
  }
  async function showJob(id) {
    selected=id;const j=await api('/phone/field/job?job_id='+id);
    $('detail').textContent=JSON.stringify(j,null,2);
    let box=$('downloads');if(!box){box=document.createElement('div');box.id='downloads';$('detail').before(box);}box.replaceChildren();
    (j.artifacts||[]).forEach(a=>{const p=document.createElement('p'),link=document.createElement('a');link.href='/phone/field/download?artifact_id='+a.id;link.textContent=`下载 ${a.name}（${(a.size/1048576).toFixed(2)} MiB）`;link.download=a.name;p.append(link);const hash=document.createElement('div');hash.className='sub';hash.textContent='SHA256 '+a.sha256;p.append(hash);box.append(p);});
  }
  async function loadPreview() {
    if(!session||previewLoading)return;
    previewLoading=true;
    try{notice('读取目标地图抽样，原始地图文件不会改变…');preview=await api('/phone/field/preview?session_id='+session.id);fit();notice('地图抽样已加载。只有同地图身份、新鲜同map扫描才会叠加。');}
    catch(e){notice('地图叠合不可用：'+e.message);preview=null;}
    finally{previewLoading=false;draw();}
  }
  function validLive(){
    const p=live?.pose,s=live?.status;
    return connected&&performance.now()-liveAt<2500&&live?.map_name===session?.target&&live?.map_identity===preview?.map_identity&&
      live?.mapping_active===false&&live?.localization_active===true&&s?.fresh===true&&s?.code===0&&s?.mode==='全局'&&
      p?.frame==='map'&&Array.isArray(p.xyz)&&p.xyz.length===3&&p.xyz.every(Number.isFinite)&&Number.isFinite(p.age)&&p.age<=.5&&
      Number.isFinite(p.stamp_age_s)&&p.stamp_age_s>=-.05&&p.stamp_age_s<=.5&&!p.error;
  }
  function validCloud(){
    const c=live?.aligned_cloud,p=live?.pose;
    const age=c?.stamp_age_s??(Number.isFinite(live?.board_time)&&Number.isFinite(c?.stamp)?live.board_time-c.stamp:p?.stamp_age_s);
    return validLive()&&c?.frame==='map'&&!c.error&&Number.isFinite(c.age)&&c.age>=0&&c.age<=1&&
      Number.isFinite(c.stamp)&&Number.isFinite(age)&&age>=-.05&&age<=.5&&Math.abs(c.stamp-p.stamp)<=.5&&
      Array.isArray(c.points)&&c.points.length>0&&c.points.every(v=>Array.isArray(v)&&v.length===3&&v.every(Number.isFinite));
  }
  function fit(){
    if(!preview?.points?.length)return;
    const second=view==='xy'?1:2;
    const xs=preview.points.map(p=>p[0]),ys=preview.points.map(p=>p[second]);
    const minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
    center=[(minX+maxX)/2,(minY+maxY)/2];scale=Math.min(900/Math.max(1,maxX-minX),420/Math.max(1,maxY-minY));draw();
  }
  function draw(){
    const canvas=$('map'),ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height,second=view==='xy'?1:2;
    ctx.fillStyle='#111f1d';ctx.fillRect(0,0,w,h);
    const good=validLive();
    if(good&&$('follow').checked)center=[live.pose.xyz[0],live.pose.xyz[second]];
    function point(p,size,color){const x=w/2+(p[0]-center[0])*scale,y=h/2-(p[second]-center[1])*scale;if(x>=0&&x<w&&y>=0&&y<h){ctx.fillStyle=color;ctx.fillRect(x-size/2,y-size/2,size,size);}}
    (preview?.points||[]).forEach(p=>point(p,2,'#7b8d85'));
    if(good){if(validCloud())live.aligned_cloud.points.forEach(p=>point(p,3,'#47e6cd'));point(live.pose.xyz,11,'#ffb85a');}
    ctx.fillStyle='#dbe6df';ctx.font='18px sans-serif';ctx.fillText(view==='xy'?'XY 俯视 · 请同时核对高度':'XZ 侧视 · 核对楼层/坡道',16,30);
    if(!good){ctx.fillStyle='#ffc58a';ctx.fillText('没有可确认的新鲜同图定位/扫描；当前显示仅是目标地图',16,h-20);}
    $('liveQuality').textContent=good?(validCloud()?'程序状态：新鲜正常全局定位与同图扫描；仍需人工核对，不证明整条路线精度。':'位姿新鲜但扫描叠合不可用/异常，不能提交人工叠合确认。'):'程序状态未确认：需目标图身份、全局状态和新鲜数据同时匹配；不显示过期机器人标记。';
    $('liveQuality').className=good?'good':'unknown';
  }
  async function poll(){
    try{
      const health=await api('/phone/field/health');csrf=health.csrf;connected=true;$('demo').hidden=!health.demo;
      $('connection').textContent=health.demo?'演示后台在线 · 不是真机':'机器人后台已连接 · 手机关闭不取消已接受任务';
      try{live=await api('/phone/field/live');liveAt=performance.now();}
      catch(e){live=null;notice('后台可用但实时证据不可用：'+e.message+'。仍可汇总导出已有记录，不代表定位合格。');}
      $('identity').textContent='当前地图：'+(live?.map_name||'未知')+'；定位会话：'+(live?.invocation||'未知');
      const p=live?.pose;$('pose').textContent=p?`XYZ: ${p.xyz?.map(v=>Number.isFinite(v)?v.toFixed(3):'?').join(', ')} m · frame=${p.frame||'空'} · child=${p.child_frame||'空（未验证参考点，只能草稿）'} · 零协方差不是精度证明`:'尚无位姿';
      await refreshJobs();
    }catch(e){connected=false;live=null;$('connection').textContent='无法确认后台连接：'+e.message+'；已接受任务仍由机器人限时执行，重连只查询。';}
    buttons();draw();setTimeout(poll,1500);
  }
  $('selfcheck').onclick=()=>submit('selfcheck',{target_map:$('target').value.trim()});
  $('recheck').onclick=()=>submit('selfcheck');
  $('restore').onclick=()=>restore($('sessionList').value).catch(e=>notice(e.message));
  function wrap(fn){return()=>{try{fn();}catch(e){notice(e.message);}};}
  $('load').onclick=wrap(()=>submit('load_map',{...safety(),remote_ready:$('remote').checked}));
  $('localize').onclick=wrap(()=>submit('localization_check',safety()));
  $('confirm').onclick=wrap(()=>{if(!checkId)throw Error('尚无本会话通过的静止检查。');if(!validCloud())throw Error('先加载预览并等同图新鲜扫描与定位；不能盲目确认。');submit('confirm_overlay',{check_id:checkId,confirmed:$('human').checked});});
  $('record').onclick=()=>submit('record',{kind:$('kind').value,seconds:Number($('seconds').value),remote_ready:$('remote').checked});
  $('waypoint').onclick=wrap(()=>submit('waypoint',{...safety(),name:$('pointName').value.trim(),floor:$('floor').value.trim()||'未填写',segment:'flat',draft:$('draft').checked}));
  $('finish').onclick=()=>submit('finish');$('preview').onclick=loadPreview;
  $('top').onclick=()=>{view='xy';fit();};$('side').onclick=()=>{view='xz';fit();};
  $('zoomIn').onclick=()=>{scale=Math.min(1000,scale*1.5);draw();};$('zoomOut').onclick=()=>{scale=Math.max(.1,scale/1.5);draw();};$('fit').onclick=fit;
  const canvas=$('map');canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY,...center];canvas.setPointerCapture(e.pointerId);$('follow').checked=false;};
  canvas.onpointermove=e=>{if(!drag)return;center=[drag[2]-(e.clientX-drag[0])*canvas.width/canvas.clientWidth/scale,drag[3]+(e.clientY-drag[1])*canvas.height/canvas.clientHeight/scale];draw();};
  canvas.onpointerup=()=>drag=null;canvas.onpointercancel=()=>drag=null;
  canvas.addEventListener('wheel',e=>{e.preventDefault();scale=Math.max(.1,Math.min(1000,scale*(e.deltaY<0?1.2:1/1.2)));draw();},{passive:false});
  setInterval(()=>{if(performance.now()-liveAt>2500)draw();},500);
  buttons();poll();
})();
