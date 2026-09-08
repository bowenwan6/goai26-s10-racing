const $=id=>document.getElementById(id);
let csrf='',state=null,terrain='basic',busy=false;
const terrainNames={basic:'基础运动',stairs:'连续台阶',ledge:'高台跨越',gravel:'碎石路面',mixed:'混合地形',unlabeled:'地形待标注'};
const statuses={recording:'录制中',stopped:'已保存',interrupted:'已中断',failed:'录制失败'};
function message(text){$('message').textContent=text;$('message').style.display=text?'block':'none';}
async function api(path,data){const r=await fetch(path,{method:data?'POST':'GET',headers:data?{'Content-Type':'application/json','X-CSRF-Token':csrf}:{},body:data?JSON.stringify(data):undefined});const v=await r.json();if(!r.ok){if(r.status===401){$('login').hidden=false;$('workspace').hidden=true;}throw Error(v.error||'请求失败');}return v;}
function key(){return Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');}
async function action(data){if(busy)return;busy=true;renderControls();try{await api('/api/action',{...data,request_id:key()});message('操作已完成');await refresh();}catch(e){message(e.message+'；结果不明时先刷新状态，不要重复点击。');await refresh();}finally{busy=false;renderControls();}}
function renderControls(){const active=!!state?.active;$('start').disabled=active||busy;$('stop').disabled=!active||busy;document.querySelectorAll('[data-event]').forEach(b=>b.disabled=!active||busy);}
function element(tag,text,cls){const e=document.createElement(tag);e.textContent=text;if(cls)e.className=cls;return e;}
function render(){const a=state.active;$('mode').textContent=state.mode==='demo'?'演示模式 · 合成数据':'ROS 2 实际采集';$('notice').textContent=state.mode==='demo'?'当前为本地演示：可以体验完整采集流程，所有生成数据均标为合成数据，不能用于训练。':(state.error||'后台只订阅数据。先核实原厂动作输出、关节反馈和 IMU；记录保存后仍需同步与动作语义审查。');$('disk').textContent='剩余 '+state.free_gb+' GB';$('recordState').textContent=a?'正在录制 · '+terrainNames[a.terrain]:'准备采集';$('sessionName').textContent=a?a.id:'选择地形，确认数据连接后开始。';$('elapsed').textContent=a?formatTime((Date.now()-a.started_wall_ns/1e6)/1000):'00:00';
 const motion=state.motion_feedback?.current;
 $('gaitFeedback').textContent=motion?(motion.synthetic?'演示 · ':'')+motion.gait_label+(motion.gait_hex?' · '+motion.gait_hex:''):'未知 · 运动反馈缺失或已过期';
 $('gaitDetail').textContent=motion?'运动状态：'+motion.state_label+'。'+(motion.hint||'自动保存步态切换，地形标签保持独立。'):'等待 /MOTION_INFO 新鲜消息，过期反馈不作为当前步态。';
 $('health').replaceChildren();Object.entries(state.health).forEach(([topic,v])=>{const d=element('div','','signal');d.append(element('strong',topic),element('div',v.fresh?'已收到 · '+v.hz.toFixed(1)+' Hz':'未收到 / 已过期',v.fresh?'on':'off'),element('p',v.label,'hint'));$('health').append(d);});
 $('history').replaceChildren();state.history.filter(v=>v.status!=='recording').forEach(v=>{const row=element('div','','history-row'),detail=element('div');detail.append(element('strong',terrainNames[v.terrain]+' · '+(statuses[v.status]||v.status)),element('p',v.id),element('p',(v.mode==='demo'?'合成数据 · 不可训练':'待训练前审查')+' · '+(v.outcome||'unlabeled')));const link=element('a','下载 ZIP');link.href='/api/download/'+encodeURIComponent(v.id);row.append(detail,link);$('history').append(row);});if(!$('history').children.length)$('history').append(element('p','暂无已保存记录。'));renderControls();}
function formatTime(s){s=Math.max(0,Math.floor(s));return String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0');}
async function refresh(){try{state=await api('/api/state');csrf=state.csrf;$('login').hidden=true;$('workspace').hidden=false;render();}catch(e){if(!$('workspace').hidden){$('gaitFeedback').textContent='未知 · 页面连接中断';message('连接中断，后台录制状态未知。恢复连接后查看当前会话。');}}}
$('loginForm').onsubmit=async e=>{e.preventDefault();try{const r=await api('/api/login',{token:$('token').value});csrf=r.csrf;$('token').value='';message('');await refresh();}catch(e){message(e.message);}};
document.querySelectorAll('[data-terrain]').forEach(b=>b.onclick=()=>{terrain=b.dataset.terrain;document.querySelectorAll('[data-terrain]').forEach(x=>{x.classList.toggle('selected',x===b);x.setAttribute('aria-pressed',String(x===b));});});
$('captureForm').onsubmit=e=>{e.preventDefault();const parameters={};['height_cm','depth_cm','slope_deg','gravel_cm'].forEach(k=>{if($(k).value!=='')parameters[k]=Number($(k).value);});action({action:'start',terrain,parameters,teacher:$('teacher').value,notes:$('notes').value,duration_s:Number($('duration').value),allow_partial:$('partial').checked});};
document.querySelectorAll('[data-event]').forEach(b=>b.onclick=()=>action({action:'mark',session_id:state.active?.id,label:b.dataset.event}));
$('stop').onclick=()=>action({action:'stop',session_id:state.active?.id,outcome:$('outcome').value});
refresh();setInterval(refresh,2000);
