'use strict';
const $=id=>document.getElementById(id),params=new URLSearchParams(location.search);
const escape=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const sid=params.get('id')||CATALOG[0].recording_id;
if(!CATALOG.some(r=>r.recording_id===sid))throw Error('未知录制 ID');
let R,t=Number(params.get('t'))||0,playing=false,last=0,limit=Number(params.get('end'))||Infinity,loopStart=0,loopEnabled=false,detailData=null;
const script=document.createElement('script');script.src='data/'+sid+'.js';script.onload=()=>{R=window.RECORDING;init();};script.onerror=()=>{$('title').textContent='本地数据文件缺失：'+script.src};document.body.append(script);
function jump(v){t=Math.max(0,Math.min(R.duration,Number(v)||0));render();}
function closest(a,k,v){if(!a.length)return null;let lo=0,hi=a.length;while(lo<hi){const m=(lo+hi)>>1;if(a[m][k]<v)lo=m+1;else hi=m;}let i=Math.min(lo,a.length-1);if(i&&Math.abs(a[i-1][k]-v)<Math.abs(a[i][k]-v))i--;return a[i];}
function inDetail(){return detailData&&$('clock').value==='src'&&t>=detailData.start_s&&t<=detailData.end_s;}
function cloudRows(topic){return inDetail()?detailData.clouds[topic]:R.clouds[topic];}
const gait=g=>({4097:'基础 0x1001',4098:'0x1002（指南高台码，映射待确认）',4099:'楼梯 0x1003',0:'无步态 0x0000'}[g]||'0x'+g.toString(16));
const state=s=>({0:'空闲',1:'站立',2:'关节阻尼/软急停',3:'开机阻尼',4:'趴下',17:'RL 控制'}[s]||'未知');
function canvas(id){let c=$(id),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);g.font='13px system-ui';return [c,g];}
function cloud(which){let [c,g]=canvas(which),mode=$('clock').value,topic='/rslidar_'+which+'/points',cc=closest(cloudRows(topic),mode,t);if(!cc){$(which+'time').textContent='无点云帧';return;}let dt=cc[mode]-t;
  $(which+'time').textContent=`src ${cc.src.toFixed(3)}s / rx ${cc.rx.toFixed(3)}s / 样本−游标 ${dt.toFixed(3)}s；frame=${cc.frame}；显示 ${cc.n}/${cc.total} 点`;
  if(Math.abs(dt)>(inDetail()?.15:.65)){g.fillStyle='#b42318';g.fillText('无邻近点云帧（不保留旧帧）',20,40);return;}
  if(!cc.points){let bytes=Uint8Array.from(atob(cc.xyz_mm_b64),x=>x.charCodeAt(0));let view=new DataView(bytes.buffer);cc.points=new Float32Array(bytes.length/2);for(let i=0;i<cc.points.length;i++)cc.points[i]=view.getInt16(i*2,true)/1000;}
  let view=$('view').value,range=6,scale=(c.width-70)/(range*2),cx=c.width/2,cy=c.height/2;
  g.strokeStyle='#e4e9ee';g.fillStyle='#6a7788';for(let i=-6;i<=6;i++){g.beginPath();g.moveTo(cx+i*scale,20);g.lineTo(cx+i*scale,c.height-25);g.stroke();g.fillText(String(i),cx+i*scale+2,c.height-8);}for(let i=-3;i<=3;i++){g.beginPath();g.moveTo(25,cy-i*scale);g.lineTo(c.width-15,cy-i*scale);g.stroke();}
  g.fillText(view==='side'?'x → / z ↑（m）':view==='top'?'x → / y ↑（m）':'本地点云立体投影',10,16);
  g.fillStyle=which==='front'?'#2065af':'#c25b24';let a=cc.points,w=Number($('width').value);
  for(let i=0;i<a.length;i+=3){let x=a[i],y=a[i+1],z=a[i+2];if(view==='side'&&Math.abs(y)>w)continue;let px=x,py=view==='top'?y:z;if(view==='iso'){px=.85*x-.5*y;py=.8*z+.25*x+.35*y;}const sx=cx+px*scale,sy=cy-py*scale;if(sx>0&&sx<c.width&&sy>0&&sy<c.height)g.fillRect(sx,sy,1.6,1.6);}
}
function robot(f){const[c,g]=canvas('robot');g.fillStyle='#875a11';g.fillText('固定原点 / 示意骨架 / 无世界位移',15,20);
 if(!f.pose){g.fillStyle='#b42318';g.fillText('IMU 或关节无邻近样本；不显示旧姿态',20,70);return;}
 const project=p=>[c.width/2+260*(.85*p[0]-.5*p[1]),125-260*(p[2]+.25*p[0]+.4*p[1])];
 for(let i=1;i<f.pose.length;i++){let p=R.body_parents[i];if(p<0)continue;let a=project(f.pose[p]),b=project(f.pose[i]);g.strokeStyle=R.body_names[i].startsWith('f')?'#2374ba':'#bc6428';g.lineWidth=5;g.beginPath();g.moveTo(...a);g.lineTo(...b);g.stroke();g.fillStyle='#203a50';g.beginPath();g.arc(...b,R.body_names[i].includes('wheel')?12:4,0,Math.PI*2);g.stroke();}
 const hips=['fl_hipx','fr_hipx','hr_hipx','hl_hipx'].map(n=>project(f.pose[R.body_names.indexOf(n)]));g.fillStyle='#7290ad88';g.beginPath();hips.forEach((p,i)=>i?g.lineTo(...p):g.moveTo(...p));g.closePath();g.fill();g.stroke();
 g.fillStyle='#244662';g.fillText('蓝：前腿；橙：后腿；轮角见右侧原值',15,c.height-15);
}
function joints(f){const[c,g]=canvas('joints');g.fillStyle='#24394e';g.fillText('数值保留原始累计轮角；蓝条以 ±π 截断显示',15,20);if(!f.joints){g.fillText('无邻近关节样本',15,55);return;}for(let i=0;i<16;i++){let x=i<8?20:350,y=52+(i%8)*35;g.fillStyle='#25394c';g.fillText(`${i.toString().padStart(2,'0')}  ${f.joints[i].toFixed(3)}`,x,y);g.fillStyle=i%4===3?'#ba6228':'#2774b3';let q=Math.max(-Math.PI,Math.min(Math.PI,f.joints[i]));g.fillRect(x+190,y-10,q*20,8);}}
function timeline(){const[c,g]=canvas('timeline'),x=v=>105+(c.width-115)*v/R.duration;
 g.fillStyle='#263b51';g.fillText('阶段候选',5,20);g.fillText('实际步态',5,48);g.fillText('运动状态',5,73);g.fillText('GAIT 接收',5,98);
 for(const s of R.segments){g.fillStyle=s.action_phase.includes('退出')?'#c67b72':s.action_phase.includes('等待')?'#d9dfe4':s.action_phase.includes('楼')||s.action_phase.includes('越障')||s.action_phase.includes('台')?'#e3b75f':'#94b9b0';g.fillRect(x(s.start_s),5,Math.max(1,x(s.end_s)-x(s.start_s)),18);}
 for(const s of R.control){g.fillStyle=({4097:'#8bb7dc',4098:'#d6a570',4099:'#a396d0'}[s.values[1]]||'#bdc3c8');g.fillRect(x(s.start_s),31,x(s.end_s)-x(s.start_s),18);g.fillStyle=s.values[0]===17?'#91bb9b':'#cc8c80';g.fillRect(x(s.start_s),57,x(s.end_s)-x(s.start_s),18);}
 for(const e of R.events.filter(e=>e.kind==='gait_command')){g.fillStyle='#bd3c39';g.fillRect(x(e.t),83,2,18);}g.strokeStyle='#162635';g.beginPath();g.moveTo(x(t),0);g.lineTo(x(t),105);g.stroke();
 if($('clock').value==='rx'){g.fillStyle='#9d381d';g.fillText('阶段/实际控制条仍引用源秒；游标现为接收秒',480,98);}
}
function render(){if(!R)return;t=Math.min(t,R.duration);$('time').value=t.toFixed(2);$('slider').value=t;let mode=$('clock').value,f=closest(inDetail()?detailData.frames:R.frames[mode],'t',t);$('clocktext').textContent=`/ ${R.duration.toFixed(3)}s`;
 const s=R.segments.find(s=>s.start_s<=t&&s.end_s>t)||R.segments.at(-1),p=R.passages.find(p=>p.passage_id===s.passage_id);$('phase').textContent=`${s.passage_id.split('-P')[1]} ${p.kind==='candidate_continuous_passage'?'通行候选':'上下文组'} / ${s.segment_id.split('-S')[1]} 片段：${s.action_phase} · 地形 ${s.terrain} · 结果 ${s.outcome} · 人工待复核`;
 $('feedback').textContent=f.motion?`实际 ${gait(f.motion[5])}；运动状态 ${f.motion[4]} ${state(f.motion[4])}；报告 vx/vy/wz=${f.motion.slice(0,3).join(' / ')}`:'实际控制反馈缺失（不延续最后状态）';
 $('imustate').textContent=f.imu_rpy?'IMU roll/pitch/yaw = '+f.imu_rpy.join(' / ')+' deg（数值为原 IMU）':'IMU 无邻近样本';
 $('samples').textContent=Object.entries(f.samples).map(([k,s])=>`${k}: src ${s.src.toFixed(3)}s, rx ${s.rx.toFixed(3)}s${s.valid?'':' [缺失/过期]'}`).join(' ｜ ');
 $('jointstate').textContent='关节顺序来自数字名 0–15，绘图对应 SDK actuator 顺序假设；无物理仿真。';cloud('front');cloud('rear');robot(f);joints(f);timeline();drawScene(f);
}
function links(rows,label){return rows.map(r=>`<p><button data-start="${r[0]}" data-end="${r[1]}">${Number(r[0]).toFixed(2)}–${Number(r[1]).toFixed(2)}s</button> ${escape(r[2])}</p>`).join('');}
function init(){document.title=sid+' · S10复核';$('title').textContent=sid;$('meta').textContent=`整段标签：${R.manifest.terrain} / ${R.manifest.outcome}；记录参数 ${JSON.stringify(R.manifest.parameters)}（未测量验证）`;$('anchor').textContent=R.anchor_ns;$('slider').max=R.duration;$('time').max=R.duration;
 $('signals').href=`previews/${sid}_signals.png`;$('cloudsheet').href=`previews/${sid}_clouds.jpg`;
 $('questions').innerHTML=links(R.observations.review_intervals||[]);$('observations').textContent=(R.observations.observations||[]).map(x=>`${x.start_s}–${x.end_s}s：${x.evidence}`).join('；');
 $('issues').innerHTML=R.issues.map(s=>'<li>'+escape(s)+'</li>').join('');$('events').innerHTML=links(R.events.map(e=>[e.t,Math.min(e.t+3,R.duration),e.kind+'：'+e.label]));
 $('segments').innerHTML=links(R.segments.map(s=>[s.start_s,s.end_s,s.segment_id.split('-S')[1]+' '+s.action_phase+' / '+s.outcome+'；'+s.visual_evidence.join('；')]));
 document.querySelectorAll('[data-start]').forEach(b=>b.onclick=()=>{playing=false;loopEnabled=false;$('play').textContent='播放';$('clock').value='src';limit=Number(b.dataset.end);loopStart=Number(b.dataset.start);jump(loopStart);reviewRange(loopStart,limit);});
 $('play').onclick=()=>{if(t>=Math.min(limit,R.duration)-.01)jump(loopStart);playing=!playing;$('play').textContent=playing?'暂停':'播放';last=performance.now();};$('back').onclick=()=>jump(t-.5);$('next').onclick=()=>jump(t+.5);$('time').onchange=e=>jump(e.target.value);$('slider').oninput=e=>jump(e.target.value);
 ['clock','view','width'].forEach(id=>$(id).onchange=render);$('timeline').onclick=e=>jump((e.offsetX/$('timeline').clientWidth*1400-105)/(1400-115)*R.duration);
 reviewUiInit();render();requestAnimationFrame(tick);
}
function tick(now){if(playing&&now-last>=16){t+=(now-last)/1000*Number($('speed').value);last=now;if(t>=Math.min(limit,R.duration)){if(loopEnabled)t=loopStart;else{t=Math.min(limit,R.duration);playing=false;$('play').textContent='播放';}}render();}requestAnimationFrame(tick);}
