'use strict';
let reviewRows=[],reviewItems=[],reviewCurrent=null,reviewDirty=false,reviewSelection=0,reviewSource='questions';
let orbit={az:.65,el:.4,range:2},dragPoint=null;
const reviewFields={level:'level',terrain:'terrain',surface_type:'surface',reference_use:'reference',action_phase:'action',direction:'direction',outcome:'outcome',obstacle_id:'obstacle',group_id:'group',notes:'notes'};

async function reviewApi(path,entries){
 const response=await fetch(path,entries?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(entries)}:{});
 if(!response.ok){let info;try{info=await response.json();}catch{throw Error('请使用 http://127.0.0.1:8767 打开，静态预览服务不能保存。');}throw Error(info.error||'请求失败');}
 return response.json();
}
function reviewMessage(text,error=false){$('review-status').textContent=text;$('review-status').className=error?'error':'';}
async function refreshReviews(){const data=await reviewApi('/api/reviews');reviewRows=data.reviews;
 const local=reviewRows.filter(r=>r.recording_id===sid);
 $('review-progress').textContent=`本录制人工标注 ${local.length} 条：已确认 ${local.filter(r=>r.human_review_status==='confirmed').length}，无法判断 ${local.filter(r=>r.human_review_status==='uncertain').length}，草稿 ${local.filter(r=>r.human_review_status==='pending').length}。自动索引保持原样。`;
}
function reviewRange(a,b){
 if(reviewDirty&&!confirm('当前修改尚未保存，放弃修改并切换？'))return;
 fillReview({start_s:a,end_s:b,label:'从时间线选取的区间'});
}
function fillReview(item){
 const level=item.level||($('review-source').value==='passages'?'passage':'segment');
 const saved=item.id?item:reviewRows.find(r=>r.recording_id===sid&&r.level===level&&Math.abs(r.start_s-item.start_s)<1e-6&&Math.abs(r.end_s-item.end_s)<1e-6);
 reviewCurrent=saved||{id:crypto.randomUUID(),recording_id:sid,start_s:item.start_s,end_s:item.end_s,level,
  terrain:'unknown',action_phase:'unknown',direction:'unknown',outcome:'uncertain',obstacle_id:'',group_id:item.passage_id||'',notes:'',human_review_status:'pending'};
 $('review-start').value=reviewCurrent.start_s.toFixed(3);$('review-end').value=reviewCurrent.end_s.toFixed(3);
 for(const [k,id] of Object.entries(reviewFields))$('review-'+id).value=reviewCurrent[k]||(k==='surface_type'?'unknown':k==='reference_use'?'pending':'');
 $('review-question').textContent=item.label||`${item.human_review_status} · ${item.start_s.toFixed(2)}–${item.end_s.toFixed(2)}s`;
 $('review-retire').disabled=!saved;reviewDirty=false;
}
function populateReviews(){
 const source=$('review-source').value;
 if(source==='questions')reviewItems=(R.observations.review_intervals||[]).map(([a,b,label])=>({start_s:a,end_s:Math.min(b,R.duration),label}));
 if(source==='passages')reviewItems=R.passages.map(p=>({...p,label:p.passage_id.split('-P')[1]+' · '+p.kind}));
 if(source==='segments')reviewItems=R.segments.map(s=>({...s,label:s.segment_id.split('-S')[1]+' · '+s.action_phase}));
 if(source==='saved')reviewItems=reviewRows.filter(r=>r.recording_id===sid).sort((a,b)=>a.start_s-b.start_s).map(r=>({...r,label:r.human_review_status+' · '+r.action_phase}));
 $('review-item').innerHTML=reviewItems.map((r,i)=>`<option value="${i}">${r.start_s.toFixed(2)}–${r.end_s.toFixed(2)}s · ${escape(r.label)}</option>`).join('');
 if(reviewItems.length)selectReview(0,true);else{$('review-question').textContent='此列表暂无项目，可从时间线选区。';}
}
function selectReview(index,force=false){
 if(!reviewItems.length)return;
 if(!force&&reviewDirty&&!confirm('当前修改尚未保存，放弃修改并切换？')){$('review-item').value=reviewSelection;return;}
 reviewSelection=Math.max(0,Math.min(reviewItems.length-1,index));$('review-item').value=reviewSelection;
 const item=reviewItems[reviewSelection];fillReview(item);playing=false;loopEnabled=false;$('play').textContent='播放';$('clock').value='src';loopStart=Math.max(0,item.start_s-2);limit=Math.min(R.duration,item.end_s+2);jump(item.start_s);
}
function readReview(status){
 const row={...reviewCurrent,start_s:Number($('review-start').value),end_s:Number($('review-end').value),human_review_status:status};
 if(!Number.isFinite(row.start_s)||!Number.isFinite(row.end_s)||row.start_s<0||row.start_s>=row.end_s||row.end_s>R.duration+1e-8)throw Error('请设置录制范围内的有效起止时间。');
 if($('clock').value!=='src')throw Error('请切换到源时间后标注，避免混用接收时间。');
 for(const [k,id] of Object.entries(reviewFields))row[k]=$('review-'+id).value;
 return row;
}
async function persistReviews(entries){
 const data=await reviewApi('/api/reviews',entries);reviewDirty=false;await refreshReviews();
 reviewMessage(`已写入本地 human_reviews.jsonl，共 ${data.saved.length} 条修订；原始录制和自动候选未修改。`);return data.saved;
}
async function saveReview(status){try{const [saved]=await persistReviews([readReview(status)]);fillReview(saved);}catch(e){reviewMessage(e.message,true);}}
async function splitReview(){try{
 const row=readReview('pending');if(!(row.start_s<t&&t<row.end_s))throw Error('请先将播放游标放在区间内部。');
 const parts=[{...row,id:crypto.randomUUID(),end_s:t},{...row,id:crypto.randomUUID(),start_s:t}].map(r=>({...r,outcome:'uncertain',action_phase:'unknown',direction:'unknown',reference_use:'pending'}));
 const entries=reviewRows.some(r=>r.id===row.id)?[{...row,retired:true},...parts]:parts;
 await persistReviews(entries);$('review-source').value='saved';populateReviews();selectReview(reviewItems.findIndex(r=>r.id===parts[0].id),true);
 reviewMessage('已保存两段待复核草稿。动作、方向和结果已重置，请分别检查后确认。');
 }catch(e){reviewMessage(e.message,true);}}
function reviewUiInit(){
 $('review-source').onchange=()=>{if(reviewDirty&&!confirm('当前修改尚未保存，放弃修改？')){$('review-source').value=reviewSource;return;}reviewSource=$('review-source').value;populateReviews();};
 $('review-item').onchange=e=>selectReview(Number(e.target.value));$('review-prev').onclick=()=>selectReview(reviewSelection-1);$('review-next').onclick=()=>selectReview(reviewSelection+1);
 document.querySelectorAll('.review-card input,.review-card textarea,.review-grid select,#review-level').forEach(el=>el.addEventListener('input',()=>reviewDirty=true));
 $('review-set-start').onclick=()=>{if($('clock').value!=='src')return reviewMessage('先切换到源时间。',true);$('review-start').value=t.toFixed(3);reviewDirty=true;};
 $('review-set-end').onclick=()=>{if($('clock').value!=='src')return reviewMessage('先切换到源时间。',true);$('review-end').value=t.toFixed(3);reviewDirty=true;};
 $('review-loop').onclick=()=>{try{const r=readReview('pending');loopStart=Math.max(0,r.start_s-2);limit=Math.min(R.duration,r.end_s+2);loopEnabled=true;jump(loopStart);playing=true;last=performance.now();$('play').textContent='暂停';}catch(e){reviewMessage(e.message,true);}};
 $('review-save').onclick=()=>saveReview('confirmed');$('review-uncertain').onclick=()=>saveReview('uncertain');$('review-split').onclick=splitReview;
 $('review-merge').onclick=()=>{try{const r=readReview('pending');const rows=r.level==='passage'?R.passages:R.segments;const previous=rows.filter(s=>s.end_s<=r.start_s+1e-6).at(-1);if(!previous)throw Error('没有相邻的前一候选，请直接调整起止时间。');$('review-start').value=previous.start_s.toFixed(3);reviewDirty=true;reviewMessage('已扩展到前一候选的起点；检查范围后点击保存。不会自动继承完成结果。');$('review-outcome').value='uncertain';$('review-reference').value='pending';}catch(e){reviewMessage(e.message,true);}};
 $('review-retire').onclick=async()=>{try{await persistReviews([{...readReview('pending'),retired:true}]);$('review-source').value='saved';populateReviews();}catch(e){reviewMessage(e.message,true);}};
 $('review-export').onclick=async()=>{try{await refreshReviews();const blob=new Blob([JSON.stringify({schema_version:1,reviews:reviewRows},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='s10-human-reviews.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}catch(e){reviewMessage(e.message,true);}};
 $('detail-load').onclick=async()=>{try{
  const r=readReview('pending');let a=Math.max(0,r.start_s-2),b=Math.min(R.duration,r.end_s+2);
  if(b-a>20){a=Math.max(0,Math.min(t-5,R.duration-20));b=Math.min(R.duration,a+20);}
  $('detail-load').disabled=true;$('detail-status').textContent=`正在读取 ${a.toFixed(2)}–${b.toFixed(2)}s 的原始采样…`;
  detailData=await reviewApi(`/api/detail?id=${encodeURIComponent(sid)}&start=${a}&end=${b}`);
  $('detail-status').textContent=`精细回放 ${a.toFixed(2)}–${b.toFixed(2)}s：${detailData.native_joint_samples} 个原关节样本，区间中位频率 ${detailData.joint_median_hz?.toFixed(1)??'未知'} Hz；前/后点云 ${Object.values(detailData.clouds).map(x=>x.length).join('/')} 帧。区间外使用总览采样。`;
  $('clock').value='src';jump(Math.max(a,Math.min(b,t)));
 }catch(e){$('detail-status').textContent='加载失败：'+e.message;}finally{$('detail-load').disabled=false;}};
 $('detail-clear').onclick=()=>{detailData=null;$('detail-status').textContent='已返回总览：动作约10Hz，点云约2Hz。';render();};
 sceneInit();populateReviews();
 const desired=Number(params.get('t'));if(Number.isFinite(desired)&&params.has('t')){const end=Math.min(Number(params.get('end'))||R.duration,R.duration);const i=reviewItems.findIndex(r=>r.start_s<=desired&&r.end_s>=desired);if(params.get('range')==='basic'&&desired>=0&&desired<end){fillReview({start_s:desired,end_s:end,label:'基础步态参考候选：请确认地形形状、表面材质与参考用途'});loopStart=desired;limit=end;jump(desired);}else if(i>=0)selectReview(i,true);else{fillReview({start_s:desired,end_s:end,label:'指定区间'});jump(desired);}}
 refreshReviews().then(()=>{if(!reviewDirty&&reviewCurrent){const saved=reviewRows.find(r=>r.recording_id===sid&&r.level===reviewCurrent.level&&Math.abs(r.start_s-reviewCurrent.start_s)<1e-6&&Math.abs(r.end_s-reviewCurrent.end_s)<1e-6);if(saved)fillReview(saved);}reviewMessage('保存服务已连接：标注写入 human_reviews.jsonl，可刷新后继续。');}).catch(e=>reviewMessage(e.message,true));
 window.addEventListener('beforeunload',e=>{if(reviewDirty){e.preventDefault();e.returnValue='';}});
}

function sceneInit(){
 const c=$('scene3d');c.onpointerdown=e=>{dragPoint=[e.clientX,e.clientY];c.setPointerCapture(e.pointerId);};
 c.onpointermove=e=>{if(!dragPoint)return;orbit.az+=(e.clientX-dragPoint[0])*.008;orbit.el=Math.max(-1.5,Math.min(1.5,orbit.el+(e.clientY-dragPoint[1])*.008));dragPoint=[e.clientX,e.clientY];render();};c.onpointerup=()=>dragPoint=null;c.onpointercancel=()=>dragPoint=null;
 c.addEventListener('wheel',e=>{e.preventDefault();orbit.range=Math.max(.7,Math.min(12,orbit.range*Math.exp(e.deltaY*.001)));render();},{passive:false});
 c.onkeydown=e=>{const actions={ArrowLeft:()=>orbit.az-=.1,ArrowRight:()=>orbit.az+=.1,ArrowUp:()=>orbit.el=Math.min(1.5,orbit.el+.1),ArrowDown:()=>orbit.el=Math.max(-1.5,orbit.el-.1),'+':()=>orbit.range=Math.max(.7,orbit.range/1.1),'-':()=>orbit.range=Math.min(12,orbit.range*1.1)};if(actions[e.key]){e.preventDefault();actions[e.key]();render();}};
 for(const id of ['showfront','showrear','showbody'])$(id).onchange=render;
 $('camera-side').onclick=()=>{orbit.az=0;orbit.el=0;render();};$('camera-top').onclick=()=>{orbit.az=0;orbit.el=Math.PI/2;render();};$('camera-reset').onclick=()=>{orbit={az:.65,el:.4,range:2};render();};
}
function cloudRotation(frame){
 if(!frame?.imu_rpy)return null;
 const [r,p,y]=frame.imu_rpy.map(v=>v*Math.PI/180),initial=(R.frames.src[0].imu_rpy?.[2]||0)*Math.PI/180;
 const cy=Math.cos(y-initial),sy=Math.sin(y-initial),cp=Math.cos(p),sp=Math.sin(p),cr=Math.cos(r),sr=Math.sin(r);
 return [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr];
}
function drawScene(f){
 const [c,g]=canvas('scene3d');g.fillStyle='#101c2c';g.fillRect(0,0,c.width,c.height);
 const scale=c.width/(2*orbit.range),ca=Math.cos(orbit.az),sa=Math.sin(orbit.az),ce=Math.cos(orbit.el),se=Math.sin(orbit.el);
 const project=([x,y,z])=>[c.width/2+scale*(ca*x-sa*y),c.height*.52-scale*(ce*z+se*(sa*x+ca*y))];
 const line=(a,b,color,width=1)=>{g.strokeStyle=color;g.lineWidth=width;g.beginPath();g.moveTo(...project(a));g.lineTo(...project(b));g.stroke();};
 for(const [axis,color,label] of [[[1,0,0],'#e98585','X / 前'],[[0,1,0],'#85d5a2','Y / 左'],[[0,0,1],'#8abff3','Z / 上']]){line([0,0,0],axis,color);g.fillStyle=color;g.fillText(label,...project(axis));}
 const infos=[];
 for(const which of ['front','rear']){
  if(!$('show'+which).checked)continue;
  const cc=closest(cloudRows('/rslidar_'+which+'/points'),$('clock').value,t);
  if(!cc||Math.abs(cc[$('clock').value]-t)>(inDetail()?.15:.65)){infos.push(which+' 无邻近帧');continue;}
  // Rotation-only display in a shared heading frame; no assumed translation or map accumulation.
  const cf=closest(inDetail()?detailData.frames:R.frames.src,'t',cc.src),rot=cloudRotation(cf);
  if(!rot||Math.abs(cf.t-cc.src)>.2){infos.push(which+' 缺少IMU姿态');continue;}
  if(!cc.points){const bytes=Uint8Array.from(atob(cc.xyz_mm_b64),x=>x.charCodeAt(0)),v=new DataView(bytes.buffer);cc.points=new Float32Array(bytes.length/2);for(let i=0;i<cc.points.length;i++)cc.points[i]=v.getInt16(i*2,true)/1000;}
  g.fillStyle=which==='front'?'#66b6ff':'#f3ae67';
  for(let i=0;i<cc.points.length;i+=3){const x=cc.points[i],y=cc.points[i+1],z=cc.points[i+2];const a=rot[0]*x+rot[1]*y+rot[2]*z,b=rot[3]*x+rot[4]*y+rot[5]*z,d=rot[6]*x+rot[7]*y+rot[8]*z;const [px,py]=project([a,b,d]);if(px>=0&&px<c.width&&py>=0&&py<c.height)g.fillRect(px,py,2,2);}
  infos.push((which==='front'?'前':'后')+'点云@'+cc.src.toFixed(3)+'s');
 }
 if($('showbody').checked&&f.pose){
  const hips=['fl_hipx','fr_hipx','hr_hipx','hl_hipx'].map(n=>f.pose[R.body_names.indexOf(n)]);
  g.fillStyle='#d9eaf099';g.beginPath();hips.forEach((p,i)=>i?g.lineTo(...project(p)):g.moveTo(...project(p)));g.closePath();g.fill();
  for(let i=1;i<f.pose.length;i++){const parent=R.body_parents[i];if(parent<0)continue;const name=R.body_names[i];line(f.pose[parent],f.pose[i],name.startsWith('f')?'#e5f4ff':'#ffce96',5);const pt=project(f.pose[i]);g.strokeStyle='#f5f9fc';g.lineWidth=3;g.beginPath();g.arc(...pt,name.includes('wheel')?.081*scale:3,0,2*Math.PI);g.stroke();}
 }else{g.fillStyle='#f9bc85';g.fillText('机器人未显示或无邻近姿态',20,45);}
 g.fillStyle='#d9e5f1';g.fillText(`${inDetail()?'原始采样精细回放':'总览抽样'} · ${t.toFixed(3)} s · 视宽 ${(orbit.range*2).toFixed(1)} m`,18,24);
 $('scene-status').textContent=infos.join(' ｜ ')+` ｜ 动作@${f.t.toFixed(3)}s。按 IMU 旋转显示；机身固定，未补偿帧间平移，未拼接地图。`;
}
