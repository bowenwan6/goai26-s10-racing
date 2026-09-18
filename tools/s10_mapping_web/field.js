'use strict';
// Pure presentation rules are also exercised by Node tests, without a browser or robot.
const FieldGuide = (() => {
  // UI transport freshness is not ROS source freshness. The SSH-backed
  // dashboard takes about 3s; every submitted action is checked again on 106.
  const UI_LIVE_TTL_MS=8000;
  function liveFresh(age){return Number.isFinite(age)&&age>=0&&age<UI_LIVE_TTL_MS;}
  function localizationDetails(l,fresh){
    if(!l)return ['没有收到定位状态；可先检查后台连接。'];
    const p=l.pose||{},s=l.status||{},seconds=v=>Number.isFinite(v)?v.toFixed(3)+' 秒':'未知';
    const rows=[];
    if(!fresh)rows.push('网页状态超过8秒未刷新；等待连接，后台操作仍重新核对传感器数据。');
    if(s.fresh!==true)rows.push('厂商定位状态消息已过期/未知。');
    if(s.code!==0||s.mode!=='全局')rows.push(`厂商报告：${s.label||'未正常定位'}（Code ${s.code??'未知'}，${s.mode||'未知模式'}）。消息新鲜不等于已定位；30秒检查只诊断，不会重置定位。`);
    if(p.frame!=='map'||p.error||!Array.isArray(p.xyz)||p.xyz.length!==3||!p.xyz.every(Number.isFinite))rows.push('位姿格式/坐标系/源时间异常，不能使用该坐标。');
    if(!Number.isFinite(p.age)||p.age<-.05||p.age>.5||!Number.isFinite(p.stamp_age_s)||p.stamp_age_s<-.05||p.stamp_age_s>.5)rows.push('位姿接收/源时间未通过0.5秒数据检查。');
    rows.push(`位姿接收延迟 ${seconds(p.age)}；源时间延迟 ${seconds(p.stamp_age_s)}。这是时间指标，不是定位质量。`);
    return rows;
  }
  const names={selfcheck:'现场自检',load_map:'加载地图',localization_check:'30秒定位检查',confirm_overlay:'人工叠合确认',record:'短录制',waypoint:'标记位置',waypoint_revisit:'航点返回复测',finish:'汇总报告'};
  const kinds={stationary:'静止',straight:'平地直行',turn:'转向'};
  function progress(job){
    if(!job||!['QUEUED','RUNNING'].includes(job.state))return null;
    const stage=job.stage||'',duration=job.duration_seconds;
    let title=job.state==='QUEUED'?'请求已收到，排队中（还没开始）':(names[job.action]||'任务')+'处理中，请等待';
    if(job.state==='RUNNING'&&stage.includes('采样结束'))title='采样已结束，检查与保存中，请继续等待';
    else if(job.state==='RUNNING'&&stage.includes('保存结果文件'))title='正在保存结果文件，请等待完成';
    else if(job.state==='RUNNING'&&stage.includes('录制已停止'))title='录制已结束，正在核验文件，请继续等待';
    else if(job.state==='RUNNING'&&job.action==='record'&&(stage.includes('本地录制')||stage.includes('演示录制计时')))title=(duration?duration+'秒':'限时')+'录制处理中；结束后还要核验文件';
    else if(job.state==='RUNNING'&&['localization_check','waypoint','waypoint_revisit'].includes(job.action))title=(duration?duration+'秒':'静止')+'采样/处理期间，请保持机器人停稳';
    const match=job.state==='RUNNING'&&/采样 (\d+)\/(\d+) 秒/.exec(stage);
    return {title,detail:stage+'。按钮恢复且出现最终结果前，不要重复操作。',
      value:match?Math.min(Number(match[1]),Number(match[2])):null,max:match?Number(match[2]):null,
      caption:match?`后台已采样 ${match[1]} / ${match[2]} 秒；采样后还需检查与保存`:'正在处理；没有可靠剩余时间，不显示假倒计时'};
  }
  function sameBinding(a,b){return !!a&&!!b&&['robot_id','boot_id','map_identity','invocation'].every(k=>!!a[k]&&a[k]===b[k]);}
  function result(j){
    if(!j)return {title:'尚无操作结果',tone:'unknown',detail:'先完成①现场自检。'};
    const r=j.result||{},name=names[j.action]||j.action;
    if(['QUEUED','RUNNING'].includes(j.state))return {title:name+'：'+(j.state==='QUEUED'?'已收到请求，尚未开始':'正在执行，请等待'),tone:'unknown',detail:(j.stage||'')+'。不要重复点击；页面关闭不取消已接受的任务。'};
    if(j.state!=='SUCCEEDED')return {title:name+'失败'+(j.action==='waypoint'?'：此点未保存坐标':j.action==='record'?'：未取得合格录制':''),tone:'bad',detail:(j.error||j.stage||'任务未完成')+'。先解决原因，再重试这一项；不要继续下一步。可随时汇总已有失败证据。'};
    if(j.action==='finish')return {title:r.passed?'报告已生成：检查通过，仍未授权自主行走':'报告已生成，但验收未通过',tone:r.passed?'good':'bad',detail:`保存位置 ${r.saved_waypoint_count??'未知'} 条，其中草稿 ${r.draft_waypoint_count??'未知'} 条；有效导航航点 ${r.valid_waypoint_count??0} 个；合格录制类型 ${(r.recordings||[]).map(k=>kinds[k]||k).join('、')||'无'}。`+(r.reasons||[]).join('；')+'。报告是此刻快照；可以继续补点，之后重新汇总并下载新版。'};
    if(j.action==='waypoint')return {title:`${r.name||'位置'}：${r.draft?'草稿':'待审航点'}保存成功`,tone:'good',detail:`楼层/区段 ${r.floor||'未填写'}；XYZ ${(r.pose?.xyz||[]).map(v=>Number.isFinite(v)?v.toFixed(3):'?').join(', ')} m。`+(r.draft?'尚未核验参考点，不可直接导航。':'仍需人工路线审阅，不代表可自主执行。')+'继续下一点时更换名称；同名不会覆盖旧点。'};
    if(j.action==='waypoint_revisit'){
      const m=r.metrics||{},f=(v,n=1)=>Number.isFinite(v)?v.toFixed(n):'未知';
      return {title:`${r.source_name||'航点'}坐标对照已保存：`+(r.within_reference?'位置差在参考线内':'位置差超过参考线')+(r.demo?'（演示）':''),
        tone:r.demo||!r.reference_verified||r.alignment_mode==='nearby'||r.sampling_precision_sufficient===false?'unknown':r.within_reference?'good':'bad',
        detail:`水平 ${f(m.horizontal_m*100)} cm；高度差 ${f(m.vertical_m*100)} cm；朝向差 ${f(m.yaw_deg)}°（允许不同）。原WP未覆盖；这不是绝对精度或导航验收。`+(r.warnings||[]).join(' ')};
    }
    if(['selfcheck','localization_check','record'].includes(j.action)&&r.passed!==true)return {title:name+'完成，但质量未通过'+(r.demo?'（演示数据）':''),tone:'bad',detail:(r.reasons||r.checks?.filter(c=>c.state==='fail').map(c=>c.name+'：'+c.detail)||[]).join('；')||'未取得合格证据，请查看下面的检查项；不要把“执行完成”当成通过。'};
    const detail={selfcheck:'候选地图和设备条件已检查；这不代表目标图已加载。继续看②实际地图。',load_map:'目标图已启用，但定位还未验收。停稳后做③30秒检查，再核对实时叠合。',localization_check:'静止采样通过。还要加载预览，看XY地标和XZ高度/楼层，再勾选并保存人工确认。',confirm_overlay:'人工确认已保存。可录10秒静止片段或停稳标点；掉定位、重启或确认到期须重做检查。',record:'录制质量通过。到时停止的是录制器，不是机器人；先用手柄停稳再进行下一项。'};
    return {title:name+'成功',tone:'good',detail:detail[j.action]||'请核对结果后再继续。'};
  }
  function workflow(x){
    const {live:l,session:s,overview:o}=x, gates={};
    const sameMap=!!s&&l?.map_name===s.target;
    const bound=sameBinding(s?.binding,l);
    const pose=l?.pose,st=l?.status;
    const healthy=x.fresh&&l?.mapping_active===false&&l?.localization_active===true&&st?.fresh===true&&st.code===0&&st.mode==='全局'&&pose?.frame==='map'&&!pose.error&&
      Array.isArray(pose.xyz)&&pose.xyz.length===3&&pose.xyz.every(Number.isFinite)&&Number.isFinite(pose.age)&&pose.age>=-.05&&pose.age<=.5&&Number.isFinite(pose.stamp_age_s)&&pose.stamp_age_s>=-.05&&pose.stamp_age_s<=.5;
    const self=o?.selfcheck,selfSeen=self?.state==='SUCCEEDED',selfOK=selfSeen&&(self.result?.passed===true||x.demo);
    const eligible=o?.eligible_job, eligibleOK=!!eligible&&eligible.id===s?.eligible_check&&eligible.state==='SUCCEEDED'&&eligible.result?.passed===true;
    const now=x.now,checkAge=eligible?(now-eligible.updated):Infinity;
    const confirmed=eligibleOK&&s.approved_check===eligible.id&&Number.isFinite(s.approved_at)&&now-s.approved_at>=0&&now-s.approved_at<=1800;
    const base=!x.connected?'后台未连接。保持机器人Wi-Fi，重连后查看任务，不重复提交。':!s?'先在①新建本次会话并自检。':x.contract!==2?'后台版本未就绪，请联系维护者；不要继续现场操作。':'';
    const realtime=base||(!x.fresh?'实时状态已过期/不可用，等待新数据；不能用旧绿灯继续。':'');
    const structureGate=realtime||(!selfSeen?'先在①自检绑定设备与地图。':!sameMap?'当前仍不是目标图。先完成②加载；不要录制或标点。':!bound?'地图/设备/定位会话已改变，先在①重新自检，再做③检查。':l?.mapping_active!==false?'建图仍在运行或状态未知。':l?.localization_active!==true?'定位服务未运行，先恢复服务后检查。':'');
    const diagnostics=localizationDetails(l,x.fresh);
    const mapGate=structureGate||(!healthy?diagnostics.join('\n'):'');
    gates.selfcheck=!x.connected?'后台未连接。':'';
    gates.recheck=!x.connected?'后台未连接。':!s?'尚未建立会话。':'';
    gates.finish=!x.connected?'后台未连接；连接恢复后可导出已有证据。':!s?'尚未建立会话。':'';
    gates.load=realtime||(!selfOK?'先在①自检通过必需项。':!x.stationary?'先用手柄停稳，勾选停稳确认。':!x.remote?'确认手柄可接管，所有 policy/导航程序已退出，再勾选。':l?.mapping_active!==false?'建图仍在运行或状态未知，不能切图。':!sameMap&&l?.navigation?.idle!==true?'停稳/导航检查未通过，见下方实时诊断；不要重复点击。':'');
    gates.localize=structureGate||(!x.features?.includes('field_workflow_v2')?'诊断后台升级未就绪，请刷新。':!x.stationary?'先用手柄停稳并勾选确认，检查期间保持30秒不动。':'');
    gates.confirm=mapGate||(!eligibleOK||checkAge<0||checkAge>300?'先重新完成30秒检查；结果需在5分钟内确认。':!x.cloudReady?'点击“加载目标地图显示资源”，等待同图实时扫描，再核对XY和XZ。':!x.human?'亲眼确认地标、方向和楼层对应，再勾选人工确认。':'');
    const reviewGate=mapGate||(!confirmed?'先完成③30秒检查与人工确认；已失效或超过30分钟时重新检查。':'');
    gates.record=reviewGate||(!selfOK?'自检中仍有录制/设备检查未通过；先查看①具体项目。':!x.remote?'请确认手柄接管可用，且没有其他策略发命令。':'');
    gates.waypoint=reviewGate||(!x.stationary?'让机器人到达该点并停稳，勾选停稳确认。':!x.name?'填写本点名称，例如WP01；保存后去下一点再换名称。':!x.floor?'填写楼层或平地区段，不能留空或猜测高度。':'');
    const reference=x.revisitPoint;
    gates.revisit=reviewGate||(!x.features?.includes('waypoint_revisit_v1')?'后台尚未支持返回复测，请等待升级。':!reference?'先保存航点或恢复原会话，并选择原航点。':
      reference.binding?.map_identity!==l?.map_identity||reference.binding?.robot_id!==l?.robot_id?'原航点与当前机器人/地图不同，不能比较。':
      !x.stationary?'先用手柄停稳，并勾选②停稳确认。':!x.remote?'先确认其他策略退出、手柄可接管。':
      !x.revisitPhysical?'请确认已到达所选实体标记附近并停稳；不要求精确位置或朝向一致。':'');
    let title,detail,tone='unknown';
    if(base){title='暂不能继续';detail=base;}
    else if(!x.fresh){title='实时状态未知：请停稳等待';detail=realtime;}
    else if(!selfSeen){title='下一步：①现场自检';detail='自检通过只代表准备条件可用，不代表地图已加载。';}
    else if(!sameMap){title='卡在②：目标地图尚未加载';detail='不要录制或标点。'+(gates.load||'停稳并确认其他策略退出后，点击②加载目标图。');tone='bad';}
    else if(!bound||!healthy){title='③定位尚未通过：可以先诊断';detail=mapGate+'\n'+(gates.localize||'可点击30秒检查查看证据；检查不会自动修复或强制放行。');tone='bad';}
    else if(!confirmed){title='下一步：③静止检查＋人工叠合';detail='先检查30秒，再看XY地标和XZ楼层，勾选后保存人工确认。';}
    else {title='可继续：④短录制 / ⑤标点';detail='先验证10秒试录和一个试标点，再继续全程。已保存的位置是草稿，不是自主导航授权。';tone='good';}
    return {gates,title,detail,tone,sameMap,bound,healthy,confirmed,diagnostics};
  }
  return {names,kinds,result,workflow,sameBinding,progress,liveFresh,UI_LIVE_TTL_MS,localizationDetails};
})();
if(typeof module!=='undefined'&&module.exports)module.exports=FieldGuide;
if(typeof document!=='undefined')(() => {
  const $ = id => document.getElementById(id);
  let csrf='', session=null, jobs=[], connected=false, busy=false, posting=false, selected=null;
  let live=null, liveAt=0, preview=null, previewLoading=false, checkId=null;
  let autoRestoreChecked=false;
  let overview=null,contract=null,demo=false,flow=null,activeJob=null,features=[];
  let detailVersion='',detailPending=null;
  let view='xy', scale=10, center=[0,0], drag=null;
  const actionIds=['selfcheck','recheck','load','localize','confirm','record','waypoint','revisit','finish'];
  const buttonLabels=Object.fromEntries(actionIds.map(id=>[id,$(id).textContent]));
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
  function selectedRevisit(){return (overview?.saved_points||[]).find(p=>p.job_id===$('revisitPoint').value);}
  function buttons() {
    flow=FieldGuide.workflow({connected,contract,demo,session,overview,live,fresh:!!live&&FieldGuide.liveFresh(performance.now()-liveAt),
      cloudReady:validCloud(),stationary:$('stationary').checked,remote:$('remote').checked,human:$('human').checked,
      features,revisitPoint:selectedRevisit(),revisitPhysical:$('revisitPhysical').checked,revisitNote:$('revisitNote').value.trim(),
      name:$('pointName').value.trim(),floor:$('floor').value.trim(),now:live?.board_time||Date.now()/1000});
    if(!flow.healthy)$('revisitPhysical').checked=false;
    const running=FieldGuide.progress(activeJob),activeId={selfcheck:'selfcheck',load_map:'load',localization_check:'localize',confirm_overlay:'confirm',record:'record',waypoint:'waypoint',waypoint_revisit:'revisit',finish:'finish'}[activeJob?.action];
    actionIds.forEach(id=>{
      const reason=posting?'请求正在提交，请等待结果，不要重复点击。':busy?(running?.title||'机器人上有任务执行中，请等它完成。'):flow.gates[id];
      $(id).disabled=!!reason;
      $(id).textContent=busy&&id===activeId?'⏳ '+(running?.title||'处理中，请等待'):buttonLabels[id];
      $(id).setAttribute('aria-busy',busy&&id===activeId?'true':'false');
      const hint=$(id+'Hint');if(hint){$(id).setAttribute('aria-describedby',id+'Hint');hint.textContent=reason||'条件已就绪，可以点击。';hint.className='gate '+(reason?'unknown':'good');}
    });
    $('busyPanel').hidden=!busy&&!posting;
    $('busyTitle').textContent=posting?'请求正在提交，尚未确认开始':running?.title||'任务处理中，请等待';
    $('busyDetail').textContent=posting?'请保持连接并等待机器人返回任务记录，不要重复点击。':running?.detail||'等待后台返回真实进度。';
    $('progressCaption').textContent=running?.caption||'提交请求不等于已经开始采样或录制。';
    if(running?.value!==null&&running?.value!==undefined&&running.max>0){$('taskProgress').max=running.max;$('taskProgress').value=running.value;}
    else $('taskProgress').removeAttribute('value');
    $('draft').disabled=live?.calibration?.verified!==true;if($('draft').disabled)$('draft').checked=true;
    $('restore').disabled=posting||busy;
    for(const id of ['revisitPoint','revisitPhysical','revisitNote','revisitRuler','revisitMode'])$(id).disabled=posting||busy;
    $('localizationDiagnostic').textContent=flow.diagnostics.join('\n');
    $('targetIdentity').textContent='本会话目标：'+(session?.target||$('target').value.trim());
    $('currentIdentity').textContent='机器人实际加载：'+(live?.map_name||'未知（不要继续录制/标点）');
    $('flowTitle').textContent=busy?'正在执行任务，请不要重复操作':flow.title;
    $('flowDetail').textContent=busy?(activeJob?.stage||'等待后台返回真实结果；不会自动重试。'):flow.detail;
    $('workflow').className='card workflow '+flow.tone;
    const n=live?.navigation,vec=v=>Array.isArray(v)?v.map(x=>Number.isFinite(x)?x.toFixed(4):'?').join(' / '):'未知';
    const navFresh=!!live&&FieldGuide.liveFresh(performance.now()-liveAt)&&n?.fresh===true;
    $('navigationDiagnostic').textContent=(navFresh?'实时导航诊断：':'导航证据未知/过期：')+(n?.reason||'等待后台提供监视数据。')+
      '\n规划器命令 X / Y / 转向：'+vec(n?.command)+'\n运动反馈 X / Y / 转向：'+vec(n?.motion)+
      '\n数值为厂商日志原值，不是已核验的物理速度。网页不确认外部 policy 已退出。'+
      (navFresh&&n?.idle?'\n反馈预检满足条件（不是已经停稳）；点击加载后还要保持不动，完成'+(n?.stop_sample_seconds||3)+'秒位姿/IMU检查。':'\n先让所有策略退出、用手柄停稳；仍不通过时联系维护者核实反馈，不要反复点击或跳过检查。');
    renderOverview();
  }
  function renderOverview(){
    const pts=overview?.saved_points||[];
    $('wpCount').textContent=`本会话已保存 ${overview?.saved_waypoint_count||0} 条位置记录（不是目标数量上限）。失败的点击不计入。`;
    const rows=pts.map(p=>{const li=document.createElement('li');li.textContent=`${p.name} · ${p.floor} · XYZ ${p.xyz.map(v=>Number.isFinite(v)?v.toFixed(3):'?').join(', ')} m · ${p.draft?'草稿，不能直接导航':'待人工路线审阅'}`+(FieldGuide.sameBinding(p.binding,live)?'':' · 历史绑定，不能当作当前定位授权');return li;});
    $('wpList').replaceChildren(...rows);
    const old=$('revisitPoint').value,signature=pts.map(p=>p.job_id+':'+p.name).join('|');
    if($('revisitPoint').dataset.signature!==signature){
      const options=pts.map(p=>{const op=document.createElement('option');op.value=p.job_id;op.textContent=`${p.name} · ${p.floor||''} · ${p.job_id.slice(0,8)}`;return op;});
      $('revisitPoint').replaceChildren(...options);$('revisitPoint').dataset.signature=signature;
      if(pts.some(p=>p.job_id===old))$('revisitPoint').value=old;
      if(old!==$('revisitPoint').value)$('revisitPhysical').checked=false;
    }
    const ref=selectedRevisit();
    $('revisitSource').textContent=ref?`原点 ${ref.name} · ${new Date(ref.created*1000).toLocaleString()} · 实体标记：${ref.marker_note||'原点未填写；请依据你现场留下的标记，无法确认则不要复测'}。同名点按编号区分。`:'先保存一个航点，或恢复该航点所在会话。';
    const comparable=ref&&flow?.healthy&&live?.robot_id===ref.binding?.robot_id&&live?.map_identity===ref.binding?.map_identity;
    $('revisitLive').textContent=comparable?`实时地图距离（仅接近提示）：水平 ${(Math.hypot(live.pose.xyz[0]-ref.xyz[0],live.pose.xyz[1]-ref.xyz[1])*100).toFixed(1)} cm，高度差 ${((live.pose.xyz[2]-ref.xyz[2])*100).toFixed(1)} cm。不要用“距离归零”代替实体对点。`:'实时接近提示不可用：需要同图的新鲜全局定位；历史结果不代表当前状态。';
    const visits=(overview?.revisits||[]).filter(v=>v.source_waypoint_id===ref?.job_id);
    $('revisitCount').textContent=`本会话已保存 ${overview?.revisit_count||0} 次复测；当前选择点 ${visits.length} 次。每次结果独立保存。`;
    $('revisitHistory').replaceChildren(...visits.map(v=>{const li=document.createElement('li'),button=document.createElement('button');button.textContent=`${new Date(v.created*1000).toLocaleTimeString()} · 水平 ${(v.metrics.horizontal_m*100).toFixed(1)} cm / 高度 ${(v.metrics.vertical_m*100).toFixed(1)} cm / 朝向 ${v.metrics.yaw_deg.toFixed(1)}° · 查看/下载`;button.onclick=()=>showJob(v.job_id);li.append(button);return li;}));
    const latest=overview?.latest_revisit;
    const r=latest?.result;
    if(latest&&(r?.source_waypoint_id===ref?.job_id||latest.request?.params?.waypoint_id===ref?.job_id)){
      const text=FieldGuide.result(latest);$('revisitResult').textContent='历史记录 · '+text.title+'。'+text.detail;$('revisitResult').className='result-box '+text.tone;
    }else{$('revisitResult').textContent='该点的历史复测见下面列表；没有记录时，先完成实体对点，再开始复测。';$('revisitResult').className='result-box';}
    $('wpWarning').textContent=overview?.duplicate_names?.length?'存在同名保存记录：'+overview.duplicate_names.join('、')+'。不会覆盖旧点；请在报告中人工选择。':'同名不会覆盖旧点。每次成功后换名称再标下一点；Z不是地面高度。';
    const rep=overview?.report;
    $('reportStatus').textContent=!rep?'还没汇总。完成第一段后可以先生成一份报告，之后继续补点。':overview.report_outdated?'有新操作未包含在上次报告中：请重新汇总并下载新版。':'上次汇总：'+FieldGuide.result(rep).title+'。汇总不会关闭会话，可继续补点。';
  }
  function safety() {
    if(!$('stationary').checked) throw Error('请先用手柄停稳并勾选确认。');
    return {stationary:true};
  }
  async function submit(action,params={}) {
    if(!connected||busy||posting) return;
    buttons();
    const ids={selfcheck:params.target_map===undefined?'recheck':'selfcheck',load_map:'load',localization_check:'localize',confirm_overlay:'confirm',record:'record',waypoint:'waypoint',waypoint_revisit:'revisit',finish:'finish'};
    if(flow.gates[ids[action]]){notice(flow.gates[ids[action]]);return;}
    let body;
    try {body={action,key:nonce(),params}; if(action!=='selfcheck'||params.target_map===undefined)body.session_id=session.id;}
    catch(e){notice('无法创建安全请求：'+e.message);return;}
    if(action==='waypoint_revisit')$('revisitPhysical').checked=false;
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
  async function restore(id,automatic=false) {
    if(!id)return;
    $('revisitPhysical').checked=false;
    const candidate=await api('/phone/field/session?session_id='+id);
    if(automatic&&candidate.target!==$('target').defaultValue){
      notice('旧会话属于 '+candidate.target+'，没有自动恢复。当前基准为 '+$('target').defaultValue+'，请新建本次会话；旧记录仍可手动查看。');
      return;
    }
    session=candidate;remember(id);
    overview=null;selected=null;
    $('target').value=session.target;$('sessionId').textContent='会话 '+session.id;
    preview=null;checkId=null;$('human').checked=false;
    await refreshJobs();buttons();
  }
  async function refreshJobs(supplied=null) {
    const requestedSession=session?.id||null;
    const data=supplied||await api('/phone/field/list'+(session?'?session_id='+session.id:''));
    if((session?.id||null)!==requestedSession)return; // Ignore a late response for a different restored session.
    const sessions=new Map();(data.sessions||[]).forEach(s=>sessions.set(s.id,s));data.jobs.forEach(j=>{if(!sessions.has(j.session_id))sessions.set(j.session_id,j);});
    const options=[];sessions.forEach((j,id)=>{const option=document.createElement('option');option.value=id;option.textContent=new Date(j.created*1000).toLocaleString()+' · '+(j.target||id.slice(0,8));options.push(option);});
    $('sessionList').replaceChildren(...options);if(session)$('sessionList').value=session.id;
    activeJob=data.active_job||null;busy=!!activeJob||data.jobs.some(j=>['QUEUED','RUNNING'].includes(j.state));
    if(!session&&!autoRestoreChecked&&data.jobs.length){autoRestoreChecked=true;const saved=remembered();await restore(sessions.has(saved)?saved:data.jobs[0].session_id,true);return;}
    jobs=data.jobs.filter(j=>session&&j.session_id===session.id);
    if(data.session&&data.session.id===session?.id)session=data.session;
    overview=data.overview||null;
    // Only the worker's still-eligible check can be confirmed. Never fall back
    // to a previous success after loss, restart, or a failed newer check.
    const eligible=data.eligible_checks?.[session?.id];
    const checked=overview?.eligible_job||jobs.find(j=>j.id===eligible);
    checkId=checked?.id===eligible&&checked?.state==='SUCCEEDED'&&checked?.result?.passed?checked.id:null;
    const rows=jobs.map(j=>{
      const row=document.createElement('div');row.className='job';
      const summary=FieldGuide.result(j);
      const label=document.createElement('div');label.className=summary.tone;label.textContent=`${new Date(j.created*1000).toLocaleTimeString()} · ${summary.title}`;
      const stage=document.createElement('div');stage.className='sub';stage.textContent=summary.detail;
      const button=document.createElement('button');button.textContent='查看结果 / 下载';button.onclick=()=>showJob(j.id);
      row.append(label,stage,button);return row;
    });
    $('jobs').replaceChildren(...rows);if(!rows.length)$('jobs').textContent='当前会话暂无任务';
    if(!selected&&jobs.length)selected=jobs[0].id;
    const selectedRow=jobs.find(j=>j.id===selected);
    if(selected&&selectedRow&&detailVersion!==selected+':'+selectedRow.updated&&detailPending!==selected){
      showJob(selected).catch(e=>notice('详情读取失败：'+e.message+'。实时状态另行更新；稍后可再查看详情。'));
    }
    const check=overview?.selfcheck||jobs.find(j=>j.action==='selfcheck'&&j.result);
    $('checks').replaceChildren(...(check?.result?.checks||[]).map(c=>{const d=document.createElement('div');d.className='check '+(c.state==='pass'?'good':c.state==='fail'?'bad':'unknown');d.textContent=`${c.name} · ${{pass:'通过',fail:'未通过',unknown:'待核实',not_required:'不需要'}[c.state]||c.state} — ${c.detail}`;return d;}));
    $('historyScope').textContent=overview?`这里显示最近 ${jobs.length} / ${overview.total_jobs} 条操作。WP清单和新汇总覆盖整个会话，不受100条列表上限限制。`:'';
    buttons();
  }
  async function showJob(id) {
    selected=id;detailPending=id;
    let j;
    try{j=await api('/phone/field/job?job_id='+id);}finally{if(detailPending===id)detailPending=null;}
    if(selected!==id)return;
    detailVersion=id+':'+j.updated;
    const summary=FieldGuide.result(j);
    $('resultTitle').textContent=summary.title;$('resultTitle').className=summary.tone;
    $('resultExplanation').textContent=summary.detail;
    if(j.id===jobs[0]?.id)notice(summary.title+'。'+summary.detail);
    $('detail').textContent=JSON.stringify(j,null,2);
    const box=$('downloads');box.replaceChildren();
    (j.artifacts||[]).forEach(a=>{const p=document.createElement('p'),link=document.createElement('a');link.href='/phone/field/download?artifact_id='+a.id;link.textContent=`下载 ${a.name}（${(a.size/1048576).toFixed(2)} MiB）`;link.download=a.name;p.append(link);const hash=document.createElement('div');hash.className='sub';hash.textContent='SHA256 '+a.sha256;p.append(hash);box.append(p);});
  }
  async function loadPreview() {
    if(!session||previewLoading)return;
    previewLoading=true;
    try{notice('读取目标地图抽样，原始地图文件不会改变…');preview=await api('/phone/field/preview?session_id='+session.id);fit();notice('地图抽样已加载。只有同地图身份、新鲜同map扫描才会叠加。');}
    catch(e){notice('地图叠合不可用：'+e.message);preview=null;}
    finally{previewLoading=false;draw();buttons();}
  }
  function validLive(){
    const p=live?.pose,s=live?.status;
    return connected&&FieldGuide.liveFresh(performance.now()-liveAt)&&live?.map_name===session?.target&&live?.map_identity===preview?.map_identity&&
      live?.mapping_active===false&&live?.localization_active===true&&s?.fresh===true&&s?.code===0&&s?.mode==='全局'&&
      p?.frame==='map'&&Array.isArray(p.xyz)&&p.xyz.length===3&&p.xyz.every(Number.isFinite)&&Number.isFinite(p.age)&&p.age>=-.05&&p.age<=.5&&
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
    const ref=selectedRevisit();
    for(const p of overview?.saved_points||[]){
      if(p.binding?.map_identity!==preview?.map_identity)continue;
      point(p.xyz,p.job_id===ref?.job_id?12:7,'#6c9dff');
      ctx.fillStyle='#aac7ff';ctx.font='13px sans-serif';
      ctx.fillText(p.name.slice(0,30),w/2+(p.xyz[0]-center[0])*scale+8,h/2-(p.xyz[second]-center[1])*scale-8);
    }
    if(ref&&ref.binding?.map_identity===preview?.map_identity){
      const x=w/2+(ref.xyz[0]-center[0])*scale,y=h/2-(ref.xyz[second]-center[1])*scale;
      if(good){ctx.strokeStyle='#ffc36b';ctx.setLineDash([7,5]);ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(w/2+(live.pose.xyz[0]-center[0])*scale,h/2-(live.pose.xyz[second]-center[1])*scale);ctx.stroke();ctx.setLineDash([]);}
      for(const v of overview?.revisits||[])if(v.source_waypoint_id===ref.job_id&&v.binding?.map_identity===preview.map_identity){
        point(ref.xyz.map((a,i)=>a+v.metrics.delta_map_m[i]),8,'#d99bf5');
      }
    }
    ctx.fillStyle='#dbe6df';ctx.font='18px sans-serif';ctx.fillText(view==='xy'?'XY 俯视 · 请同时核对高度':'XZ 侧视 · 核对楼层/坡道',16,30);
    if(!good){ctx.fillStyle='#ffc58a';ctx.fillText('没有可确认的新鲜同图定位/扫描；当前显示仅是目标地图',16,h-20);}
    $('liveQuality').textContent=!preview?'显示资源尚未加载：点击“加载目标地图显示资源”。后台检查/授权状态见页面顶部；这里只显示叠图状态。':good?(validCloud()?'程序状态：新鲜正常全局定位与同图扫描；仍需人工核对，不证明整条路线精度。':'位姿新鲜但扫描叠合不可用/异常，不能提交人工叠合确认。'):FieldGuide.localizationDetails(live,FieldGuide.liveFresh(performance.now()-liveAt)).join(' ');
    $('liveQuality').className=good?'good':'unknown';
  }
  async function poll(){
    try{
      const requestedSession=session?.id||null;
      const dashboard=await api('/phone/field/dashboard'+(session?'?session_id='+session.id:''));
      if((session?.id||null)!==requestedSession){setTimeout(poll,200);return;}
      const health=dashboard.health;csrf=health.csrf;connected=true;contract=health.ui_contract;features=health.features||[];demo=health.demo===true;$('demo').hidden=!demo;
      $('connection').textContent=health.demo?'演示后台在线 · 不是真机':'机器人后台已连接 · 手机关闭不取消已接受任务';
      live=dashboard.live;liveAt=performance.now();
      if(!live)notice('后台可用但实时证据不可用：'+dashboard.live_error+'。仍可汇总导出已有记录，不代表定位合格。');
      $('identity').textContent='当前地图：'+(live?.map_name||'未知')+'；定位会话：'+(live?.invocation||'未知');
      const p=live?.pose;$('pose').textContent=p?`实时参考点 XYZ: ${p.xyz?.map(v=>Number.isFinite(v)?v.toFixed(3):'?').join(', ')} m（不是已保存的WP，也不是地面高度） · frame=${p.frame||'空'} · child=${p.child_frame||'空（只能草稿）'}`:'尚无位姿';
      await refreshJobs(dashboard.listing);
    }catch(e){connected=false;live=null;$('connection').textContent='无法确认后台连接：'+e.message+'；已接受任务仍由机器人限时执行，重连只查询。';}
    buttons();draw();setTimeout(poll,200);
  }
  $('selfcheck').onclick=()=>submit('selfcheck',{target_map:$('target').value.trim()});
  $('recheck').onclick=()=>submit('selfcheck');
  $('restore').onclick=()=>restore($('sessionList').value).catch(e=>notice(e.message));
  function wrap(fn){return()=>{try{fn();}catch(e){notice(e.message);}};}
  $('load').onclick=wrap(()=>submit('load_map',{...safety(),remote_ready:$('remote').checked}));
  $('localize').onclick=wrap(()=>submit('localization_check',safety()));
  $('confirm').onclick=wrap(()=>{if(!checkId)throw Error('没有仍有效的静止检查；掉定位、重启或新检查失败后，请重新检查30秒。');if(!validCloud())throw Error('先加载预览并等同图新鲜扫描与定位；不能盲目确认。');submit('confirm_overlay',{check_id:checkId,confirmed:$('human').checked});});
  $('record').onclick=()=>submit('record',{kind:$('kind').value,seconds:Number($('seconds').value),remote_ready:$('remote').checked});
  $('waypoint').onclick=wrap(()=>{const params={...safety(),name:$('pointName').value.trim(),floor:$('floor').value.trim()||'未填写',segment:'flat',draft:$('draft').checked};if($('markerNote').value.trim())params.marker_note=$('markerNote').value.trim();submit('waypoint',params);});
  $('revisitPoint').onchange=()=>{$('revisitPhysical').checked=false;const ref=selectedRevisit();$('revisitNote').value=ref?.marker_note||'';buttons();draw();};
  $('revisit').onclick=wrap(()=>{
    if(!$('revisitRuler').checkValidity())throw Error('尺量值必须为0–1000厘米；没有测量请留空。');
    const params={...safety(),remote_ready:$('remote').checked,waypoint_id:$('revisitPoint').value,
      physical_confirmed:$('revisitPhysical').checked,alignment_mode:$('revisitMode').value};
    if($('revisitNote').value.trim())params.reference_note=$('revisitNote').value.trim();
    if($('revisitRuler').value.trim())params.ruler_offset_cm=Number($('revisitRuler').value);
    submit('waypoint_revisit',params);
  });
  $('finish').onclick=()=>submit('finish');$('preview').onclick=loadPreview;
  $('top').onclick=()=>{view='xy';fit();};$('side').onclick=()=>{view='xz';fit();};
  $('zoomIn').onclick=()=>{scale=Math.min(1000,scale*1.5);draw();};$('zoomOut').onclick=()=>{scale=Math.max(.1,scale/1.5);draw();};$('fit').onclick=fit;
  const canvas=$('map');canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY,...center];canvas.setPointerCapture(e.pointerId);$('follow').checked=false;};
  canvas.onpointermove=e=>{if(!drag)return;center=[drag[2]-(e.clientX-drag[0])*canvas.width/canvas.clientWidth/scale,drag[3]+(e.clientY-drag[1])*canvas.height/canvas.clientHeight/scale];draw();};
  canvas.onpointerup=()=>drag=null;canvas.onpointercancel=()=>drag=null;
  canvas.addEventListener('wheel',e=>{e.preventDefault();scale=Math.max(.1,Math.min(1000,scale*(e.deltaY<0?1.2:1/1.2)));draw();},{passive:false});
  ['stationary','remote','human','pointName','floor','target','revisitPhysical','revisitNote'].forEach(id=>$(id).addEventListener('input',buttons));
  setInterval(()=>{if(!FieldGuide.liveFresh(performance.now()-liveAt)){draw();buttons();}},500);
  buttons();poll();
})();
