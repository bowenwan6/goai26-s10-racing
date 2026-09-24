/* Live pose overlay for the existing full-cloud viewer. No robot commands. */
(() => {
  'use strict';
  function livePose(row, expectedMap) {
    const loc=row?.localization, status=loc?.status, p=row?.streams?.localization_pose;
    if(!row?.online)return {ok:false,label:'连接中断，位置已隐藏'};
    if(row.active)return {ok:false,label:'正在建图，定位已暂停'};
    if(loc?.active_map!==expectedMap)return {ok:false,label:'激活地图已变化，请加载对应地图'};
    if(loc.service!=='active')return {ok:false,label:'定位服务未运行'};
    const statusAge=row.board_time-status?.stamp;
    if(!status?.fresh||!Number.isFinite(statusAge)||statusAge<-.25||statusAge>5)
      return {ok:false,label:'定位状态过期，位置已隐藏'};
    if(status.code!==0||status.mode!=='全局')
      return {ok:false,label:`${status.label} · ${status.mode}，位置已隐藏`};
    if(!p||p.frame!=='map'||p.error||!Number.isFinite(p.age)||p.age>=3||
       !Number.isFinite(p.stamp_age_s)||p.stamp_age_s<-.25||p.stamp_age_s>1||
       !Number.isFinite(p.stamp)||p.stamp<loc.started_at||
       !Array.isArray(p.xyz)||p.xyz.length!==3||!p.xyz.every(Number.isFinite)||!Number.isFinite(p.yaw))
      return {ok:false,label:'等待有效定位坐标，位置已隐藏'};
    return {ok:true,label:'正常 · 全局定位',pose:p};
  }
  if(typeof module!=='undefined'){module.exports={livePose};return;}

  const plot=document.getElementById('plot'), header=document.querySelector('header');
  const displayTop=meta.bounds[1][2]+.3;
  document.title='48号 · 实时定位';
  header.querySelector('h1').textContent='48号 · 狗的实时位置';
  const panel=document.createElement('section');
  panel.innerHTML='<div id="liveState" role="status" style="font-size:19px;font-weight:700;color:#a35913">正在连接定位…</div>'+
    '<p id="liveCoords">等待实时坐标</p><nav><button id="followDog" aria-pressed="false">跟随狗：关</button>'+
    '<a href="/">返回手机建图</a><a id="liveLogin" href="/localization" hidden>重新登录</a></nav>'+
    '<p class="note">红色定位针标出狗的位置，箭头表示朝向；针脚对应实际高度。约每秒更新位置，保留当前视角。</p>';
  header.insertBefore(panel,header.children[1]);
  header.querySelectorAll(':scope > p:not([id])').forEach(p=>p.remove());
  const settings=document.createElement('details'), summary=document.createElement('summary');
  summary.textContent='高度配色设置';settings.style.marginTop='10px';summary.style.cursor='pointer';
  const colors=document.getElementById('colors');colors.before(settings);
  settings.append(summary,colors,document.getElementById('colorNote'));
  const statusEl=document.getElementById('liveState'), coords=document.getElementById('liveCoords');
  let dogId,arrowId,follow=false,lastDataAt=0,current=null,stale=true;
  function hide(label){
    statusEl.textContent=label;statusEl.style.color='#a35913';coords.textContent='位置未显示';
    current=null;window.s10LiveReady=null;
    if(!stale&&dogId!==undefined)Plotly.restyle(plot,{visible:false},[dogId,arrowId]);
    stale=true;
  }
  function followDog(p){
    if(!p)return;
    return Plotly.relayout(plot,{'scene.xaxis.range':[p.xyz[0]-5,p.xyz[0]+5],
      'scene.yaxis.range':[p.xyz[1]-5,p.xyz[1]+5]});
  }
  document.getElementById('followDog').onclick=()=>{
    follow=!follow;const b=document.getElementById('followDog');
    b.textContent='跟随狗：'+(follow?'开':'关');b.setAttribute('aria-pressed',String(follow));
    if(follow)followDog(current);
  };
  async function poll(){
    try{
      const response=await fetch('/phone/state',{cache:'no-store',signal:AbortSignal.timeout(4000)});
      if(response.status===401){document.getElementById('liveLogin').hidden=false;throw Error('登录已过期，请重新登录');}
      if(!response.ok)throw Error('读取定位失败');
      const row=await response.json();lastDataAt=performance.now();
      const live=livePose(row,window.s10LiveMap);
      if(!live.ok){hide(live.label);return;}
      current=live.pose;stale=false;
      const [x,y,z]=current.xyz,a=current.yaw,tip=[x+.9*Math.cos(a),y+.9*Math.sin(a),displayTop];
      const left=[tip[0]-.28*Math.cos(a-.5),tip[1]-.28*Math.sin(a-.5),displayTop];
      const right=[tip[0]-.28*Math.cos(a+.5),tip[1]-.28*Math.sin(a+.5),displayTop];
      const arrow=[current.xyz,[x,y,displayTop],tip,left,tip,right];
      await Plotly.restyle(plot,{x:[[x],arrow.map(p=>p[0])],y:[[y],arrow.map(p=>p[1])],
        z:[[displayTop],arrow.map(p=>p[2])],visible:true},[dogId,arrowId]);
      statusEl.textContent=live.label;statusEl.style.color='#087651';
      coords.textContent=`X ${x.toFixed(2)} m · Y ${y.toFixed(2)} m · Z ${z.toFixed(2)} m · 朝向 ${(a*180/Math.PI).toFixed(1)}°`;
      if(follow)await followDog(current);
      window.s10LiveReady={map:window.s10LiveMap,stamp:current.stamp,xyz:current.xyz,yaw:a};
    }catch(error){hide(error.message||'连接中断，正在重连');}
    finally{setTimeout(poll,1000);}
  }
  async function init(){
    if(!window.fullCloudReady){setTimeout(init,200);return;}
    const clouds=plot.data.map((t,i)=>t.mode==='markers'?i:null).filter(i=>i!==null);
    await Plotly.restyle(plot,{'marker.opacity':.25},clouds);
    document.getElementById('trail').checked=false;
    await Plotly.restyle(plot,{visible:false},[plot.data.length-1]);
    dogId=plot.data.length;arrowId=dogId+1;
    await Plotly.addTraces(plot,[
      {type:'scatter3d',mode:'markers+text',x:[],y:[],z:[],text:['狗'],textposition:'top center',
       textfont:{size:18,color:'#a30f35'},name:'狗的实时位置',marker:{size:10,color:'#e51b4c',line:{color:'#fff',width:2}},
       visible:false,showlegend:false,hovertemplate:'X %{x:.2f} m<br>Y %{y:.2f} m<extra>狗的位置 · 实际高度见页面坐标</extra>'},
      {type:'scatter3d',mode:'lines',x:[],y:[],z:[],line:{color:'#e51b4c',width:8},
       visible:false,showlegend:false,hoverinfo:'skip'}]);
    const fixedView={'scene.uirevision':window.s10LiveMap,
      'scene.camera':{eye:{x:0,y:0,z:2.1},up:{x:0,y:1,z:0},projection:{type:'orthographic'}}};
    for(const [i,axis] of ['x','y','z'].entries()){
      fixedView[`scene.${axis}axis.autorange`]=false;
      fixedView[`scene.${axis}axis.range`]=[meta.bounds[0][i],i===2?displayTop:meta.bounds[1][i]];
    }
    await Plotly.relayout(plot,fixedView);
    setInterval(()=>{if(lastDataAt&&performance.now()-lastDataAt>4500&&!stale)hide('连接中断，位置已隐藏');},500);
    poll();
  }
  init();
})();
