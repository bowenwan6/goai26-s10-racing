// Run: node tools/s10_gait_capture/test_point_overlay.cjs (no browser or ROS).
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const source=fs.readFileSync(__dirname+'/web/app.js','utf8');
const nodes={pointSide:{value:'both'},pointCanvas:{getBoundingClientRect:()=>({width:0})}};
const sandbox={performance:{now:()=>1000},$:id=>nodes[id]??={},
 ResizeObserver:class{observe(){}},setInterval(){}};
vm.createContext(sandbox);
vm.runInContext(source.slice(source.indexOf('let pointFrames=')),sandbox);
const frame={fresh:true,age_s:.1,receivedAt:1000,frame_id:'base_link',points:[[1,2,3]],total:1};
const scene=(frames,selection='both',now=1000)=>sandbox.pointScene(frames,selection,now);
assert.equal(scene({front:frame,rear:frame}).layers.length,2);
assert.equal(scene({front:frame,rear:frame}).robot,true);
assert.equal(scene({front:frame,rear:frame},'rear').layers[0].key,'rear');
assert.equal(scene({front:frame,rear:{...frame,frame_id:'rear_lidar'}}).layers.length,0);
assert.match(scene({front:frame,rear:{...frame,frame_id:'rear_lidar'}}).message,/坐标系不同/);
assert.equal(scene({front:{...frame,frame_id:'map'}},'front').robot,false);
assert.equal(scene({front:{...frame,frame_id:''},rear:{...frame,frame_id:''}}).layers.length,0);
assert.equal(scene({front:frame,rear:{fresh:false,message:'后雷达连接失败'}}).layers.length,1);
assert.match(scene({front:frame,rear:{fresh:false,message:'后雷达连接失败'}}).message,/后雷达连接失败/);
assert.equal(scene({front:frame,rear:frame},'both',3001).layers.length,0);
assert.equal(scene({}).robot,false);
console.log('Point overlay, coordinate checks, single-lidar fallback and expiry checks passed');
