const assert=require('node:assert/strict');
const {livePose}=require('./localization.js');
const row={online:true,active:false,board_time:100,
 localization:{active_map:'indoor',service:'active',started_at:90,status:{fresh:true,stamp:100,code:0,mode:'全局',label:'正常'}},
 streams:{localization_pose:{frame:'map',age:0,stamp:100,stamp_age_s:.02,xyz:[1,2,3],yaw:0}}};
assert(livePose(row,'indoor').ok);
assert(!livePose(row,'other-map').ok);
for(const edit of [r=>r.online=false,r=>r.active=true,r=>r.localization.status.code=3,
 r=>r.localization.status.mode='局部',r=>r.board_time=106,r=>r.streams.localization_pose.age=4,
 r=>r.streams.localization_pose.xyz[0]=NaN,r=>r.streams.localization_pose.frame='lidar_link',
 r=>r.streams.localization_pose.stamp=80,r=>r.streams.localization_pose.error='time fault']){
 const changed=structuredClone(row);edit(changed);assert(!livePose(changed,'indoor').ok);
}
console.log('LOCALIZATION_VIEW_CHECK_OK');
