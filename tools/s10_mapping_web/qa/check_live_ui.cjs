// Read-only presentation evaluation of an archived real dashboard. No HTTP/SSH.
const fs=require('node:fs');const assert=require('node:assert/strict');
const G=require('../field.js');const d=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
const flow=G.workflow({connected:true,contract:d.health.ui_contract,features:d.health.features,demo:d.health.demo,
 live:d.live,session:d.listing.session,overview:d.listing.overview,fresh:true,now:d.live.board_time,
 stationary:true,remote:false,human:false,cloudReady:false,name:'',floor:''});
const result={read_only:true,hypothetical_stationary_checkbox:true,no_task_submitted:true,
 status:d.live.status,pose_source_age:d.live.pose?.stamp_age_s,diagnostic_reason:flow.gates.localize,
 confirm_reason:flow.gates.confirm,waypoint_reason:flow.gates.waypoint};
assert.equal(flow.gates.localize,'','current same-map session should allow stopped diagnostic');
assert.ok(flow.gates.confirm);assert.ok(flow.gates.waypoint);
console.log(JSON.stringify(result,null,2));
