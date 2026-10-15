const assert=require('node:assert/strict');
const fs=require('node:fs');const vm=require('node:vm');const path=require('node:path');
const base=path.join(__dirname,'../esp32s3/AmphiLink_flash/firmware-v2/web');
const code=fs.readFileSync(path.join(base,'robot.js'),'utf8');
const html=fs.readFileSync(path.join(base,'index.html'),'utf8');
async function check(state,expected,enabled) {
 const nodes=new Map();const buttons=[{dataset:{robotAction:'1'}},{dataset:{robotPitch:'90'}}];
 const element=selector=>{
  if(!nodes.has(selector)) {
   assert(html.includes('id="'+selector.slice(1)+'"'),selector);
   nodes.set(selector,{textContent:'',dataset:{},addEventListener(){},removeAttribute(){}});
  }
  return nodes.get(selector);
 };
 buttons.forEach(button=>button.addEventListener=()=>{});
 vm.runInNewContext(code,{document:{querySelector:element,querySelectorAll:()=>buttons},
  fetch:async()=>({ok:true,json:async()=>state}),setTimeout(){},
  location:{protocol:'http:',hostname:'device'}});
 await new Promise(setImmediate);
 assert.equal(nodes.get('#robot-battery').textContent,expected);
 buttons.forEach(button=>assert.equal(button.disabled,!enabled));
 assert(!nodes.get('#robot-battery').textContent.includes('NaN'));
}
(async()=>{
 const state={online:true,ready:true,camera:true,received:12,round_trip_us:2000,
  battery_valid:true,battery_voltage:7.401,battery_state:0,wifi_control_allowed:false,
  tail1:90,tail2:90,left_fin:90,right_fin:90,left_esc:0,right_esc:0};
 await check(state,'7.40 V',false);
 await check({...state,wifi_control_allowed:true},'7.40 V',true);
 await check({...state,online:false,battery_valid:false,battery_voltage:null},'\u7535\u538b\u91c7\u6837\u6682\u4e0d\u53ef\u7528',false);
 await check({...state,battery_voltage:6.59,battery_state:1},'6.59 V · 请充电，已停机',false);
 await check({...state,battery_voltage:8.41,battery_state:2},'8.41 V · 电压异常，已停机',false);
 await check({...state,battery_valid:false,battery_voltage:null,battery_state:3},'电压采样异常，已停机',false);
 console.log('PASS: battery display, invalid/offline state, backup control gate and DOM IDs');
})().catch(error=>{console.error(error);process.exitCode=1});
