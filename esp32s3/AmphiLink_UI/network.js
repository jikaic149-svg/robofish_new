'use strict';
let networkSnapshot={revision:0,items:[]},networkBusy=false;
let simNetworks={revision:1,items:[{id:1,ssid:'RoboFish-Lab (模拟)',priority:0,connected:true,has_password:true}]};
const SIM_SCAN=[{ssid:'RoboFish-Lab (模拟)',rssi:-42,channel:6,supported:true},{ssid:'实验室备用 (模拟)',rssi:-67,channel:11,supported:true},{ssid:'开放测试网络 (模拟)',rssi:-72,channel:1,supported:true}];
async function networkCall(action,payload={}){
  if(stationMode==='simulation'){
    if(action==='networks')return structuredClone(simNetworks);
    if(action==='scan')return {operation_id:1};if(action==='scan-result')return {state:'succeeded',items:SIM_SCAN};
    if(action==='connect')return {operation_id:2};if(action==='connect-result')return {state:'succeeded',ip:'仿真IP'};
    if(action==='network-add'){simNetworks.items.push({id:Math.max(0,...simNetworks.items.map(n=>n.id))+1,ssid:payload.ssid,priority:simNetworks.items.length,connected:false,has_password:!!payload.password});}
    if(action==='network-delete')simNetworks.items=simNetworks.items.filter(item=>item.id!==payload.id);
    if(action==='network-order')simNetworks.items=payload.ids.map(id=>simNetworks.items.find(n=>n.id===id));
    simNetworks.revision++;return structuredClone(simNetworks);
  }
  return service('device/network',{host:robotHost(),action,payload});
}
function renderNetworks(){
  const list=$('network-saved');list.replaceChildren();
  if(!networkSnapshot.items.length){list.textContent='没有已保存网络';return;}
  networkSnapshot.items.forEach((item,index)=>{
    const row=document.createElement('div');row.className='network-row';const text=document.createElement('span');text.textContent=`${index+1}. ${item.ssid}`;const detail=document.createElement('small');detail.textContent=(item.connected?'当前连接 · ':'')+(item.has_password?'密码已设置':'开放网络');text.append(detail);
    const actions=document.createElement('div');actions.className='button-row';
    const up=document.createElement('button');up.className='text-button';up.textContent='↑ 上移';up.disabled=index===0;up.onclick=()=>networkOperation(async()=>{const ids=networkSnapshot.items.map(n=>n.id);[ids[index-1],ids[index]]=[ids[index],ids[index-1]];networkSnapshot=await networkCall('network-order',{revision:networkSnapshot.revision,ids});renderNetworks();});
    const remove=document.createElement('button');remove.className='text-button';remove.textContent='删除';remove.onclick=()=>{if(confirm('删除已保存网络 '+item.ssid+'？'))networkOperation(async()=>{networkSnapshot=await networkCall('network-delete',{id:item.id,revision:networkSnapshot.revision});renderNetworks();});};
    actions.append(up,remove);row.append(text,actions);list.append(row);
  });
}
async function networkOperation(callback){
  if(networkBusy)return;networkBusy=true;const epoch=stationEpoch;
  $('network-operation').textContent=stationMode==='simulation'?'正在操作仿真网络…':'正在请求设备…';
  try{await callback();if(epoch===stationEpoch){$('network-operation').textContent=stationMode==='simulation'?'仿真操作完成；没有修改任何实物网络。':'管理操作完成';log('网络管理操作完成'+(stationMode==='simulation'?'（仿真）':''),'ok');}}
  catch(error){$('network-operation').textContent=error.message;toast(error.message);log('网络管理：'+error.message,'warn');}
  finally{networkBusy=false;}
}
async function waitNetworkResult(kind,id){
  const epoch=stationEpoch,deadline=Date.now()+20000;
  while(Date.now()<deadline){
    if(epoch!==stationEpoch)throw Error('工作模式已变化，结束等待；请重新读取当前设备状态');
    const result=await networkCall(kind+'-result',{id});
    if(result.state==='succeeded')return result;
    if(result.state==='failed'||result.state==='cancelled')throw Error('设备网络操作失败，错误码 '+result.error);
    await new Promise(resolve=>setTimeout(resolve,500));
  }
  throw Error('网络操作等待超时；设备可能已切换IP，请重新发现或手动填写地址。');
}
$('open-probe-form').onsubmit=event=>{
  event.preventDefault();const host=robotAddress($('management-ip').value);if(!host)return toast('请填写有效设备IPv4地址');
  $('robot-host').value=host;identityEpoch++;robotIdentity=null;if(!liveState.session_active)$('host-input').value=host.split(':')[0];
  $('management-result').textContent='已选择 '+host+'，正在检测…';
  pollIdentity().then(()=>{$('management-result').textContent=robotIdentity?'设备已发现；登录后直接在本页配网。':'找不到设备，请检查热点、局域网或IP地址。';});
};
$('network-refresh').onclick=()=>networkOperation(async()=>{networkSnapshot=await networkCall('networks');renderNetworks();});
$('network-scan').onclick=()=>networkOperation(async()=>{
  $('network-scan-state').textContent='扫描中，最多等待20秒…';
  const job=await networkCall('scan'),result=await waitNetworkResult('scan',job.operation_id),list=$('network-scan-results');list.replaceChildren();
  for(const item of result.items||[]){const row=document.createElement('div');row.className='network-row';const label=document.createElement('span');label.textContent=item.ssid||'隐藏网络';const info=document.createElement('small');info.textContent=`${item.rssi} dBm · 信道 ${item.channel}`;label.append(info);const choose=document.createElement('button');choose.className='text-button';choose.textContent=item.supported?'选择':'不支持';choose.disabled=!item.supported;choose.onclick=()=>{$('network-ssid').value=item.ssid;$('network-password').focus();};row.append(label,choose);list.append(row);}
  $('network-scan-state').textContent=`${stationMode==='simulation'?'模拟扫描':'扫描完成'} · ${result.items?.length||0}个网络`;
}).finally(()=>{if($('network-scan-state').textContent.startsWith('扫描中'))$('network-scan-state').textContent='扫描未完成，请查看操作结果。';});
$('network-save-form').onsubmit=event=>{event.preventDefault();networkOperation(async()=>{
  networkSnapshot=await networkCall('networks');networkSnapshot=await networkCall('network-add',{revision:networkSnapshot.revision,ssid:$('network-ssid').value,password:$('network-password').value});$('network-password').value='';renderNetworks();
});};
$('network-connect').onclick=()=>networkOperation(async()=>{const job=await networkCall('connect',{ssid:$('network-ssid').value,password:$('network-password').value});$('network-password').value='';const result=await waitNetworkResult('connect',job.operation_id);$('management-result').textContent='设备报告连接成功'+(result.ip?' · '+result.ip:'')+'；请让电脑加入同一网络后重新检测。';});
setTimeout(()=>{if(!Object.keys(probeLinks.wifi||{}).length&&!$('management-ip').value)$('management-result').textContent='找不到设备：请连接探针热点或手动填写IP。测试模式无需实物。';},6000);
