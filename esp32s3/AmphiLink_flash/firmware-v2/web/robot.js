
(() => {
  const status=document.querySelector("#robot-status");
  if (!status) return;
  const image=document.querySelector("#robot-image");
  let streaming=false, lastCommand=0;
  const controls=[...document.querySelectorAll("[data-robot-action], [data-robot-pitch]")];
  async function poll() {
    try {
      const response=await fetch("/api/v1/robot",{cache:"no-store"});
      if (!response.ok) throw new Error(response.status===401?"请先登录设备":"控制器状态暂不可用");
      const state=await response.json();
      status.textContent=state.online?(state.ready?"STM32 已连接 · "+state.received+" 帧 · 往返约 "+(state.round_trip_us/1000).toFixed(1)+" ms":"STM32 正在初始化"):"STM32 链路离线 · LoRa 控制保持";
      controls.forEach(button=>button.disabled=!(state.online&&state.ready&&state.wifi_control_allowed));
      const text=(id,value)=>document.querySelector(id).textContent=value;
      const voltage=state.battery_valid && state.battery_voltage!==null &&
        Number.isFinite(Number(state.battery_voltage)) ? Number(state.battery_voltage).toFixed(2)+" V" : "";
      const batteryText=!state.online ? "电压采样暂不可用" :
        state.battery_state===1 ? voltage+" · 请充电，已停机" :
        state.battery_state===2 ? voltage+" · 电压异常，已停机" :
        state.battery_state===3 || !voltage ? "电压采样异常，已停机" : voltage;
      text("#robot-battery",batteryText);
      text("#robot-control-source",state.wifi_control_allowed ?
        "2.4G 备用控制可用" : "LoRa 主控 · 2.4G 备用控制待确认心跳规则");
      text("#robot-tail",state.tail1+"° / "+state.tail2+"°");
      text("#robot-fins",state.left_fin+"° / "+state.right_fin+"°");
      text("#robot-esc",state.left_esc+"% / "+state.right_esc+"%");
      if (lastCommand && state.command_ack===lastCommand) {
        text("#robot-command",state.command_result===1?"控制器已接受指令":"控制器拒绝了指令");
        lastCommand=0;
      } else if (lastCommand && state.command_id===lastCommand && state.command_result===3) {
        text("#robot-command","指令确认超时，请查看连接后重新操作");lastCommand=0;
      }
      document.querySelector("#robot-stream").disabled=!state.camera;
      if (!state.camera && !streaming) text("#robot-camera-status","摄像头尚未就绪");
    } catch (error) {
      status.textContent=error.message;controls.forEach(button=>button.disabled=true);
    } finally { setTimeout(poll,500); }
  }
  async function submit(command) {
    const result=document.querySelector("#robot-command");
    result.textContent="正在提交…";
    try {
      const response=await fetch("/api/v1/robot/command",{
        method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(command)
      });
      const data=await response.json();
      if (!response.ok) throw new Error(data.error?.code||"提交失败");
      lastCommand=data.id;result.textContent="已提交，等待控制器确认";
    } catch (error) {result.textContent=error.message;}
  }
  controls.forEach(button=>button.addEventListener("click",()=>{
    if (button.dataset.robotPitch) submit({type:3,pitch:Number(button.dataset.robotPitch)});
    else submit({type:1,action:Number(button.dataset.robotAction),
      mode:Number(document.querySelector("#robot-mode").value),
      power:Number(document.querySelector("#robot-power").value),
      strength:Number(document.querySelector("#robot-strength").value)});
  }));
  document.querySelector("#robot-stream").addEventListener("click",()=>{
    streaming=!streaming;
    if (streaming) {
      image.src=location.protocol+"//"+location.hostname+":81/stream";image.hidden=false;
      document.querySelector("#robot-camera-status").hidden=true;
    } else {
      image.removeAttribute("src");image.hidden=true;
      document.querySelector("#robot-camera-status").hidden=false;
    }
    document.querySelector("#robot-stream").textContent=streaming?"关闭实时图像":"开启实时图像";
  });
  poll();
})();
