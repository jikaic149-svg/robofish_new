'use strict';
let recorded={rows:[],recording:false,count:0,dropped:0},dataPending=false,chartFrozen=null,chartHover=-1,lastCaptureError='';
const PLOTS={battery:[['battery_v','电池电压','#178b7d']],esc:[['left_esc_pct','左螺旋桨','#178b7d'],['right_esc_pct','右螺旋桨','#637bc5']],servo:[['left_fin_deg','左胸鳍','#178b7d'],['right_fin_deg','右胸鳍','#637bc5'],['tail1_deg','尾舵1','#b27c3c'],['tail2_deg','尾舵2','#b56d94']],latency:[['latency_ms','通信往返','#178b7d']]};
const UNITS={battery:'V',esc:'%',servo:'°',latency:'ms'};
function renderCapture(){
  const sourceLabel=({simulation:'仿真数据',bench:'台架遥测',mission:'LoRa遥测'})[stationMode];
  document.querySelectorAll('.capture-count').forEach(el=>el.textContent=recorded.count.toLocaleString());
  document.querySelectorAll('.capture-status').forEach(el=>{el.textContent=recorded.recording?'正在记录':'未记录';el.dataset.state=recorded.recording?'online':'unknown';});
  robotText('capture-capacity',`${(recorded.capacity||30000).toLocaleString()} / ${recorded.dropped||0}`);
  const sources=[...new Set((recorded.rows||[]).map(row=>row.source))];
  robotText('capture-source',sources.length?sources.map(s=>({simulation:'仿真',http_spi:'台架',lora:'LoRa'})[s]||s).join(' / '):'—');
  robotText('capture-detail',recorded.error||'后台每秒一条；最多30,000条内存记录，关闭服务前请导出CSV/JSON。');
  robotText('capture-overview-state',`${recorded.recording?'正在记录':'未记录'} · ${sourceLabel} · ${recorded.count.toLocaleString()}条`);
  robotText('capture-overview-detail',stationMode==='mission'?'LoRa遥测上行尚未接入，当前不能记录实物任务数据。可选择台架遥测或离线仿真。':'每秒一条：电压、油门、六路输出、延迟、控制权和安全状态。不含视频；内存保存，需导出CSV/JSON。');
  $('capture-overview-state').dataset.state=recorded.recording?'online':'unknown';
  $('capture-toggle').disabled=stationMode==='mission';$('capture-toggle').textContent=recorded.recording?'停止遥测记录':stationMode==='mission'?'遥测未接入':`记录${sourceLabel}`;$('capture-clear').disabled=recorded.recording||!recorded.count;
  for(const id of ['data-export-csv','data-export-json','chart-export'])$(id).disabled=!recorded.count;
  if(state.page==='data')renderDataTable();
}
function renderDataTable(){
  const body=$('data-rows');body.replaceChildren();
  if(!recorded.rows.length){const row=body.insertRow();const cell=row.insertCell();cell.colSpan=7;cell.textContent='尚无数据';return;}
  for(const row of recorded.rows.slice(-12).reverse()){
    const tr=body.insertRow();
    for(const value of [row.time.replace('T',' ').replace('+00:00','Z'),row.source,row.valid?'是':'否',robotNumber(row.battery_v,'',3),robotNumber(row.left_esc_pct),robotNumber(row.right_esc_pct),row.safety])tr.insertCell().textContent=value;
  }
}
async function pollData(){
  if(dataPending||location.protocol==='file:')return;dataPending=true;
  try{recorded=await service('data');if(recorded.error&&recorded.error!==lastCaptureError)log('数据记录：'+recorded.error,'warn');lastCaptureError=recorded.error||'';renderCapture();if(state.page==='data')drawChart();}
  catch(e){robotText('capture-detail',e.message);}
  finally{dataPending=false;}
}
async function captureAction(action){
  try{recorded=await service('data/record',{action});if(action==='clear'){chartFrozen=null;$('chart-freeze').textContent='冻结图表';}renderCapture();drawChart();log(({start:'开始后台数据记录',stop:'停止数据记录',clear:'已清空本次内存数据'})[action],'info');}
  catch(e){toast(e.message);log(e.message,'warn');}
}
function drawChart(){
  const canvas=$('data-chart'),box=canvas.getBoundingClientRect();if(box.width<20)return;
  const ratio=window.devicePixelRatio||1,width=box.width,height=box.height;
  canvas.width=width*ratio;canvas.height=height*ratio;
  const context=canvas.getContext('2d');context.scale(ratio,ratio);
  const css=getComputedStyle(document.body),ink=css.getPropertyValue('--muted'),line=css.getPropertyValue('--line');
  context.fillStyle=css.getPropertyValue('--card');context.fillRect(0,0,width,height);
  const group=$('chart-group').value,series=PLOTS[group],rows=(chartFrozen||recorded.rows||[]).slice(-Number($('chart-range').value));
  $('chart-legend').replaceChildren(...series.map(([,label,color])=>{const span=document.createElement('span'),dot=document.createElement('i');dot.style.background=color;span.append(dot,document.createTextNode(label));return span;}));
  $('chart-empty').hidden=rows.some(row=>series.some(([key])=>Number.isFinite(row[key])));
  const values=rows.flatMap(row=>series.map(([key])=>row[key])).filter(Number.isFinite);
  let low=values.length?Math.min(...values):0,high=values.length?Math.max(...values):1;
  const padding=high===low?Math.max(1,Math.abs(low)*.05):(high-low)*.15;low-=padding;high+=padding;
  const plot={x:55,y:20,w:width-77,h:height-57},x=i=>plot.x+(rows.length>1?i/(rows.length-1):.5)*plot.w,y=v=>plot.y+(high-v)/(high-low)*plot.h;
  context.font='10px "Cascadia Code",Consolas,monospace';context.fillStyle=ink;context.textAlign='right';
  for(let i=0;i<=4;i++){const yy=plot.y+plot.h*i/4;context.strokeStyle=line;context.lineWidth=1;context.beginPath();context.moveTo(plot.x,yy);context.lineTo(width-22,yy);context.stroke();context.fillText((high-(high-low)*i/4).toFixed(group==='battery'?2:1),plot.x-10,yy+3);}
  context.textAlign='left';context.fillText(UNITS[group],plot.x,10);
  if(rows.length){context.fillText(rows[0].time.slice(11,19)+' UTC',plot.x,height-10);context.textAlign='right';context.fillText(rows.at(-1).time.slice(11,19)+' UTC',width-22,height-10);}
  for(const [key,,color] of series){
    context.strokeStyle=color;context.fillStyle=color;context.lineWidth=2;context.beginPath();let active=false;
    rows.forEach((row,i)=>{if(!Number.isFinite(row[key])){active=false;return;}const xx=x(i),yy=y(row[key]);if(active)context.lineTo(xx,yy);else context.moveTo(xx,yy);active=true;});context.stroke();
    rows.forEach((row,i)=>{if(Number.isFinite(row[key])&&(rows.length<15||i===chartHover)){context.beginPath();context.arc(x(i),y(row[key]),2.7,0,Math.PI*2);context.fill();}});
  }
  if(chartHover>=0&&chartHover<rows.length){context.setLineDash([3,4]);context.strokeStyle=ink;context.beginPath();context.moveTo(x(chartHover),plot.y);context.lineTo(x(chartHover),plot.y+plot.h);context.stroke();context.setLineDash([]);const row=rows[chartHover];robotText('chart-note',row.time+' · '+series.map(([key,label])=>label+' '+robotNumber(row[key],UNITS[group],2)).join(' / '));}
  else robotText('chart-note',chartFrozen?'图表已冻结，后台仍持续记录。移动指针查看样本。':'移动指针查看样本；无效/离线样本留空，曲线不跨缺失数据连接。');
}
$('capture-toggle').onclick=()=>captureAction(recorded.recording?'stop':'start');
$('capture-clear').onclick=()=>{if(confirm('清空本次内存记录？需要保留时请先导出CSV或JSON。'))captureAction('clear');};
$('chart-group').onchange=$('chart-range').onchange=()=>{chartHover=-1;drawChart();};
$('chart-freeze').onclick=()=>{chartFrozen=chartFrozen?null:recorded.rows.slice();$('chart-freeze').textContent=chartFrozen?'恢复实时':'冻结图表';drawChart();};
$('data-chart').onpointermove=event=>{const box=$('data-chart').getBoundingClientRect(),rows=(chartFrozen||recorded.rows||[]).slice(-Number($('chart-range').value));chartHover=Math.max(0,Math.min(rows.length-1,Math.round((event.clientX-box.left-55)/(box.width-77)*(rows.length-1))));drawChart();};
$('data-chart').onpointerleave=()=>{chartHover=-1;drawChart();};
$('chart-export').onclick=()=>{drawChart();$('data-chart').toBlob(blob=>{if(blob)downloadBlob(blob,'robofish-'+$('chart-group').value+'.png');});};
for(const format of ['csv','json'])$('data-export-'+format).onclick=()=>{const link=document.createElement('a');link.href='/api/data/export?format='+format;link.download='robofish-data.'+format;link.click();log('导出数据 '+format.toUpperCase()+'，'+recorded.count+'条样本');};
new ResizeObserver(()=>{if(state.page==='data')drawChart();}).observe($('data-chart').parentElement);
window.addEventListener('stationtheme',drawChart);
window.addEventListener('stationpage',event=>{if(event.detail==='data'){renderCapture();requestAnimationFrame(drawChart);}if(event.detail==='journal')renderJournal();});
function renderJournal(){
  const search=$('journal-search').value.toLowerCase(),level=$('journal-level').value;
  const rows=state.logs.filter(row=>(level==='all'||row.level===level)&&row.message.toLowerCase().includes(search));
  $('journal-rows').replaceChildren(...rows.slice(-500).reverse().map(row=>logRow(row,true)));
  $('journal-count').textContent=`${rows.length}条`+(rows.length>500?' · 显示最近500条':'');
}
$('journal-level').onchange=$('journal-search').oninput=renderJournal;
$('journal-export-csv').onclick=()=>exportLogs('csv');

renderJournal();renderCapture();pollData();setInterval(pollData,1000);
