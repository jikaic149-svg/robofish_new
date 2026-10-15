// Exercise the shipped extension tasks through a real loopback mock of EmberProbe.
// No native driver, debugger, OpenOCD or hardware process is started.
'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),os=require('node:os'),path=require('node:path'),http=require('node:http'),Module=require('node:module');
const root=fs.mkdtempSync(path.join(os.tmpdir(),'robofish-ember-test-'));
assert(path.resolve(root).startsWith(path.resolve(os.tmpdir())+path.sep));
const calls=[];let attach;
const fakeVscode={debug:{startDebugging:async(folder,config)=>{attach=config;return true;}},workspace:{workspaceFolders:[{}],getConfiguration:()=>({get:()=>path.join(root,'openocd/bin/openocd.exe')})}};
const originalLoad=Module._load;
Module._load=function(name,...args){return name==='vscode'?fakeVscode:originalLoad.call(this,name,...args);};
const {tasks}=require('./extension');Module._load=originalLoad;
const native=http.createServer(async(request,response)=>{
 let raw='';for await(const chunk of request)raw+=chunk;
 assert.equal(request.headers.authorization,'Bearer test-private-token');
 const input=JSON.parse(raw);calls.push(input);
 let result={};
 if(input.method==='debug.status')result={state:'none'};
 if(input.method==='debug.inspect')result={threads:[{id:7,name:'MotionControl'}]};
 if(input.method==='flash.authorize')result={authorized:false,confirmationId:'test-one-time-id'};
 const denied=input.method==='flash.execute';
 response.writeHead(200,{'Content-Type':'application/json'});
 response.end(JSON.stringify(denied?{ok:false,error:{code:'USER_DENIED',message:'Native approval denied'}}:{ok:true,result}));
});
(async()=>{
 try{
  await new Promise(resolve=>native.listen(0,'127.0.0.1',resolve));
  const pointer=path.join(root,'.agents/skills/_emberprobe');fs.mkdirSync(pointer,{recursive:true});
  fs.writeFileSync(path.join(pointer,'agent-bridge.json'),JSON.stringify({host:'127.0.0.1',port:native.address().port,workspace:root,token:'test-private-token'}));
  const build=path.join(root,'stm32f405/build/debug');fs.mkdirSync(build,{recursive:true});
  fs.writeFileSync(path.join(build,'RoboFishF405.elf'),'symbol fixture');fs.writeFileSync(path.join(build,'RoboFishFactory.elf'),'factory fixture');
  const debug=path.join(root,'tools/debug');fs.mkdirSync(debug,{recursive:true});
  for(const name of ['robofish-dap-usb.cfg','robofish-dap-wifi.cfg','robofish-f405.cfg','robofish-f405-flash.cfg'])fs.writeFileSync(path.join(debug,name),'fixture');
  fs.mkdirSync(path.join(root,'openocd/share/openocd/scripts/target'),{recursive:true});
  await tasks.connect(root,{transport:'usb',speed:100});
  assert.equal(attach.request,'attach');assert.equal(attach.type,'emberprobe');assert.equal(attach.executable,path.join(build,'RoboFishF405.elf'));
  const configured=calls.find(item=>item.method==='config.set').params.values;
  assert.equal(configured.debugger,'robofish/robofish-dap-usb.cfg');assert.equal(configured.mcu,'robofish/robofish-f405.cfg');
  assert(fs.existsSync(path.join(root,'openocd/share/openocd/scripts/target',configured.mcu)));
  await tasks.pause(root);assert.deepEqual(calls.at(-1),{method:'debug.control',params:{action:'pause'}});
  assert.equal((await tasks.threads(root)).threads[0].id,7);
  await tasks.stack(root,{threadId:7,start:20,count:50});
  assert.deepEqual(calls.at(-1).params,{action:'stack',threadId:7,start:20,count:50});
  await tasks.locals(root,{reference:'current-handle',start:50,count:50});
  assert.deepEqual(calls.at(-1).params,{action:'variables',reference:'current-handle',start:50,count:50});
  const plan=await tasks['flash-plan'](root,{transport:'usb'});assert.equal(plan.confirmationId,'test-one-time-id');
  assert.equal(calls.at(-1).params.elf,path.join(build,'RoboFishFactory.elf'));
  assert.equal(calls.at(-1).params.probe,'robofish/robofish-dap-usb.cfg');assert.equal(calls.at(-1).params.target,'robofish/robofish-f405-flash.cfg');
  assert.equal(calls.at(-1).params.elfSha256.length,64);
  await assert.rejects(tasks.flash(root,{transport:'usb',confirmationId:plan.confirmationId}),error=>error.code==='USER_DENIED');
  assert.equal(calls.filter(item=>item.method==='flash.execute').length,1);
  await assert.rejects(tasks.flash(root,{transport:'wifi'}),/烧录计划/);
  console.log('PASS: attach without download; native pause/inspection; exact factory plan; native denial propagated without fallback.');
 }finally{
  native.close();
  assert(path.resolve(root).startsWith(path.resolve(os.tmpdir())+path.sep));
  fs.rmSync(root,{recursive:true,force:true});
 }
})().catch(error=>{console.error(error);process.exitCode=1;});
