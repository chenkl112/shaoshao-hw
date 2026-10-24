// Exercise the actual page handlers without a browser or physical vehicle.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(require('path').join(__dirname,'../navcar_move.html'),'utf8');
class Element{
  constructor(action){this.dataset={action};this.events={};this.value='0.10';this.classList={toggle(){}}}
  addEventListener(k,f){this.events[k]=f}setPointerCapture(){}setAttribute(){}
  checkValidity(){return this.value!==''&&Number.isFinite(Number(this.value))&&(this.min===undefined||Number(this.value)>=Number(this.min))&&(this.max===undefined||Number(this.value)<=Number(this.max))}
  reportValidity(){}
}
const directions=['forward','backward','left','right'].map(x=>new Element(x));
const elements=Object.fromEntries(['strength','limit','error','status','stop','feedback','yawAngle','yawMinus','yawApply','yawPlus','yawZero','spinRate','spinToggle','spinApply'].map(x=>[x,new Element()]));
Object.assign(elements.yawAngle,{min:-180,max:180});Object.assign(elements.spinRate,{min:-6,max:6});
const document={hidden:false,events:{},getElementById:id=>elements[id],querySelectorAll:()=>directions,
  addEventListener(k,f){this.events[k]=f}};
const window={events:{},addEventListener(k,f){this.events[k]=f}};
const calls=[],yawCalls=[],spinCalls=[],allCalls=[],intervals=[];
let fakeYaw=17,fakeSpin=false,fakeRate=.1,fakeOwner=null;
const context={document,window,location:{search:'?token=secret'},crypto:{randomUUID:()=> 'test-client'},
  URLSearchParams,AbortController,Date,Math,Number,console,setTimeout,clearTimeout,
  setInterval(f){intervals.push(f)},
  async fetch(path,args){
    if(args.body){
      const packet=JSON.parse(args.body);allCalls.push(packet);fakeOwner=packet.client;
      if(path==='/api/command'){calls.push(packet);if('spin_enabled' in packet)fakeSpin=packet.spin_enabled;if('spin_rate' in packet)fakeRate=packet.spin_rate}
      if(path==='/api/yaw'){yawCalls.push(packet);fakeYaw=packet.angle_deg;fakeSpin=false}
      if(path==='/api/spin'){spinCalls.push(packet);fakeSpin=packet.enabled;fakeRate=packet.rate}
    }
    return {ok:true,async json(){return {connected:true,error:'',max_strength:.3,control_started:false,yaw_target_deg:fakeYaw,feedback_targets:null,feedback_measurements:{velocity_valid:true,xy_valid:true,forward_mps:.05,left_mps:-.02,omega_radps:.3,yaw_actual_deg:12},navigation_feedback_reason:'等待 0x82',active:'stop',control_client:fakeOwner,spin:{enabled:fakeSpin,rate:fakeRate,max_rate:6}}}};
  }};
vm.createContext(context);vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const event={preventDefault(){},pointerId:1};
(async()=>{
  await flush();assert.equal(calls.length,0,'loading must not send control');
  assert.equal(allCalls.length,0,'loading must not change yaw or enable spin');
  assert(elements.feedback.textContent.includes('XY'));
  assert(elements.feedback.textContent.includes('前向 0.050 m/s'));
  assert(!elements.feedback.textContent.includes('旋转'));
  assert(!elements.feedback.textContent.includes('Yaw'));
  assert.equal(Number(elements.yawAngle.value),17,'initialize from target, never force zero');
  window.events.blur();await flush();assert.equal(calls.length,0,'idle blur must not take control');
  directions[0].events.pointerdown(event);await flush();assert.equal(calls.at(-1).action,'forward');
  directions[0].events.pointerup(event);await flush();assert.equal(calls.at(-1).action,'stop');
  directions[2].events.pointerdown(event);await flush();window.events.blur();await flush();
  assert.equal(calls.at(-1).action,'stop');
  document.events.keydown({key:'ArrowRight',target:{tagName:'BODY'},repeat:false,preventDefault(){}});
  await flush();assert.equal(calls.at(-1).action,'right');
  document.events.keyup({key:'ArrowRight',preventDefault(){}});await flush();
  assert.equal(calls.at(-1).action,'stop');
  directions[1].events.pointerdown(event);await flush();document.hidden=true;
  document.events.visibilitychange();await flush();assert.equal(calls.at(-1).action,'stop');
  const before=calls.length;intervals[0]();await flush();assert.equal(calls.length,before,'hidden page cannot renew movement');
  assert(calls.every((x,i)=>i===0||x.seq>calls[i-1].seq));
  assert(calls.every(x=>x.speed_unit==='m/s'),'speed commands must declare the calibrated unit');
  document.hidden=false;
  elements.yawAngle.value='30';elements.yawAngle.events.input();elements.yawApply.events.click();await flush();
  assert.equal(yawCalls.at(-1).angle_deg,30);
  elements.yawPlus.events.click();await flush();assert.equal(yawCalls.at(-1).angle_deg,35);
  elements.yawMinus.events.click();await flush();assert.equal(yawCalls.at(-1).angle_deg,30);
  elements.yawZero.events.click();await flush();assert.equal(yawCalls.at(-1).angle_deg,0);
  elements.yawAngle.value='179';elements.yawPlus.events.click();await flush();assert.equal(yawCalls.at(-1).angle_deg,-176);
  const yawCount=yawCalls.length;elements.yawAngle.value='999';elements.yawApply.events.click();await flush();assert.equal(yawCalls.length,yawCount);
  elements.spinToggle.events.click();await flush();assert.equal(spinCalls.at(-1).enabled,true);
  const stationaryCalls=calls.length;intervals[0]();await flush();assert.equal(calls.length,stationaryCalls,'backend latches spin; stationary page sends no keepalive');
  directions[0].events.pointerdown(event);await flush();assert.equal(fakeSpin,true);
  directions[0].events.pointerup(event);await flush();assert.equal(calls.at(-1).action,'stop');assert.equal(fakeSpin,true,'release stops translation and retains backend spin');
  elements.spinRate.value='6';elements.spinApply.events.click();await flush();assert.equal(spinCalls.at(-1).rate,6);
  elements.spinRate.value='-6';elements.spinApply.events.click();await flush();assert.equal(spinCalls.at(-1).rate,-6);
  const spinCount=spinCalls.length;elements.spinRate.value='6.01';elements.spinApply.events.click();await flush();assert.equal(spinCalls.length,spinCount);
  elements.spinRate.value='.1';
  window.events.blur();await flush();assert.equal(fakeSpin,true,'changing pages preserves backend spin');
  elements.spinToggle.events.click();await flush();assert.equal(spinCalls.at(-1).enabled,false);
  elements.spinToggle.events.click();await flush();assert.equal(spinCalls.at(-1).enabled,true);
  elements.stop.events.click();await flush();assert.equal(calls.at(-1).spin_enabled,false);
  assert(allCalls.every((x,i)=>i===0||x.seq>allCalls[i-1].seq),'all control interfaces share sequence ordering');
  console.log('Movement UI: directions, yaw targets, measured feedback, spin toggle/rate/latch, stop and sequence checks passed');
})().catch(e=>{console.error(e);process.exitCode=1});
