const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(require('path').join(__dirname,'../index.html'),'utf8');
const calls=[],elements={cancelGoal:{},goGoal:{},retryLocalization:{},error:{}};
let finishGoal;
const context={mapCanvas:{setPointerCapture(){},addEventListener(){}},selectedGoal:null,mapDrag:null,
 mapImage:{width:100,height:100},mapInfo:{resolution:.025,origin:[0,0,0]},mapRevision:'m1',mapZoom:1,mapX:0,mapY:0,
 planRequest:0,planError:'',lastState:{navigation:{status:'idle'}},connected:true,goalRequest:false,cancelAfterGoal:false,
 goalSubmitting:false,navigationRequests:0,requestError:'',navigationControls(){},drawMap(){},previewGoal(){},$:id=>elements[id],
 async api(path){calls.push(path);if(path==='/api/navigation/goal')await new Promise(r=>finishGoal=r);
   return {navigation:{status:path.endsWith('goal')?'active':'canceled'}}}};
context.mapCanvas.getBoundingClientRect=()=>({left:0,top:0});
vm.createContext(context);
function load(from,to){vm.runInContext(html.slice(html.indexOf(from),html.indexOf(to,html.indexOf(from))),context)}
load('function selectGoal(e){','async function previewGoal()');
load('async function navigationRequest(path,body){',"$('retryLocalization').onclick");
load('mapCanvas.onpointerdown=',"mapCanvas.onpointercancel=");
const left={button:0,clientX:10,clientY:20,pointerId:1};
context.mapCanvas.onpointerdown(left);context.mapCanvas.onpointerup(left);
assert.equal(context.selectedGoal.map_revision,'m1');assert.equal(context.selectedGoal.x,.2625);
assert.equal(calls.length,0,'selecting a goal must not move the car');
const selected=context.selectedGoal;
context.mapCanvas.onpointerdown({...left,button:2});context.mapCanvas.onpointerup({...left,button:2});
assert.strictEqual(context.selectedGoal,selected,'right release cannot select');
let prevented=false;
context.mapCanvas.oncontextmenu({preventDefault(){prevented=true}});
assert(prevented);assert.equal(context.selectedGoal,null);assert.equal(context.planRequest,1);
assert.equal(calls.length,0,'canceling a preview needs no motor/navigation command');
context.mapCanvas.onpointerdown(left);context.mapCanvas.onpointermove({...left,clientX:30});
context.mapCanvas.onpointerup({...left,clientX:30});assert.equal(context.selectedGoal,null,'drag is not a selection');
const flush=()=>new Promise(r=>setImmediate(r));
(async()=>{
 context.lastState={navigation:{status:'active'}};
 context.mapCanvas.oncontextmenu({preventDefault(){}});await flush();
 assert.equal(calls.at(-1),'/api/navigation/cancel');
 context.lastState={navigation:{status:'idle'}};
 const request=context.navigationRequest('/api/navigation/goal',{x:1,y:1});
 context.mapCanvas.oncontextmenu({preventDefault(){}});await flush();
 assert.equal(context.goalRequest,true,'cancel response cannot unlock a still pending goal');
 const cancellations=calls.filter(x=>x.endsWith('cancel')).length;
 finishGoal();await request;await flush();
 assert.equal(calls.filter(x=>x.endsWith('cancel')).length,cancellations+1,
   'right-click during goal submission must cancel again after the goal response');
 assert.equal(context.selectedGoal,null);
 context.goalRequest=true;context.goalSubmitting=false;context.cancelAfterGoal=false;
 context.lastState={navigation:{status:'idle'}};context.cancelMapGoal();
 assert.equal(context.cancelAfterGoal,false,'a localization request cannot cancel a future navigation goal');
 console.log('Map goals: left selection, right cancel, drag, active cancel, and submission race passed');
})().catch(e=>{console.error(e);process.exitCode=1});
