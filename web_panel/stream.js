'use strict';
// State and cloud transport only; control requests remain authenticated HTTP.
window.ShaoShaoStream = class {
 constructor(token, onState, onCloud, onUnavailable) {
  this.token=token;this.onState=onState;this.onCloud=onCloud;this.onUnavailable=onUnavailable;
  this.lastStateAt=0;this.retry=500;this.closed=false;this.pending=null;this.connect();
 }
 get active(){return !!this.socket&&this.socket.readyState===WebSocket.OPEN&&Date.now()-this.lastStateAt<1500}
 connect(){
  if(this.closed)return;
  const protocol=location.protocol==='https:'?'wss:':'ws:';
  const socket=new WebSocket(protocol+'//'+location.host+'/ws');this.socket=socket;socket.binaryType='arraybuffer';
  socket.onopen=()=>socket.send(JSON.stringify({type:'auth',token:this.token}));
  socket.onmessage=event=>{
   try{
    if(typeof event.data==='string'){
     const packet=JSON.parse(event.data);
     if(packet.type==='ready'){this.retry=500;return}
     if(packet.type==='unavailable'){this.lastStateAt=0;this.onUnavailable(packet.error);this.ack(packet.serial);return}
     if(packet.type!=='state'||!Number.isSafeInteger(packet.serial))throw Error('无效状态消息');
     this.onState(packet.body);this.lastStateAt=Date.now();
     if(packet.cloud)this.pending={serial:packet.serial,sequence:packet.cloud_sequence};else this.ack(packet.serial);
    }else{
     const view=new DataView(event.data);
     if(view.byteLength<8||view.getUint32(0,true)!==0x43504353)throw Error('无效点云报文');
     const length=view.getUint32(4,true);
     if(length>4096||8+length>view.byteLength)throw Error('无效点云头');
     const header=JSON.parse(new TextDecoder().decode(new Uint8Array(event.data,8,length)));
     if(!Number.isSafeInteger(header.count)||header.count<0||header.count>6000||view.byteLength!==8+length+header.count*12||!this.pending||header.sequence!==this.pending.sequence)throw Error('无效点云长度或序号');
     const points=[];let offset=8+length;
     for(let i=0;i<header.count;i++){
      const point=[view.getFloat32(offset,true),view.getFloat32(offset+4,true),view.getFloat32(offset+8,true)];offset+=12;
      if(!point.every(Number.isFinite))throw Error('无效点云坐标');points.push(point);
     }
     this.onCloud({...header,points});this.ack(this.pending.serial);this.pending=null;
    }
   }catch(error){this.onUnavailable(error.message);socket.close()}
  };
  socket.onclose=()=>{this.lastStateAt=0;this.pending=null;if(!this.closed){setTimeout(()=>this.connect(),this.retry);this.retry=Math.min(10000,this.retry*2)}};
  socket.onerror=()=>socket.close();
 }
 ack(serial){if(this.socket?.readyState===WebSocket.OPEN)this.socket.send(JSON.stringify({type:'ack',serial}))}
 close(){this.closed=true;this.socket?.close()}
};
