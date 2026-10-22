const fs=require('fs'),vm=require('vm'),assert=require('assert');
const sockets=[],states=[],clouds=[],errors=[];
class Socket {
 static OPEN=1;
 constructor(url){this.url=url;this.readyState=1;this.sent=[];sockets.push(this)}
 send(value){this.sent.push(JSON.parse(value))}
 close(){this.closed=true;this.readyState=3}
}
const context={window:{},WebSocket:Socket,location:{protocol:'http:',host:'robot:8080'},Date,JSON,Math,Number,DataView,Uint8Array,TextDecoder,
 setTimeout(){return 1}};
vm.createContext(context);vm.runInContext(fs.readFileSync(require('path').join(__dirname,'../stream.js'),'utf8'),context);
const stream=new context.window.ShaoShaoStream('token',value=>states.push(value),value=>clouds.push(value),value=>errors.push(value));
const socket=sockets[0];socket.onopen();assert.deepEqual(socket.sent,[{type:'auth',token:'token'}]);
assert(!socket.url.includes('token'),'credentials must not be added to the WebSocket URL');
socket.onmessage({data:JSON.stringify({type:'ready'})});
socket.onmessage({data:JSON.stringify({type:'state',serial:1,cloud:true,cloud_sequence:4,body:{mode:'idle'}})});
assert.equal(socket.sent.length,1,'acknowledge only after the matching cloud is decoded');
const metadata=Buffer.from(JSON.stringify({sequence:4,count:1,frame:'map'}));
const bytes=Buffer.alloc(8+metadata.length+12);bytes.write('SCPC');bytes.writeUInt32LE(metadata.length,4);metadata.copy(bytes,8);
bytes.writeFloatLE(1,8+metadata.length);bytes.writeFloatLE(2,12+metadata.length);bytes.writeFloatLE(3,16+metadata.length);
socket.onmessage({data:bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.length)});
assert.equal(clouds.length,1);assert.deepEqual(Array.from(clouds[0].points[0]),[1,2,3]);
assert.deepEqual(socket.sent[1],{type:'ack',serial:1});assert(stream.active);
socket.onmessage({data:JSON.stringify({type:'unavailable',serial:2,error:'offline'})});
assert.equal(errors.at(-1),'offline');assert(!stream.active);
socket.onmessage({data:new ArrayBuffer(2)});assert(socket.closed,'malformed binary frames close the stream');
console.log('WebSocket UI: auth, binary decoding, ACK ordering, loss and malformed frames passed');
