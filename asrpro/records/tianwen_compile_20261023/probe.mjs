import fs from 'node:fs';
const pages = await (await fetch('http://127.0.0.1:9333/json/list')).json();
const page = pages.find(p => p.type === 'page' && p.url.includes('asr_pro.html'));
if (!page) throw Error('ASRPRO page not found');
const ws = new WebSocket(page.webSocketDebuggerUrl);
await new Promise((resolve,reject) => { ws.onopen=resolve; ws.onerror=reject; });
const expression=fs.readFileSync(process.argv[2],'utf8');
const timer=setTimeout(()=>{ws.close();process.exitCode=1;console.error('timeout')},45000);
ws.onmessage=e=>{const m=JSON.parse(e.data);if(m.id===1){clearTimeout(timer);console.log(JSON.stringify(m,null,2));if(m.result?.exceptionDetails)process.exitCode=1;ws.close();}};
ws.send(JSON.stringify({id:1,method:'Runtime.evaluate',params:{expression: `Promise.resolve().then(() => (0,eval)(${JSON.stringify(expression)}))`,returnByValue:true,awaitPromise:true}}));


