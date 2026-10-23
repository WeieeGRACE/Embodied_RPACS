// Local Tianwen Block verification helper. No browser dependencies required.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const pages = await (await fetch('http://127.0.0.1:9333/json/list')).json();
const page = pages.find(p => p.type === 'page' && /asr_pro\.html|asrpro\/index\.html/.test(p.url));
if (!page) throw new Error('Open an ASRPRO project in Tianwen Block first.');
const ws = new WebSocket(page.webSocketDebuggerUrl);
await new Promise((resolve, reject) => { ws.onopen = resolve; ws.onerror = reject; });
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const expression = `window.yyProjectRoot=${JSON.stringify(root)};window.yyProjectXml=${JSON.stringify(fs.readFileSync(path.join(root,'asrpro.hd'),'utf8'))};\n` + fs.readFileSync(process.argv[2], 'utf8');
ws.send(JSON.stringify({ id: 1, method: 'Runtime.evaluate', params: {
  expression: `Promise.resolve().then(() => (0,eval)(${JSON.stringify(expression)}))`, returnByValue: true, awaitPromise: true,
}}));
const result = await new Promise((resolve, reject) => {
  const timeout = setTimeout(() => reject(new Error('CDP timed out')), 45000);
  ws.onmessage = e => { const msg = JSON.parse(e.data); if (msg.id === 1) {
    clearTimeout(timeout); resolve(msg);
  }};
});
ws.close();
if (result.result?.exceptionDetails) {
  console.error(JSON.stringify(result.result.exceptionDetails)); process.exitCode = 1;
} else {
  const value=result.result?.result?.value ?? result;
  if(process.argv[3]) fs.writeFileSync(process.argv[3],JSON.stringify(value,null,2));
  else console.log(JSON.stringify(value));
}
