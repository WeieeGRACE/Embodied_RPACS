(async () => {
const fs=require('fs'),crypto=require('crypto');window.yyCompileRoot='D:/Gowin/Project/Contest/Embodied_RPACS/Embodied_RPACS/Embodied_RPACS/asrpro';
window.yySourceHash=crypto.createHash('sha256').update(fs.readFileSync(yyCompileRoot+'/asrpro.hd')).digest('hex');
Array.from(document.querySelectorAll('a')).find(x=>x.getAttribute('data-dir')===yyCompileRoot.replace(/\//g,'\\')+'\\asrpro.hd').click();
await new Promise(r=>setTimeout(r,800));
const code=Blockly.Arduino.workspaceToCode(Blockly.getMainWorkspace());fs.writeFileSync(yyCompileRoot+'/records/tianwen_compile_20261023/generated.cpp',code);
return {blocks:Blockly.getMainWorkspace().getAllBlocks().length,codeBytes:code.length,device:editType,sourceSHA256:yySourceHash,models:Array.from(document.querySelectorAll('.modal')).filter(x=>getComputedStyle(x).display!=='none').map(x=>({id:x.id,text:x.innerText.slice(0,200)}))};
})()
