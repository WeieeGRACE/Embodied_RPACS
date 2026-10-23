(async () => {
  const fs = require('fs'), path = require('path');
  const root = window.yyProjectRoot;
  const records = path.join(root,'records'); if(!fs.existsSync(records)) fs.mkdirSync(records);
  const ws = Blockly.getMainWorkspace();
  const old = Blockly.Xml.domToText(Blockly.Xml.workspaceToDom(ws));
  fs.writeFileSync(path.join(require('os').tmpdir(),'yaoyao_previous_tianwen_workspace.xml'),old,'utf8');
  const input = fs.readFileSync(path.join(root,'asrpro.hd'),'utf8');
  ws.clear(); Blockly.Xml.domToWorkspace(Blockly.Xml.textToDom(input),ws);
  await new Promise(resolve=>setTimeout(resolve,500)); // Blockly scope change events
  const first = Blockly.Arduino.workspaceToCode(ws);
  const saved = Blockly.Xml.domToPrettyText(Blockly.Xml.workspaceToDom(ws));
  const suffix = '\n<!-- edittype="asr_pro" -->\n<!-- officialLib="[]" -->\n<!-- personalLib="[]" -->\n';
  fs.writeFileSync(path.join(root,'asrpro.hd'),saved+suffix,'utf8');
  ws.clear(); Blockly.Xml.domToWorkspace(Blockly.Xml.textToDom(saved),ws);
  await new Promise(resolve=>setTimeout(resolve,500));
  const second = Blockly.Arduino.workspaceToCode(ws);
  const normalize = s=>s.replace(/\n\s*\n/g,'\n').trim();
  if(normalize(first) !== normalize(second)) throw new Error('Generated code changed on save/reopen');
  fs.writeFileSync(path.join(records,'generated.cpp'),second,'utf8');
  const blocks=ws.getAllBlocks();
  const report={blocks:blocks.length,recognition:blocks.filter(b=>b.type==='asr_id').map(b=>({
    phrase:b.getFieldValue('ASR'),type:b.getFieldValue('leibie'),id:b.getFieldValue('id'),reply:b.getFieldValue('huifu')})),
    audio:blocks.filter(b=>b.type==='asr_add_id').map(b=>({id:b.getFieldValue('id'),text:b.getFieldValue('NAME')})),
    codeBytes:second.length,saveReopenStable:true,disabled:blocks.filter(b=>b.disabled).length};
  fs.writeFileSync(path.join(records,'blockly_validation.json'),JSON.stringify(report,null,2),'utf8');
  return report;
})()
