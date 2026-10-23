(async function(){
 const ws=Blockly.getMainWorkspace();
 ws.clear(); Blockly.Xml.domToWorkspace(Blockly.Xml.textToDom(yyProjectXml),ws);
 await new Promise(r=>setTimeout(r,700));
 const first=Blockly.Arduino.workspaceToCode(ws);
 const saved=Blockly.Xml.domToPrettyText(Blockly.Xml.workspaceToDom(ws));
 ws.clear();Blockly.Xml.domToWorkspace(Blockly.Xml.textToDom(saved),ws);
 await new Promise(r=>setTimeout(r,700));
 const code=Blockly.Arduino.workspaceToCode(ws);
 if(first.replace(/\s/g,'')!==code.replace(/\s/g,''))throw Error('Unstable save/reopen');
 if(!code.includes('Serial1.begin(115200);'))throw Error('Missing baud');
 if(code.includes('Serial1.begin();'))throw Error('Empty baud');
 ace.edit('editor').setValue(code,-1);
 const blocks=ws.getAllBlocks();
 return {code,xml:saved+'\n<!-- edittype="asr_pro" -->\n<!-- officialLib="[]" -->\n<!-- personalLib="[]" -->\n',
 report:{blocks:blocks.length,saveReopenStable:true,baud:115200,
 recognition:blocks.filter(b=>b.type==='asr_id').map(b=>({phrase:b.getFieldValue('ASR'),type:b.getFieldValue('leibie'),id:b.getFieldValue('id'),reply:b.getFieldValue('huifu')})),
 audio:blocks.filter(b=>b.type==='asr_add_id').map(b=>({id:b.getFieldValue('id'),text:b.getFieldValue('NAME')}))}};
})()
