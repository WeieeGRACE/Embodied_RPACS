(async function(){
 window.yyBuildEvents=[];
 asrpro_run_flash.property_build_stus_changed.connect(function(ok,message){yyBuildEvents.push({kind:'build',ok,message});});
 asrpro_run_flash.property_flash_stus_changed.connect(function(ok,message){yyBuildEvents.push({kind:'flash',ok,message});});
 const code=Blockly.Arduino.workspaceToCode(Blockly.getMainWorkspace());
 if(!code.includes('Serial1.begin(115200);'))throw Error('baud missing');
 main_file_path=rp+'/asrpro/_asr.cpp';
 file_ops.writeFile(main_file_path,code,'rebuild');
 await new Promise(r=>setTimeout(r,1000));
 return {events:yyBuildEvents};
})()
