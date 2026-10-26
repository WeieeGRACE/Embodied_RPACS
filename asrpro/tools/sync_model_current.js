(async function(){
 window.yyImportEvents=[];
 folder_ops.copyModelChanged.connect(function(ok,message){yyImportEvents.push({ok,message});});
 modelCopyAction='open';
 folder_ops.copy_model_files(yyProjectRoot+'/records/build/firmware_obstacle_20261025/resources',rp+'/asrpro');
 await new Promise(r=>setTimeout(r,2500));
 return {events:yyImportEvents,logs:yyUiLog.slice(-3)};
})()
