(async function(){
 asrpro_run_flash.property_asrpro_set_break_changed.connect(function(mode,message){yyBuildEvents.push({kind:'aec-setting',mode,message});});
 // The setting operation receives no port; no burner method is invoked here.
 asrpro_run_flash.asrpro_set_break_mode('break','');
 await new Promise(r=>setTimeout(r,1000));
 return {events:yyBuildEvents};
})()
