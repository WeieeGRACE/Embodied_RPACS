(async function(){
 window.yyModelEvents=[];
 for(const k of ['download_finished','unzip_finished','error_occurred'])if(asrpro_model_load_manager[k])asrpro_model_load_manager[k].connect(function(v){yyModelEvents.push({kind:k,text:String(v).slice(0,800),time:Date.now()});});
 jQuery('#command-create-modal').triggerHandler('click');
 await new Promise(r=>setTimeout(r,2500));
 return {active:on_creat_model,loggedIn:!!sessionStorage.getItem('userid'),ui:yyUiLog,events:yyModelEvents,dialogs:document.body.innerText.slice(-800)};
})()
