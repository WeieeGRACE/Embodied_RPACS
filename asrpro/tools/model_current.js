(async function(){
 window.yyUiLog=window.yyUiLog||[];
 if(!window.yyToastsWrapped){
  for(const k of ['showToast','showToastShow','showContentModal'])if(typeof window[k]==='function'){
   const old=window[k];window[k]=function(){yyUiLog.push({kind:k,text:String(arguments[0]).slice(0,1400),time:Date.now()});return old.apply(this,arguments);};
  }
  window.yyToastsWrapped=true;
 }
 env_Path.getLoginState(function(state){if(state)applyRemoteLoginState(state);});
 await new Promise(r=>setTimeout(r,800));
 window.yyGenerationStartedAt=Date.now();
 jQuery('#command-create-modal').triggerHandler('click');
 await new Promise(r=>setTimeout(r,1500));
 return {loggedIn:!!sessionStorage.getItem('userid'),logs:yyUiLog,
 dialogs:document.body.innerText.slice(-1800),envMethods:Object.keys(window).filter(k=>/asrpro|burn|model|folder|uncompress|file_dialogs/i.test(k))};
})()
