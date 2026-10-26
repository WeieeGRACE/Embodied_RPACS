"""Add reason-bearing playback completion only to the isolated project SDK."""
from pathlib import Path

def patch_sdk(sdk):
 p=Path(sdk)/'components/cmd_info/prompt_player.c'
 s=p.read_text(encoding='utf-8')
 if 'yy_prompt_play' in s:return
 s=s.replace('play_done_callback_t play_done_callback;    //!', 'void (*done_ex)(uint32_t,int32_t);\n    uint32_t cookie;\n    play_done_callback_t play_done_callback;    //!',1)
 # Error paths and queue purge must never look like successful playback.
 s=s.replace('if (voice_play_info.play_done_callback)', 'if(voice_play_info.done_ex)voice_play_info.done_ex(voice_play_info.cookie,-99);\n        if (voice_play_info.play_done_callback)')
 a=s.index('static void combination_callback(int32_t play_cb_state)\n{')
 b=s.index('static uint32_t voice_play_info_add_to_queue',a)
 part=s[a:b].replace('voice_play_info.done_ex(voice_play_info.cookie,-99)', 'voice_play_info.done_ex(voice_play_info.cookie,play_cb_state)')
 s=s[:a]+part+s[b:]
 sig='uint32_t prompt_play_by_cmd_handle(\n            cmd_handle_t cmd_handle, \n            int select_index, \n            play_done_callback_t play_done_callback,\n            bool preemptive)'
 assert sig in s
 s=s.replace(sig, 'static uint32_t yy_play_inner_ex(cmd_handle_t cmd_handle,int select_index,play_done_callback_t play_done_callback,bool preemptive,void (*done_ex)(uint32_t,int32_t),uint32_t cookie)')
 s=s.replace('    voice_play_info.cmd_handle = cmd_handle;', '    voice_play_info.done_ex=done_ex;voice_play_info.cookie=cookie;\n    voice_play_info.cmd_handle = cmd_handle;')
 # Invalid handles, allocation/queue failures, and disabled player do not authorize motion.
 s=s.replace('    #if !AUDIO_PLAYER_ENABLE\n', '    #if !AUDIO_PLAYER_ENABLE\n        if(done_ex)done_ex(cookie,-99);\n',1)
 s=s.replace('        if (play_done_callback)\n        {\n            play_done_callback(cmd_handle);', '        if(done_ex)done_ex(cookie,-99);\n        if (play_done_callback)\n        {\n            play_done_callback(cmd_handle);')
 wrapper='''
uint32_t prompt_play_by_cmd_handle(cmd_handle_t h,int index,play_done_callback_t cb,bool preempt){
 return yy_play_inner_ex(h,index,cb,preempt,NULL,0);
}
uint32_t yy_prompt_play(uint16_t id,void (*cb)(uint32_t,int32_t),uint32_t cookie){
 return yy_play_inner_ex(cmd_info_find_command_by_id(id),-1,NULL,true,cb,cookie);
}
'''
 s=s.replace('uint32_t prompt_play_by_voice_id(',wrapper+'\nuint32_t prompt_play_by_voice_id(',1)
 p.write_text(s,encoding='utf-8')
 print('Patched isolated SDK completion reason and request cookie')
