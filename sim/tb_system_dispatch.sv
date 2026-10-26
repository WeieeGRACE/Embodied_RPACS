`timescale 1ns/1ps
module tb_system_dispatch;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,voice_enable=0,cv=0,ack_valid=0,menu_valid=0,stop_gate=0,stop_request=0;
 reg ready=1,fault=0,busy=0,cmd_ready=1,done=0,tof_ready=1,post_fresh=1;
 reg [7:0] command=0,ack=0,menu_command=0;reg [4:0] route_state=0;
 wire avoid_mode,auto_run,waiting,manual_active,manual_submit,stopping,route_cancel,route_press,refresh,show_avoid,show_root,aux_valid;
 wire [7:0] manual_action,response,aux_type,aux_code,mode_status;
 reg completed=0;integer checks=0,announces=0,obstacles=0,clears=0,exits=0,navigations=0;reg [7:0] token,old_token;
 system_dispatch #(.CLK_HZ(1000),.AUDIO_TIMEOUT_MS(20)) dut(.*);
 reg buf_cancel=0,buf_request=0,buf_ready=0,buf_busy=0;
 reg [230:0] proposal=231'd123;reg [132:0] context_in=133'd456;
 wire buffer_ready,buffer_submit,buffer_busy,buffer_pending;
 wire [230:0] saved_command;wire [132:0] saved_context;
 obstacle_command_buffer guard(.clk(clk),.rst_n(rst_n),.cancel(buf_cancel),.request(buf_request),
 .motion_ready(buf_ready),.motion_busy(buf_busy),.proposal(proposal),.context_in(context_in),
 .ready(buffer_ready),.submit(buffer_submit),.busy(buffer_busy),.pending(buffer_pending),
 .command(saved_command),.context_out(saved_context));
 always @(posedge clk)if(aux_valid)case(aux_type)
 4:begin token=aux_code;announces++;end
 7:case(aux_code)1:obstacles++;2:clears++;3:exits++;endcase
 endcase
 always @(posedge clk)if(show_avoid)navigations++;
 task cycles(input integer n);repeat(n)begin @(negedge clk);end endtask
 task check(input bit value,input string msg);begin checks++;if(!value)$fatal(1,"%s",msg);end endtask
 task voice(input [7:0] code);begin command=code;cv=1;cycles(1);cv=0;cycles(5);end endtask
 task confirm;begin ack=token;ack_valid=1;cycles(1);ack_valid=0;cycles(3);end endtask
 task start;begin voice(8'h16);check(waiting && avoid_mode && !auto_run,"must wait for audio");confirm();check(auto_run && !waiting,"matched completion starts");end endtask
 initial begin
 cycles(2);rst_n=1;cycles(2);
 voice(8'h16);check(!avoid_mode && announces==0 && navigations==1,"voice off entry navigates but does not start");
 menu_command=8'h16;menu_valid=1;cycles(1);menu_valid=0;cycles(2);check(!avoid_mode,"voice off menu entry rejected");
 voice_enable=1;voice(8'h16);check(waiting && !auto_run && announces==1,"entry waits");
 ack=token+1;ack_valid=1;cycles(1);ack_valid=0;cycles(2);check(!auto_run,"wrong token ignored");
 post_fresh=0;confirm();check(waiting && !auto_run,"old sensor shots cannot start");post_fresh=1;cycles(2);check(auto_run,"fresh samples allow start");
 busy=1;route_state=2;cycles(2);voice(8'h16);check(auto_run && announces==1,"duplicate entry no route restart");
 route_state=3;cycles(3);check(obstacles==1,"one obstacle event");cycles(3);check(obstacles==1,"no level duplicate");
 route_state=20;cycles(2);route_state=1;cycles(3);check(clears==1,"only complete return event");
 route_state=2;cycles(1);route_state=3;cycles(3);check(obstacles==2,"new obstacle speaks again");
 voice(8'h12);check(!auto_run && avoid_mode && manual_action==8'h12 && !manual_submit,"takeover waits for stillness");
 busy=0;wait(manual_submit);@(posedge clk);#1;busy=1;cycles(1);check(manual_active && avoid_mode,"manual accepted after stop");busy=1;
 voice(8'h13);check(manual_action==8'h12,"busy manual cannot be replaced");
 done=1;busy=0;cycles(1);done=0;cycles(3);check(!auto_run && avoid_mode && !manual_active,"manual completion stays paused");
 start();busy=1;voice(2);check(!auto_run && avoid_mode,"STOP pauses retains mode");busy=0;cycles(4);
 voice(8'h10);check(waiting,"forward means continuous resume");confirm();check(auto_run,"forward resume");
 busy=1;menu_command=8'h16;menu_valid=1;cycles(1);menu_valid=0;cycles(3);check(!auto_run && avoid_mode,"K2 toggles pause");busy=0;cycles(3);
 voice(8'h16);cycles(22);check(!waiting && !auto_run && avoid_mode,"missing audio timeout pauses");confirm();check(!auto_run,"late confirmation ignored");
 voice(8'h16);voice(2);confirm();check(!auto_run && avoid_mode,"stop invalidates request");
 voice(8'h16);fault=1;cycles(2);confirm();check(!auto_run,"fault invalidates audio");fault=0;cycles(2);
 // Exercise arbitration in every executable route phase, including completion
 // checks and both extra-distance phases. Cancelled routes never report success.
 for(integer stage=1;stage<=23;stage++)if(stage!=17 && stage!=18)begin
 route_state=0;busy=0;cycles(3);start();busy=1;route_state=stage;cycles(2);
 voice(8'h11+(stage%3));
 check(!auto_run && avoid_mode && !waiting && !manual_submit,"phase takeover must stop first");
 old_token=token;confirm();check(!auto_run,"old audio completion cannot revive cancelled route");
 busy=0;wait(manual_submit);@(posedge clk);#1;busy=1;cycles(2);
 check(manual_active,"phase takeover submits exactly after stillness");
 done=1;busy=0;cycles(1);done=0;cycles(2);
 check(!manual_active && !auto_run && avoid_mode,"phase takeover finishes paused");
 end
 for(integer blocked=17;blocked<=18;blocked++)begin
 route_state=0;start();route_state=blocked;cycles(3);
 check(!auto_run && avoid_mode,"blocked or failed route retains paused mode");
 route_state=0;cycles(3);
 end
 // Stale confirmation across a new request and soft reset must not match.
 voice(8'h16);old_token=token;voice(2);voice(8'h16);
 ack=old_token;ack_valid=1;cycles(1);ack_valid=0;cycles(2);
 check(waiting && !auto_run,"superseded audio request ignored");voice(2);
 old_token=token;rst_n=0;cycles(2);rst_n=1;cycles(2);voice(8'h16);
 ack=old_token;ack_valid=1;cycles(1);ack_valid=0;cycles(2);
 check(waiting && !auto_run && token!=old_token,"soft reset preserves token sequence");voice(2);
 start();busy=1;voice(8'h17);check(!avoid_mode && !auto_run && exits==0,"exit stops before exit event");busy=0;cycles(4);check(exits==1,"exit after stationary");
 start();stop_gate=1;cycles(2);stop_gate=0;cycles(5);check(!avoid_mode && exits==2,"K0 exits");
 start();voice_enable=0;cycles(3);check(!auto_run && avoid_mode,"disable voice stops");
 buf_request=1;cycles(1);buf_request=0;proposal=231'd789;context_in=133'd987;
 check(buffer_pending && saved_command==123 && saved_context==456,"whole command and context captured atomically");
 buf_cancel=1;buf_ready=1;#1;check(!buffer_submit,"cancel beats submission on same edge");
 cycles(1);buf_cancel=0;cycles(4);check(!buffer_pending && !buffer_submit,"cancelled command never reappears");
 buf_cancel=1;buf_request=1;cycles(1);buf_request=0;buf_cancel=0;cycles(2);
 check(!buffer_pending && !buffer_submit,"cancel beats new capture");
 buf_ready=0;buf_request=1;cycles(1);buf_request=0;
 check(buffer_pending && saved_command==789 && saved_context==987,"fresh route replaces cancelled transaction");
 buf_ready=1;#1;check(buffer_submit,"fresh command may submit");
 @(posedge clk);#1;buf_busy=1;cycles(2);check(!buffer_submit && !buffer_pending,"submit exactly once");
 buf_busy=0;cycles(2);check(buffer_ready && !buffer_submit,"completion does not replay old command");
 $display("PASS dispatch %0d checks",checks);completed=1;$finish;
 end
 initial begin #100000;$fatal(1,"watchdog");end
endmodule
