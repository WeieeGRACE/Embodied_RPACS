`timescale 1ns/1ps
module tb_system_dispatch;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,voice_enable=0,cv=0,ack_valid=0,menu_valid=0,stop_gate=0,stop_request=0;
 reg ready=1,fault=0,busy=0,cmd_ready=1,done=0,tof_ready=1,post_fresh=1;
 reg [7:0] command=0,ack=0,menu_command=0;reg [4:0] route_state=0;
 wire voice_command_valid;
 wire avoid_mode,auto_run,waiting,manual_active,manual_submit,stopping,route_cancel,route_press,refresh,show_avoid,show_root,aux_valid;
 wire [7:0] manual_action,response,aux_type,aux_code,mode_status,start_diag;
 reg completed=0;integer checks=0,announces=0,obstacles=0,clears=0,exits=0,navigations=0;reg [7:0] token,old_token;
 system_dispatch #(.CLK_HZ(1000),.AUDIO_TIMEOUT_MS(200)) dut(.*);
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
 task key_toggle;begin menu_command=8'h16;menu_valid=1;cycles(1);menu_valid=0;cycles(3);end endtask
 task leave_mode;begin stop_gate=1;cycles(1);stop_gate=0;busy=0;route_state=0;cycles(5);check(!avoid_mode,"board exit");end endtask
 task blocked_controls;reg was_auto,was_wait;integer nav;begin
 was_auto=auto_run;was_wait=waiting;nav=navigations;
 for(integer i=0;i<11;i++)begin
 command=(i<3)?i+1:8'h10+i-3;cv=1;#1;
 check(!voice_command_valid && !stopping && !route_cancel && !manual_submit,"blocked control has no arbitration effects");
 cycles(1);cv=0;cycles(2);
 check(avoid_mode && auto_run==was_auto && waiting==was_wait && !manual_active && !route_press && !refresh && navigations==nav,"blocked control preserves mode and route");
 end
 end endtask
 initial begin
 cycles(2);rst_n=1;cycles(2);
 voice(8'h16);check(!avoid_mode && announces==0 && navigations==1,"voice off entry navigates but does not start");
 key_toggle();check(avoid_mode && auto_run && !waiting,"voice off K2 starts without audio");blocked_controls();
 key_toggle();check(!auto_run && avoid_mode,"K2 pauses with voice off");blocked_controls();leave_mode();
 voice_enable=1;voice(8'h16);check(waiting && announces==1,"voice entry waits");blocked_controls();
 ack=token+1;ack_valid=1;cycles(1);ack_valid=0;cycles(2);check(!auto_run,"wrong token ignored");
 post_fresh=0;confirm();check(waiting && !auto_run,"old sensor shots cannot start");post_fresh=1;cycles(2);check(auto_run,"fresh samples allow start");
 busy=1;route_state=2;cycles(3);blocked_controls();
 route_state=3;cycles(3);check(obstacles==1,"one obstacle event");cycles(3);check(obstacles==1,"no level duplicate");
 route_state=20;cycles(2);route_state=1;cycles(3);check(clears==1,"only complete return event");
 for(integer stage=1;stage<=23;stage++)if(stage!=17 && stage!=18)begin
 route_state=stage;cycles(3);blocked_controls();
 end
 key_toggle();busy=0;cycles(3);check(!auto_run && avoid_mode,"board pause");blocked_controls();
 key_toggle();check(auto_run && !waiting,"board resume needs no audio");leave_mode();
 voice(8'h16);cycles(202);check(!waiting && !auto_run && avoid_mode,"missing audio timeout pauses");confirm();check(!auto_run,"late confirmation ignored");leave_mode();
 voice(8'h16);old_token=token;leave_mode();voice(8'h16);
 ack=old_token;ack_valid=1;cycles(1);ack_valid=0;cycles(2);check(waiting && !auto_run,"superseded audio request ignored");
 old_token=token;rst_n=0;cycles(2);rst_n=1;cycles(2);voice(8'h16);
 ack=old_token;ack_valid=1;cycles(1);ack_valid=0;cycles(2);check(waiting && !auto_run && token!=old_token,"soft reset preserves token sequence");leave_mode();
 voice(8'h16);fault=1;cycles(2);confirm();check(!auto_run && !waiting,"fault invalidates audio");fault=0;leave_mode();
 for(integer blocked=17;blocked<=18;blocked++)begin
 start();route_state=blocked;cycles(3);check(!auto_run && avoid_mode,"blocked or failed route retains mode");blocked_controls();leave_mode();
 end
 // Non-avoidance voice movement and stop remain available.
 cmd_ready=0;voice(8'h12);check(manual_action==8'h12 && !manual_submit,"outside mode manual captured");
 cmd_ready=1;@(posedge clk);#1;busy=1;cycles(2);check(manual_active,"outside mode manual accepted");
 voice(2);check(!auto_run,"outside mode stop");busy=0;done=1;cycles(1);done=0;cycles(3);
 start();busy=1;stop_gate=1;command=8'h10;cv=1;cycles(2);cv=0;stop_gate=0;check(!avoid_mode && !auto_run,"board stop wins over ignored voice");busy=0;cycles(5);
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
