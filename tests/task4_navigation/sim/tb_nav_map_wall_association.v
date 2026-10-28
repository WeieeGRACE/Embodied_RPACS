`timescale 1ns/1ps
module tb_nav_map_wall_association;
    reg signed [31:0] x_mm=2700,y_mm=300,yaw_mdeg=0;
    reg [1:0] direction=0;
    reg [7:0] expected_wall=10;
    reg range_valid=1;
    reg [15:0] measured_mm=578;
    wire [7:0] ray_wall;
    wire [15:0] predicted_mm;
    wire associated;

    nav_map_wall_association dut(
      .pose_x_mm(x_mm),.pose_y_mm(y_mm),.yaw_mdeg(yaw_mdeg),.tof_dir(direction),
      .expected_wall_id(expected_wall),.range_valid(range_valid),
      .measured_range_mm(measured_mm),.ray_wall_id(ray_wall),
      .predicted_range_mm(predicted_mm),.associated(associated));

    initial begin
        #1;
        if(ray_wall!=10 || predicted_mm!=578 || !associated)
            $fatal(1,"front ray did not associate with nearest MID_H wall id=%0d range=%0d ok=%b",
                   ray_wall,predicted_mm,associated);

        // A nearer obstacle returns 578 mm where the route's expected top
        // boundary predicts 2378 mm, so the segment event must not associate.
        expected_wall=3;measured_mm=578;#1;
        if(ray_wall!=3 || predicted_mm!=2378 || associated)
            $fatal(1,"nearer obstacle was incorrectly associated id=%0d range=%0d ok=%b",
                   ray_wall,predicted_mm,associated);

        expected_wall=10;measured_mm=16'd20;#1;
        if(associated) $fatal(1,"20 mm sample was accepted by map association");

        // Left sensor points west from its forward/lateral mount offset and
        // sees A_SIDE before any farther wall.
        x_mm=2700;y_mm=300;direction=1;expected_wall=14;measured_mm=178;#1;
        if(ray_wall!=14 || predicted_mm!=178 || !associated)
            $fatal(1,"left ray failed to associate A_SIDE id=%0d range=%0d ok=%b",
                   ray_wall,predicted_mm,associated);
        $display("PASS: expected-wall range association, nearer-obstacle rejection, exact-20 rejection, side sensor geometry");
        $finish;
    end
endmodule
