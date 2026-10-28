`timescale 1ns/1ps
// A map-linked wall event requires fresh range samples, an entry edge into the
// mapped ToF band, and an independent odometry-distance window. The map ID is
// emitted from the route ROM; no observed ID is accepted from the testbench.
module nav_tof_wall_event #(
    parameter integer RANGE_AGE_LIMIT=250,
    parameter integer CONSECUTIVE_REQUIRED=3,
    parameter integer HYSTERESIS_MM=30,
    parameter integer EDGE_DELTA_MM=80
) (
    input wire clk, input wire rst_n,
    input wire clear_event,
    input wire enable,
    input wire age_tick,
    input wire [7:0] expected_wall_id,
    input wire map_match,
    input wire sample_update,
    input wire sample_valid,
    input wire [15:0] sample_mm,
    input wire [15:0] event_range_min_mm,
    input wire [15:0] event_range_max_mm,
    input wire [31:0] segment_distance_mm,
    input wire [15:0] distance_min_mm,
    input wire [15:0] distance_max_mm,
    output reg [15:0] age,
    output reg [15:0] accepted_mm,
    output reg accepted_valid,
    output reg accepted_pulse,
    output reg [7:0] wall_event_id,
    output reg event_pulse,
    output reg event_latched,
    output reg stale_fault,
    output reg sample_discarded_20
);
    reg [7:0] consecutive;
    reg [15:0] previous_mm;
    reg previous_valid;
    reg edge_seen;

    wire normal_sample = sample_update && sample_valid && sample_mm!=16'd20 &&
                          sample_mm>=16'd25 && sample_mm<=16'd2500;
    wire range_in_band = normal_sample && sample_mm>=event_range_min_mm &&
                         sample_mm<=event_range_max_mm;
    wire distance_in_band = segment_distance_mm>=distance_min_mm &&
                            segment_distance_mm<=distance_max_mm;
    wire stable_change = previous_valid &&
        ((sample_mm>=previous_mm && sample_mm-previous_mm<=HYSTERESIS_MM) ||
         (previous_mm>sample_mm && previous_mm-sample_mm<=HYSTERESIS_MM));
    wire edge_change = previous_valid &&
        ((sample_mm>previous_mm && sample_mm-previous_mm>=EDGE_DELTA_MM) ||
         (previous_mm>sample_mm && previous_mm-sample_mm>=EDGE_DELTA_MM));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            age<=16'hffff; accepted_mm<=0; accepted_valid<=0;
            accepted_pulse<=0; wall_event_id<=0; event_pulse<=0;
            event_latched<=0; stale_fault<=0; sample_discarded_20<=0;
            consecutive<=0; previous_mm<=0; previous_valid<=0; edge_seen<=0;
        end else begin
            accepted_pulse<=0; event_pulse<=0; sample_discarded_20<=0;
            if (age_tick && age!=16'hffff) age<=age+1'b1;

            if (sample_update && sample_valid && sample_mm==16'd20) begin
                // 20 mm is discarded: it cannot refresh accepted value or age.
                sample_discarded_20<=1;consecutive<=0;previous_valid<=0;
            end else if (normal_sample) begin
                accepted_mm<=sample_mm; accepted_valid<=1; accepted_pulse<=1; age<=0;
                previous_mm<=sample_mm; previous_valid<=1;
                if (!range_in_band) begin
                    if (sample_mm<event_range_min_mm-HYSTERESIS_MM ||
                        sample_mm>event_range_max_mm+HYSTERESIS_MM)
                        edge_seen<=1;
                    consecutive<=0;
                end else if (edge_seen && (!previous_valid || edge_change || stable_change) &&
                             enable && map_match && expected_wall_id!=0 &&
                             distance_in_band && !event_latched) begin
                    // Fresh, stable samples inside this route's map-linked range
                    // confirm the segment event; counts advance only on updates.
                    if (consecutive<CONSECUTIVE_REQUIRED)
                        consecutive<=consecutive+1'b1;
                    if (consecutive+1>=CONSECUTIVE_REQUIRED) begin
                        event_latched<=1; event_pulse<=1;
                        wall_event_id<=expected_wall_id;
                    end
                end else begin
                    consecutive<=0;
                end
            end else if (sample_update) begin
                // A failed/invalid completion does not refresh age or progress debounce.
                consecutive<=0;previous_valid<=0;
            end

            if (enable && age>=RANGE_AGE_LIMIT) stale_fault<=1;
            if (clear_event) begin
                event_latched<=0; wall_event_id<=0; consecutive<=0;
                // Entering a route segment arms the map association. A band
                // excursion can also arm it while the segment is in progress.
                edge_seen<=1; stale_fault<=0;
            end else if (!enable) begin
                consecutive<=0;
            end
        end
    end
endmodule
