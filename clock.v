module decoder_am_pm (cntr,Seven_seg,pm);
input [3:0] cntr;
input pm;
output reg [7:0] Seven_seg;
reg [6:0] val;
always@(cntr)
    begin
        case(cntr)
                        // 7'abcdefg
            4'b0000:val = 7'b1111110;
            4'b0001:val = 7'b0110000;
            4'b0010:val = 7'b1101101;
            4'b0011:val = 7'b1111001;
            4'b0100:val = 7'b0110011;
            4'b0101:val = 7'b1011011;
            4'b0110:val = 7'b1011111;
            4'b0111:val = 7'b1110000;
            4'b1000:val = 7'b1111111;
            4'b1001:val = 7'b1111011;
            default:val = 7'b0000000;
        endcase
        Seven_seg = {~pm,~val};
    end
endmodule
module decoder (cntr,Seven_seg);
input [3:0] cntr;
output reg [7:0] Seven_seg;
reg [6:0] val;
always@(cntr)
    begin
        case(cntr)
                        // 7'abcdefg
            4'b0000:val = 7'b1111110;
            4'b0001:val = 7'b0110000;
            4'b0010:val = 7'b1101101;
            4'b0011:val = 7'b1111001;
            4'b0100:val = 7'b0110011;
            4'b0101:val = 7'b1011011;
            4'b0110:val = 7'b1011111;
            4'b0111:val = 7'b1110000;
            4'b1000:val = 7'b1111111;
            4'b1001:val = 7'b1111011;
            default:val = 7'b0000000;
        endcase
        Seven_seg = {1'b1,~val};
    end
endmodule
module clk_divider(inClk,outClk,rst);
    input inClk;  // clock present in the FPGA 50MHz
    input rst;
    output reg outClk;
    reg [27:0] clockCount;
    reg [27:0] v = 25000000;   // 25MHz to get 1Hz clock as output
    always @(posedge inClk)begin
        if (rst)begin
            outClk=1'b0;
            clockCount=0;
        end else begin
            if(v==clockCount)begin
                outClk = ~outClk;
                clockCount = 0;
            end else begin
                clockCount = clockCount+1;
            end
        end
    end
endmodule
// This module implements a simple digital clock with AM/PM indication
// It uses a clock divider to slow down the clock present in the FPGA and a decoder to convert binary values to 7-segment display values
// The clock counts minutes and hours, and toggles AM/PM at the appropriate time
module top_module(
    input clk,
    input reset,
    input ena,
    output reg pm,
    output [7:0] Seven_seg_M1,
    output [7:0] Seven_seg_M2,
    output [7:0] Seven_seg_H1,
    output [7:0] Seven_seg_H2
);
    
    reg [7:0] hh;
    reg [7:0] mm;
    reg [7:0] ss;
    wire clk_diveded;
    clk_divider uut(clk,clk_diveded,reset);
    decoder uut1(hh[7:4],Seven_seg_H1);
    decoder uut2(hh[3:0],Seven_seg_H2);
    decoder uut3(mm[7:4],Seven_seg_M1);
    decoder_am_pm uut6(ss[3:0],Seven_seg_M2,pm);
    initial begin
        ss<=0;
        mm<=0;
        hh<=8'h12;
        pm<=0;
    end
    always@(posedge clk_diveded)begin
        if(reset)begin
            ss<=0;
            mm<=0;
            hh<=8'h12;
            pm<=0;
        end else if(ena)begin
            if(ss[3:0]==9)begin
                ss[3:0]=0;
                if(ss[7:4]==5)begin
                    ss[7:4]=0;
                    if(mm[3:0]==9)begin
                        mm[3:0]=0;
                        if(mm[7:4]==5)begin
                            mm[7:4]=0;
                            if(hh[3:0]==9)begin
                                hh[3:0]=0;
                                hh[7:4]=hh[7:4]+1;
                            end else begin
                                if(hh[7:4]==1 && hh[3:0]==1)begin
        							pm=~pm;
                                    hh[3:0]=hh[3:0]+1;
                                end else if(hh[7:4]==1 && hh[3:0]==2)begin
                                    hh[7:4]=0;
                                    hh[3:0]=1;                                    
                                end else begin
                                    hh[3:0]=hh[3:0]+1; 
                                end
                            end
                        end else begin
                            mm[7:4]=mm[7:4]+1;
                        end
                    end else begin
                        mm[3:0]=mm[3:0]+1;
                    end
                end else begin
                    ss[7:4]=ss[7:4]+1;                    
                end
            end else begin
                ss[3:0]=ss[3:0]+1;
            end
        end
    end
endmodule

module tb;
    reg clk=0,reset=1;
    reg ena=1;
    wire [7:0] Seven_seg_H1,Seven_seg_H2,Seven_seg_M1,Seven_seg_M2;
    wire pm;
    top_module uut(clk,reset,ena,pm,Seven_seg_H1,Seven_seg_H2,Seven_seg_M1,Seven_seg_M2);
    always#0.5 clk=~clk;
    initial begin
        $dumpfile("clock.vcd");
        $dumpvars(0,tb);
        #10 reset=0;
        #1000000 $finish;
    end
endmodule