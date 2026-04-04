`timescale 1ns / 1ps
//
// trackingAXIS - 3-Word Packet Design (25 Channels)
//
// Packet format (3 words × 64 bits = 24 bytes per channel):
//   Word 0: [63:32] = ABS counter (32 bits), [31:0] = Carrier Freq (32 bits)
//   Word 1: [63:47] = I_P integer (17 bits, signed), [46:30] = Q_P integer (17 bits, signed),
//           [29:17] = reserved (13 bits), [16:15] Sat_sys, [14:13] Freq Band, [12:8] = channel (5 bits), [7:0] = PRN (8 bits)
//   Word 2: [63:0] = Carrier Phase (64 bits)
//
// I_P / Q_P are Q17.16 format (33 bits total), only the 17-bit integer part [32:16] is sent.
//
// Design contract:
// - chX_valid is a single-cycle pulse every 1 ms per channel
// - Data captured immediately on valid pulse
// - Clock domain: 16.384 MHz
// - AXI-Stream output @ 100 MHz via clock converter -> DMA -> SmartConnect -> MPSoC
//

module trackingAXIS(
    input  wire        clk,          // 16.384 MHz
    input  wire        rst,

    // Common absolute counter (captured per-channel on valid pulse)
    input  wire [31:0] abs_count,

    // Channel 0
    input  wire        ch0_valid,
    input  wire [32:0] ch0_I,        // Q17.16 format
    input  wire [32:0] ch0_Q,        // Q17.16 format
    input  wire [31:0] ch0_freq,
    input  wire [63:0] ch0_phase,
    input  wire [7:0]  ch0_PRN,
    input  wire [1:0]  ch0_sat_sys,
    input  wire [1:0]  ch0_freq_band,

    // Channel 1
    input  wire        ch1_valid,
    input  wire [32:0] ch1_I,
    input  wire [32:0] ch1_Q,
    input  wire [31:0] ch1_freq,
    input  wire [63:0] ch1_phase,
    input  wire [7:0]  ch1_PRN,
    input  wire [1:0]  ch1_sat_sys,
    input  wire [1:0]  ch1_freq_band,

    // Channel 2
    input  wire        ch2_valid,
    input  wire [32:0] ch2_I,
    input  wire [32:0] ch2_Q,
    input  wire [31:0] ch2_freq,
    input  wire [63:0] ch2_phase,
    input  wire [7:0]  ch2_PRN,
    input  wire [1:0]  ch2_sat_sys,
    input  wire [1:0]  ch2_freq_band,

    // Channel 3
    input  wire        ch3_valid,
    input  wire [32:0] ch3_I,
    input  wire [32:0] ch3_Q,
    input  wire [31:0] ch3_freq,
    input  wire [63:0] ch3_phase,
    input  wire [7:0]  ch3_PRN,
    input  wire [1:0]  ch3_sat_sys,
    input  wire [1:0]  ch3_freq_band,
    
    // Channel 4
    input  wire        ch4_valid,
    input  wire [32:0] ch4_I,
    input  wire [32:0] ch4_Q,
    input  wire [31:0] ch4_freq,
    input  wire [63:0] ch4_phase,
    input  wire [7:0]  ch4_PRN,
    input  wire [1:0]  ch4_sat_sys,
    input  wire [1:0]  ch4_freq_band,
    
    // Channel 5
    input  wire        ch5_valid,
    input  wire [32:0] ch5_I,
    input  wire [32:0] ch5_Q,
    input  wire [31:0] ch5_freq,
    input  wire [63:0] ch5_phase,
    input  wire [7:0]  ch5_PRN,
    input  wire [1:0]  ch5_sat_sys,
    input  wire [1:0]  ch5_freq_band,
    
    // Channel 6
    input  wire        ch6_valid,
    input  wire [32:0] ch6_I,
    input  wire [32:0] ch6_Q,
    input  wire [31:0] ch6_freq,
    input  wire [63:0] ch6_phase,
    input  wire [7:0]  ch6_PRN,
    input  wire [1:0]  ch6_sat_sys,
    input  wire [1:0]  ch6_freq_band,
    
    // Channel 7
    input  wire        ch7_valid,
    input  wire [32:0] ch7_I,
    input  wire [32:0] ch7_Q,
    input  wire [31:0] ch7_freq,
    input  wire [63:0] ch7_phase,
    input  wire [7:0]  ch7_PRN,
    input  wire [1:0]  ch7_sat_sys,
    input  wire [1:0]  ch7_freq_band,
    
    // Channel 8
    input  wire        ch8_valid,
    input  wire [32:0] ch8_I,
    input  wire [32:0] ch8_Q,
    input  wire [31:0] ch8_freq,
    input  wire [63:0] ch8_phase,
    input  wire [7:0]  ch8_PRN,
    input  wire [1:0]  ch8_sat_sys,
    input  wire [1:0]  ch8_freq_band,
    
    // Channel 9
    input  wire        ch9_valid,
    input  wire [32:0] ch9_I,
    input  wire [32:0] ch9_Q,
    input  wire [31:0] ch9_freq,
    input  wire [63:0] ch9_phase,
    input  wire [7:0]  ch9_PRN,
    input  wire [1:0]  ch9_sat_sys,
    input  wire [1:0]  ch9_freq_band,

    // Channel 10
    input  wire        ch10_valid,
    input  wire [32:0] ch10_I,
    input  wire [32:0] ch10_Q,
    input  wire [31:0] ch10_freq,
    input  wire [63:0] ch10_phase,
    input  wire [7:0]  ch10_PRN,
    input  wire [1:0]  ch10_sat_sys,
    input  wire [1:0]  ch10_freq_band,

    // Channel 11
    input  wire        ch11_valid,
    input  wire [32:0] ch11_I,
    input  wire [32:0] ch11_Q,
    input  wire [31:0] ch11_freq,
    input  wire [63:0] ch11_phase,
    input  wire [7:0]  ch11_PRN,
    input  wire [1:0]  ch11_sat_sys,
    input  wire [1:0]  ch11_freq_band,

    // Channel 12
    input  wire        ch12_valid,
    input  wire [32:0] ch12_I,
    input  wire [32:0] ch12_Q,
    input  wire [31:0] ch12_freq,
    input  wire [63:0] ch12_phase,
    input  wire [7:0]  ch12_PRN,
    input  wire [1:0]  ch12_sat_sys,
    input  wire [1:0]  ch12_freq_band,

    // Channel 13
    input  wire        ch13_valid,
    input  wire [32:0] ch13_I,
    input  wire [32:0] ch13_Q,
    input  wire [31:0] ch13_freq,
    input  wire [63:0] ch13_phase,
    input  wire [7:0]  ch13_PRN,
    input  wire [1:0]  ch13_sat_sys,
    input  wire [1:0]  ch13_freq_band,

    // Channel 14
    input  wire        ch14_valid,
    input  wire [32:0] ch14_I,
    input  wire [32:0] ch14_Q,
    input  wire [31:0] ch14_freq,
    input  wire [63:0] ch14_phase,
    input  wire [7:0]  ch14_PRN,
    input  wire [1:0]  ch14_sat_sys,
    input  wire [1:0]  ch14_freq_band,

    // Channel 15
    input  wire        ch15_valid,
    input  wire [32:0] ch15_I,
    input  wire [32:0] ch15_Q,
    input  wire [31:0] ch15_freq,
    input  wire [63:0] ch15_phase,
    input  wire [7:0]  ch15_PRN,
    input  wire [1:0]  ch15_sat_sys,
    input  wire [1:0]  ch15_freq_band,

    // Channel 16
    input  wire        ch16_valid,
    input  wire [32:0] ch16_I,
    input  wire [32:0] ch16_Q,
    input  wire [31:0] ch16_freq,
    input  wire [63:0] ch16_phase,
    input  wire [7:0]  ch16_PRN,
    input  wire [1:0]  ch16_sat_sys,
    input  wire [1:0]  ch16_freq_band,

    // Channel 17
    input  wire        ch17_valid,
    input  wire [32:0] ch17_I,
    input  wire [32:0] ch17_Q,
    input  wire [31:0] ch17_freq,
    input  wire [63:0] ch17_phase,
    input  wire [7:0]  ch17_PRN,
    input  wire [1:0]  ch17_sat_sys,
    input  wire [1:0]  ch17_freq_band,

    // Channel 18
    input  wire        ch18_valid,
    input  wire [32:0] ch18_I,
    input  wire [32:0] ch18_Q,
    input  wire [31:0] ch18_freq,
    input  wire [63:0] ch18_phase,
    input  wire [7:0]  ch18_PRN,
    input  wire [1:0]  ch18_sat_sys,
    input  wire [1:0]  ch18_freq_band,

    // Channel 19
    input  wire        ch19_valid,
    input  wire [32:0] ch19_I,
    input  wire [32:0] ch19_Q,
    input  wire [31:0] ch19_freq,
    input  wire [63:0] ch19_phase,
    input  wire [7:0]  ch19_PRN,
    input  wire [1:0]  ch19_sat_sys,
    input  wire [1:0]  ch19_freq_band,

    // Channel 20
    input  wire        ch20_valid,
    input  wire [32:0] ch20_I,
    input  wire [32:0] ch20_Q,
    input  wire [31:0] ch20_freq,
    input  wire [63:0] ch20_phase,
    input  wire [7:0]  ch20_PRN,
    input  wire [1:0]  ch20_sat_sys,
    input  wire [1:0]  ch20_freq_band,

    // Channel 21
    input  wire        ch21_valid,
    input  wire [32:0] ch21_I,
    input  wire [32:0] ch21_Q,
    input  wire [31:0] ch21_freq,
    input  wire [63:0] ch21_phase,
    input  wire [7:0]  ch21_PRN,
    input  wire [1:0]  ch21_sat_sys,
    input  wire [1:0]  ch21_freq_band,

    // Channel 22
    input  wire        ch22_valid,
    input  wire [32:0] ch22_I,
    input  wire [32:0] ch22_Q,
    input  wire [31:0] ch22_freq,
    input  wire [63:0] ch22_phase,
    input  wire [7:0]  ch22_PRN,
    input  wire [1:0]  ch22_sat_sys,
    input  wire [1:0]  ch22_freq_band,

    // Channel 23
    input  wire        ch23_valid,
    input  wire [32:0] ch23_I,
    input  wire [32:0] ch23_Q,
    input  wire [31:0] ch23_freq,
    input  wire [63:0] ch23_phase,
    input  wire [7:0]  ch23_PRN,
    input  wire [1:0]  ch23_sat_sys,
    input  wire [1:0]  ch23_freq_band,

    // Channel 24
    input  wire        ch24_valid,
    input  wire [32:0] ch24_I,
    input  wire [32:0] ch24_Q,
    input  wire [31:0] ch24_freq,
    input  wire [63:0] ch24_phase,
    input  wire [7:0]  ch24_PRN,
    input  wire [1:0]  ch24_sat_sys,
    input  wire [1:0]  ch24_freq_band,
    
    // AXI4-Stream Master (connects to Clock Converter -> DMA)
    output reg  [63:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast
);

    // ---------------------------------------------------------
    // FSM states
    // ---------------------------------------------------------
    localparam [1:0]
        IDLE     = 2'd0,
        SEND_HDR = 2'd1,
        SEND_IQ  = 2'd2,
        SEND_PHASE = 2'd3;

    reg [1:0] state;

    // ---------------------------------------------------------
    // Captured packet data storage (10 channels)
    // ---------------------------------------------------------
    reg [4:0]  captured_ch    [0:24];
    reg [7:0]  captured_PRN   [0:24];
    reg [16:0] captured_I     [0:24];  // 17-bit integer part of Q17.16
    reg [16:0] captured_Q     [0:24];  // 17-bit integer part of Q17.16
    reg [31:0] captured_ABS   [0:24];
    reg [31:0] captured_freq  [0:24];
    reg [63:0] captured_phase [0:24];
    reg [1:0]  captured_sat_sys [0:24];
    reg [1:0]  captured_freq_band [0:24];

    // ---------------------------------------------------------
    // Pending flags (1 per channel)
    // Set when valid pulse captured, cleared when packet sent
    // INVARIANT: If clear and set happen same cycle, set wins (new data priority)
    // ---------------------------------------------------------
    reg [24:0] pending;

    // ---------------------------------------------------------
    // Current packet being transmitted
    // ---------------------------------------------------------
    reg [4:0]  tx_ch;
    reg [7:0]  tx_PRN;
    reg [16:0] tx_I;
    reg [16:0] tx_Q;
    reg [31:0] tx_ABS;
    reg [31:0] tx_freq;
    reg [63:0] tx_phase;
    reg [1:0] tx_sat_sys;
    reg [1:0] tx_freq_band;

    // ---------------------------------------------------------
    // Priority encoder for channel selection
    // ---------------------------------------------------------
    wire       any_pending;
        wire [4:0] next_ch;

    assign any_pending = |pending;

    // Fixed priority: Ch0 has highest priority
    assign next_ch =
        pending[0]  ? 5'd0  :
        pending[1]  ? 5'd1  :
        pending[2]  ? 5'd2  :
        pending[3]  ? 5'd3  :
        pending[4]  ? 5'd4  :
        pending[5]  ? 5'd5  :
        pending[6]  ? 5'd6  :
        pending[7]  ? 5'd7  :
        pending[8]  ? 5'd8  :
        pending[9]  ? 5'd9  :
        pending[10] ? 5'd10 :
        pending[11] ? 5'd11 :
        pending[12] ? 5'd12 :
        pending[13] ? 5'd13 :
        pending[14] ? 5'd14 :
        pending[15] ? 5'd15 :
        pending[16] ? 5'd16 :
        pending[17] ? 5'd17 :
        pending[18] ? 5'd18 :
        pending[19] ? 5'd19 :
        pending[20] ? 5'd20 :
        pending[21] ? 5'd21 :
        pending[22] ? 5'd22 :
        pending[23] ? 5'd23 :
        pending[24] ? 5'd24 : 5'd0;

    // ---------------------------------------------------------
    // DATA CAPTURE: Immediate capture on valid pulse
    // ---------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            pending <= 25'b0;
        end else begin
            // Clear pending when packet completes
            if (state == SEND_PHASE && m_axis_tready) begin
                pending[tx_ch] <= 1'b0;
            end

            // Channel 0
            if (ch0_valid) begin
                captured_ch[0]    <= 5'd0;
                captured_PRN[0]   <= ch0_PRN;
                captured_I[0]     <= ch0_I[32:16];   // Integer part of Q17.16
                captured_Q[0]     <= ch0_Q[32:16];
                captured_ABS[0]   <= abs_count;
                captured_freq[0] <= ch0_freq;
                captured_phase[0] <= ch0_phase;
                captured_sat_sys[0] <= ch0_sat_sys;
                captured_freq_band[0] <= ch0_freq_band;
                pending[0]        <= 1'b1;
            end

            // Channel 1
            if (ch1_valid) begin
                captured_ch[1]    <= 5'd1;
                captured_PRN[1]   <= ch1_PRN;
                captured_I[1]     <= ch1_I[32:16];
                captured_Q[1]     <= ch1_Q[32:16];
                captured_ABS[1]   <= abs_count;
                captured_freq[1] <= ch1_freq;
                captured_phase[1] <= ch1_phase;
                captured_sat_sys[1] <= ch1_sat_sys;
                captured_freq_band[1] <= ch1_freq_band;
                pending[1]        <= 1'b1;
            end

            // Channel 2
            if (ch2_valid) begin
                captured_ch[2]    <= 5'd2;
                captured_PRN[2]   <= ch2_PRN;
                captured_I[2]     <= ch2_I[32:16];
                captured_Q[2]     <= ch2_Q[32:16];
                captured_ABS[2]   <= abs_count;
                captured_freq[2] <= ch2_freq;
                captured_phase[2] <= ch2_phase;
                captured_sat_sys[2] <= ch2_sat_sys;
                captured_freq_band[2] <= ch2_freq_band;
                pending[2]        <= 1'b1;
            end

            // Channel 3
            if (ch3_valid) begin
                captured_ch[3]    <= 5'd3;
                captured_PRN[3]   <= ch3_PRN;
                captured_I[3]     <= ch3_I[32:16];
                captured_Q[3]     <= ch3_Q[32:16];
                captured_ABS[3]   <= abs_count;
                captured_freq[3] <= ch3_freq;
                captured_phase[3] <= ch3_phase;
                captured_sat_sys[3] <= ch3_sat_sys;
                captured_freq_band[3] <= ch3_freq_band;
                pending[3]        <= 1'b1;
            end

            // Channel 4
            if (ch4_valid) begin
                captured_ch[4]    <= 5'd4;
                captured_PRN[4]   <= ch4_PRN;
                captured_I[4]     <= ch4_I[32:16];
                captured_Q[4]     <= ch4_Q[32:16];
                captured_ABS[4]   <= abs_count;
                captured_freq[4] <= ch4_freq;
                captured_phase[4] <= ch4_phase;
                captured_sat_sys[4] <= ch4_sat_sys;
                captured_freq_band[4] <= ch4_freq_band;                
                pending[4]        <= 1'b1;
            end

            // Channel 5
            if (ch5_valid) begin
                captured_ch[5]    <= 5'd5;
                captured_PRN[5]   <= ch5_PRN;
                captured_I[5]     <= ch5_I[32:16];
                captured_Q[5]     <= ch5_Q[32:16];
                captured_ABS[5]   <= abs_count;
                captured_freq[5] <= ch5_freq;
                captured_phase[5] <= ch5_phase;
                captured_sat_sys[5] <= ch5_sat_sys;
                captured_freq_band[5] <= ch5_freq_band;  
                pending[5]        <= 1'b1;
            end

            // Channel 6
            if (ch6_valid) begin
                captured_ch[6]    <= 5'd6;
                captured_PRN[6]   <= ch6_PRN;
                captured_I[6]     <= ch6_I[32:16];
                captured_Q[6]     <= ch6_Q[32:16];
                captured_ABS[6]   <= abs_count;
                captured_freq[6] <= ch6_freq;
                captured_phase[6] <= ch6_phase;
                captured_sat_sys[6] <= ch6_sat_sys;
                captured_freq_band[6] <= ch6_freq_band; 
                pending[6]        <= 1'b1;
            end

            // Channel 7
            if (ch7_valid) begin
                captured_ch[7]    <= 5'd7;
                captured_PRN[7]   <= ch7_PRN;
                captured_I[7]     <= ch7_I[32:16];
                captured_Q[7]     <= ch7_Q[32:16];
                captured_ABS[7]   <= abs_count;
                captured_freq[7] <= ch7_freq;
                captured_phase[7] <= ch7_phase;
                captured_sat_sys[7] <= ch7_sat_sys;
                captured_freq_band[7] <= ch7_freq_band; 
                pending[7]        <= 1'b1;
            end

            // Channel 8
            if (ch8_valid) begin
                captured_ch[8]    <= 5'd8;
                captured_PRN[8]   <= ch8_PRN;
                captured_I[8]     <= ch8_I[32:16];
                captured_Q[8]     <= ch8_Q[32:16];
                captured_ABS[8]   <= abs_count;
                captured_freq[8] <= ch8_freq;
                captured_phase[8] <= ch8_phase;
                captured_sat_sys[8] <= ch8_sat_sys;
                captured_freq_band[8] <= ch8_freq_band; 
                pending[8]        <= 1'b1;
            end

            // Channel 9
            if (ch9_valid) begin
                captured_ch[9]    <= 5'd9;
                captured_PRN[9]   <= ch9_PRN;
                captured_I[9]     <= ch9_I[32:16];
                captured_Q[9]     <= ch9_Q[32:16];
                captured_ABS[9]   <= abs_count;
                captured_freq[9]  <= ch9_freq;
                captured_phase[9] <= ch9_phase;
                captured_sat_sys[9] <= ch9_sat_sys;
                captured_freq_band[9] <= ch9_freq_band; 
                pending[9]        <= 1'b1;
            end

            // Channel 10
            if (ch10_valid) begin
                captured_ch[10]    <= 5'd10;
                captured_PRN[10]   <= ch10_PRN;
                captured_I[10]     <= ch10_I[32:16];
                captured_Q[10]     <= ch10_Q[32:16];
                captured_ABS[10]   <= abs_count;
                captured_freq[10]  <= ch10_freq;
                captured_phase[10] <= ch10_phase;
                captured_sat_sys[10] <= ch10_sat_sys;
                captured_freq_band[10] <= ch10_freq_band; 
                pending[10]        <= 1'b1;
            end

            // Channel 11
            if (ch11_valid) begin
                captured_ch[11]    <= 5'd11;
                captured_PRN[11]   <= ch11_PRN;
                captured_I[11]     <= ch11_I[32:16];
                captured_Q[11]     <= ch11_Q[32:16];
                captured_ABS[11]   <= abs_count;
                captured_freq[11]  <= ch11_freq;
                captured_phase[11] <= ch11_phase;
                captured_sat_sys[11] <= ch11_sat_sys;
                captured_freq_band[11] <= ch11_freq_band; 
                pending[11]        <= 1'b1;
            end

            // Channel 12
            if (ch12_valid) begin
                captured_ch[12]    <= 5'd12;
                captured_PRN[12]   <= ch12_PRN;
                captured_I[12]     <= ch12_I[32:16];
                captured_Q[12]     <= ch12_Q[32:16];
                captured_ABS[12]   <= abs_count;
                captured_freq[12]  <= ch12_freq;
                captured_phase[12] <= ch12_phase;
                captured_sat_sys[12] <= ch12_sat_sys;
                captured_freq_band[12] <= ch12_freq_band; 
                pending[12]        <= 1'b1;
            end

            // Channel 13
            if (ch13_valid) begin
                captured_ch[13]    <= 5'd13;
                captured_PRN[13]   <= ch13_PRN;
                captured_I[13]     <= ch13_I[32:16];
                captured_Q[13]     <= ch13_Q[32:16];
                captured_ABS[13]   <= abs_count;
                captured_freq[13]  <= ch13_freq;
                captured_phase[13] <= ch13_phase;
                captured_sat_sys[13] <= ch13_sat_sys;
                captured_freq_band[13] <= ch13_freq_band; 
                pending[13]        <= 1'b1;
            end

            // Channel 14
            if (ch14_valid) begin
                captured_ch[14]    <= 5'd14;
                captured_PRN[14]   <= ch14_PRN;
                captured_I[14]     <= ch14_I[32:16];
                captured_Q[14]     <= ch14_Q[32:16];
                captured_ABS[14]   <= abs_count;
                captured_freq[14]  <= ch14_freq;
                captured_phase[14] <= ch14_phase;
                captured_sat_sys[14] <= ch14_sat_sys;
                captured_freq_band[14] <= ch14_freq_band; 
                pending[14]        <= 1'b1;
            end

            // Channel 15
            if (ch15_valid) begin
                captured_ch[15]    <= 5'd15;
                captured_PRN[15]   <= ch15_PRN;
                captured_I[15]     <= ch15_I[32:16];
                captured_Q[15]     <= ch15_Q[32:16];
                captured_ABS[15]   <= abs_count;
                captured_freq[15]  <= ch15_freq;
                captured_phase[15] <= ch15_phase;
                captured_sat_sys[15] <= ch15_sat_sys;
                captured_freq_band[15] <= ch15_freq_band; 
                pending[15]        <= 1'b1;
            end

            // Channel 16
            if (ch16_valid) begin
                captured_ch[16]    <= 5'd16;
                captured_PRN[16]   <= ch16_PRN;
                captured_I[16]     <= ch16_I[32:16];
                captured_Q[16]     <= ch16_Q[32:16];
                captured_ABS[16]   <= abs_count;
                captured_freq[16]  <= ch16_freq;
                captured_phase[16] <= ch16_phase;
                captured_sat_sys[16] <= ch16_sat_sys;
                captured_freq_band[16] <= ch16_freq_band; 
                pending[16]        <= 1'b1;
            end

            // Channel 17
            if (ch17_valid) begin
                captured_ch[17]    <= 5'd17;
                captured_PRN[17]   <= ch17_PRN;
                captured_I[17]     <= ch17_I[32:16];
                captured_Q[17]     <= ch17_Q[32:16];
                captured_ABS[17]   <= abs_count;
                captured_freq[17]  <= ch17_freq;
                captured_phase[17] <= ch17_phase;
                captured_sat_sys[17] <= ch17_sat_sys;
                captured_freq_band[17] <= ch17_freq_band; 
                pending[17]        <= 1'b1;
            end

            // Channel 18
            if (ch18_valid) begin
                captured_ch[18]    <= 5'd18;
                captured_PRN[18]   <= ch18_PRN;
                captured_I[18]     <= ch18_I[32:16];
                captured_Q[18]     <= ch18_Q[32:16];
                captured_ABS[18]   <= abs_count;
                captured_freq[18]  <= ch18_freq;
                captured_phase[18] <= ch18_phase;
                captured_sat_sys[18] <= ch18_sat_sys;
                captured_freq_band[18] <= ch18_freq_band; 
                pending[18]        <= 1'b1;
            end

            // Channel 19
            if (ch19_valid) begin
                captured_ch[19]    <= 5'd19;
                captured_PRN[19]   <= ch19_PRN;
                captured_I[19]     <= ch19_I[32:16];
                captured_Q[19]     <= ch19_Q[32:16];
                captured_ABS[19]   <= abs_count;
                captured_freq[19]  <= ch19_freq;
                captured_phase[19] <= ch19_phase;
                captured_sat_sys[19] <= ch19_sat_sys;
                captured_freq_band[19] <= ch19_freq_band; 
                pending[19]        <= 1'b1;
            end

            // Channel 20
            if (ch20_valid) begin
                captured_ch[20]    <= 5'd20;
                captured_PRN[20]   <= ch20_PRN;
                captured_I[20]     <= ch20_I[32:16];
                captured_Q[20]     <= ch20_Q[32:16];
                captured_ABS[20]   <= abs_count;
                captured_freq[20]  <= ch20_freq;
                captured_phase[20] <= ch20_phase;
                captured_sat_sys[20] <= ch20_sat_sys;
                captured_freq_band[20] <= ch20_freq_band; 
                pending[20]        <= 1'b1;
            end

            // Channel 21
            if (ch21_valid) begin
                captured_ch[21]    <= 5'd21;
                captured_PRN[21]   <= ch21_PRN;
                captured_I[21]     <= ch21_I[32:16];
                captured_Q[21]     <= ch21_Q[32:16];
                captured_ABS[21]   <= abs_count;
                captured_freq[21]  <= ch21_freq;
                captured_phase[21] <= ch21_phase;
                captured_sat_sys[21] <= ch21_sat_sys;
                captured_freq_band[21] <= ch21_freq_band; 
                pending[21]        <= 1'b1;
            end

            // Channel 22
            if (ch22_valid) begin
                captured_ch[22]    <= 5'd22;
                captured_PRN[22]   <= ch22_PRN;
                captured_I[22]     <= ch22_I[32:16];
                captured_Q[22]     <= ch22_Q[32:16];
                captured_ABS[22]   <= abs_count;
                captured_freq[22]  <= ch22_freq;
                captured_phase[22] <= ch22_phase;
                captured_sat_sys[22] <= ch22_sat_sys;
                captured_freq_band[22] <= ch22_freq_band; 
                pending[22]        <= 1'b1;
            end

            // Channel 23
            if (ch23_valid) begin
                captured_ch[23]    <= 5'd23;
                captured_PRN[23]   <= ch23_PRN;
                captured_I[23]     <= ch23_I[32:16];
                captured_Q[23]     <= ch23_Q[32:16];
                captured_ABS[23]   <= abs_count;
                captured_freq[23]  <= ch23_freq;
                captured_phase[23] <= ch23_phase;
                captured_sat_sys[23] <= ch23_sat_sys;
                captured_freq_band[23] <= ch23_freq_band; 
                pending[23]        <= 1'b1;
            end

            // Channel 24
            if (ch24_valid) begin
                captured_ch[24]    <= 5'd24;
                captured_PRN[24]   <= ch24_PRN;
                captured_I[24]     <= ch24_I[32:16];
                captured_Q[24]     <= ch24_Q[32:16];
                captured_ABS[24]   <= abs_count;
                captured_freq[24]  <= ch24_freq;
                captured_phase[24] <= ch24_phase;
                captured_sat_sys[24] <= ch24_sat_sys;
                captured_freq_band[24] <= ch24_freq_band; 
                pending[24]        <= 1'b1;
            end
        end
    end

    // ---------------------------------------------------------
    // AXI-Stream Transmission FSM
    // ---------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state         <= IDLE;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            m_axis_tdata  <= 64'd0;

            tx_ch           <= 5'd0;
            tx_PRN          <= 8'd0;
            tx_I            <= 17'd0;
            tx_Q            <= 17'd0;
            tx_ABS          <= 32'd0;
            tx_freq         <= 32'd0;
            tx_phase        <= 64'd0;
            tx_sat_sys      <= 2'd0;
            tx_freq_band    <= 2'd0;
        end else begin
            case (state)
                //--------------------------------------------------
                IDLE: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;

                    if (any_pending) begin
                        // Latch selected channel data
                        tx_ch    <= next_ch;
                        tx_PRN   <= captured_PRN[next_ch];
                        tx_I     <= captured_I[next_ch];
                        tx_Q     <= captured_Q[next_ch];
                        tx_ABS   <= captured_ABS[next_ch];
                        tx_freq <= captured_freq[next_ch];
                        tx_phase <= captured_phase[next_ch];
                        tx_sat_sys <= captured_sat_sys[next_ch];
                        tx_freq_band <= captured_freq_band[next_ch];

                        // Word 0: [63:32]=ABS, [31:0]=CarrierFreq
                        m_axis_tdata  <= {captured_ABS[next_ch], captured_freq[next_ch]};
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= 1'b0;

                        state <= SEND_HDR;
                    end
                end

                //--------------------------------------------------
                SEND_HDR: begin
                    if (m_axis_tready) begin
                        // Word 1: [63:47]=I_P(17), [46:30]=Q_P(17), [29:17]=0, [16:15] sat_sys, [14:13] freq_band, [12:8]=CH(5), [7:0]=PRN
                        m_axis_tdata  <= {tx_I, tx_Q, 13'b0, tx_sat_sys, tx_freq_band, tx_ch, tx_PRN};
                        m_axis_tlast  <= 1'b0;
                        state <= SEND_IQ;
                    end
                end

                //--------------------------------------------------
                SEND_IQ: begin
                    if (m_axis_tready) begin
                        // Word 2: [63:0] = Carrier Phase
                        m_axis_tdata  <= tx_phase;
                        m_axis_tlast  <= 1'b1;
                        state <= SEND_PHASE;
                    end
                end

                //--------------------------------------------------
                SEND_PHASE: begin
                    if (m_axis_tready) begin
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                        state <= IDLE;
                    end
                end

                //--------------------------------------------------
                default: begin
                    state <= IDLE;
                    m_axis_tvalid <= 1'b0;
                end
            endcase
        end
    end

endmodule