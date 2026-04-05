// SPI Slave — FPGA receiving data from STM32
// SPI Mode 0: CPOL=0, CPHA=0
// No hardware CS — STM32 NSS is in software mode
// Byte framing: bit counter resets after a SCK idle gap
// Samples MOSI on SCK rising edge; drives MISO on SCK falling edge
// LED[7:0] shows the last fully received byte
//
// Connections:
//   SPI_SCK  <- STM32 SPI_SCK
//   SPI_MOSI <- STM32 SPI_MOSI
//   SPI_MISO -> STM32 SPI_MISO

`timescale 1ns / 1ps

module SPI_slave #(
    parameter CLK_FREQ   = 100_000_000, // FPGA system clock (Hz)
    parameter SPI_FREQ   =   1_000_000, // expected SPI clock (Hz) — used for idle timeout
    // Idle timeout: if SCK is quiet for IDLE_CYCLES system clocks, reset the bit counter.
    // Default = 4 SPI periods = 4 * (CLK_FREQ/SPI_FREQ) system clocks.
    parameter IDLE_CYCLES = 4 * (CLK_FREQ / SPI_FREQ)
)(
    input  wire       clk,      // FPGA system clock
    input  wire       rst,      // active-high synchronous reset

    // User interface
    output reg  [7:0] rx_data,  // last fully received byte
    output reg        rx_valid, // one clk pulse when rx_data is updated

    // Optional TX path (slave → master on MISO)
    input  wire [7:0] tx_data,  // byte to send back; latched on SCK idle → active

    // SPI bus (3-wire, no CS)
    input  wire       SPI_SCK,
    input  wire       SPI_MOSI,
    output reg        SPI_MISO,

    // Debug
    output wire [7:0] LED       // shows last received byte
);

    // ----------------------------------------------------------------
    // Double-flop synchronisers (metastability protection)
    // ----------------------------------------------------------------
    reg [1:0] sck_sync, mosi_sync;

    always @(posedge clk) begin
        if (rst) begin
            sck_sync  <= 2'b00;
            mosi_sync <= 2'b00;
        end else begin
            sck_sync  <= {sck_sync[0],  SPI_SCK};
            mosi_sync <= {mosi_sync[0], SPI_MOSI};
        end
    end

    wire sck_rose = (sck_sync == 2'b01);
    wire sck_fell = (sck_sync == 2'b10);
    wire mosi_d   = mosi_sync[1];

    // ----------------------------------------------------------------
    // SCK idle detector — counts system clocks since last SCK edge
    // Resets the bit counter so the next SCK rising edge is bit 7 (MSB)
    // ----------------------------------------------------------------
    integer idle_cnt;
    reg     sck_idle; // high while SCK has been quiet long enough

    always @(posedge clk) begin
        if (rst) begin
            idle_cnt <= 0;
            sck_idle <= 1;
        end else begin
            if (sck_rose || sck_fell) begin
                idle_cnt <= 0;
                sck_idle <= 0;
            end else if (idle_cnt < IDLE_CYCLES) begin
                idle_cnt <= idle_cnt + 1;
                sck_idle <= 0;
            end else begin
                sck_idle <= 1; // SCK has been quiet — ready for a new byte
            end
        end
    end

    // ----------------------------------------------------------------
    // Receive / transmit shift registers
    // ----------------------------------------------------------------
    reg [2:0] bit_cnt;
    reg [7:0] shift_rx;
    reg [7:0] shift_tx;

    assign LED = rx_data;

    always @(posedge clk) begin
        if (rst) begin
            bit_cnt  <= 3'd7;
            shift_rx <= 8'd0;
            shift_tx <= 8'd0;
            rx_data  <= 8'd0;
            rx_valid <= 1'b0;
            SPI_MISO <= 1'b0;
        end else begin
            rx_valid <= 0;

            // When SCK has been idle, pre-load the TX shift register and
            // drive MSB on MISO so it is ready before the first SCK edge.
            if (sck_idle) begin
                shift_tx <= tx_data;
                SPI_MISO <= tx_data[7];
                bit_cnt  <= 3'd7;
            end else begin
                // Sample MOSI on rising edge
                if (sck_rose) begin
                    shift_rx <= {shift_rx[6:0], mosi_d};
                    if (bit_cnt == 0) begin
                        rx_data  <= {shift_rx[6:0], mosi_d};
                        rx_valid <= 1'b1;
                        bit_cnt  <= 3'd7;
                    end else begin
                        bit_cnt <= bit_cnt - 1;
                    end
                end

                // Drive next MISO bit on falling edge
                if (sck_fell) begin
                    shift_tx <= {shift_tx[6:0], 1'b0};
                    SPI_MISO <= shift_tx[6];
                end
            end
        end
    end

endmodule
