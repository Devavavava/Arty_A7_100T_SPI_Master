// SPI Master — FPGA to STM32
// SPI Mode 0: CPOL=0, CPHA=0
// No hardware CS — STM32 NSS is in software mode
// Sends tx_data over MOSI; samples MISO for rx_data
// LED[7:0] shows the last transmitted byte
//
// Connections:
//   SPI_CLK  -> STM32 SPI_SCK
//   SPI_MOSI -> STM32 SPI_MOSI
//   SPI_MISO <- STM32 SPI_MISO

`timescale 1ns / 1ps

module SPI_master #(
    parameter CLK_FREQ  = 100_000_000, // FPGA system clock (Hz)
    parameter SPI_FREQ  =   1_000_000  // SPI clock (Hz)
)(
    input  wire       clk,       // system clock
    input  wire       rst,       // active-high synchronous reset

    // User interface
    input  wire [7:0] tx_data,   // byte to transmit
    input  wire       tx_valid,  // pulse high for one clk to start a transfer
    output reg  [7:0] rx_data,   // byte received from slave
    output reg        rx_valid,  // high for one clk when rx_data is valid
    output reg        busy,      // high while a transfer is in progress

    // SPI bus (3-wire, no CS)
    output reg        SPI_CLK,
    output reg        SPI_MOSI,
    input  wire       SPI_MISO,

    // Debug
    output wire [7:0] LED        // shows last transmitted byte
);

    // ----------------------------------------------------------------
    // Clock divider
    // ----------------------------------------------------------------
    localparam HALF_PERIOD = CLK_FREQ / (2 * SPI_FREQ);

    integer clk_cnt;
    reg spi_rise; // one-cycle pulse on SPI_CLK rising edge
    reg spi_fall; // one-cycle pulse on SPI_CLK falling edge

    always @(posedge clk) begin
        if (rst) begin
            clk_cnt  <= 0;
            SPI_CLK  <= 0;
            spi_rise <= 0;
            spi_fall <= 0;
        end else begin
            spi_rise <= 0;
            spi_fall <= 0;
            if (clk_cnt == HALF_PERIOD - 1) begin
                clk_cnt  <= 0;
                SPI_CLK  <= ~SPI_CLK;
                spi_rise <= ~SPI_CLK; // high when SCK is about to go high
                spi_fall <=  SPI_CLK; // high when SCK is about to go low
            end else begin
                clk_cnt <= clk_cnt + 1;
            end
        end
    end

    // ----------------------------------------------------------------
    // State machine — IDLE → SHIFTING → DONE gap → IDLE
    // ----------------------------------------------------------------
    localparam IDLE     = 2'd0,
               SHIFTING = 2'd1,
               GAP      = 2'd2;

    reg [1:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] shift_tx;
    reg [7:0] shift_rx;
    reg [7:0] tx_byte_latch;
    reg [2:0] gap_cnt;

    reg [7:0] last_tx;
    assign LED = last_tx;

    always @(posedge clk) begin
        if (rst) begin
            state         <= IDLE;
            SPI_CLK       <= 0;
            SPI_MOSI      <= 0;
            bit_cnt       <= 7;
            shift_tx      <= 0;
            shift_rx      <= 0;
            rx_data       <= 0;
            rx_valid      <= 0;
            busy          <= 0;
            gap_cnt       <= 0;
            last_tx       <= 0;
            tx_byte_latch <= 0;
            clk_cnt       <= 0;
            spi_rise      <= 0;
            spi_fall      <= 0;
        end else begin
            rx_valid <= 0;

            case (state)

                IDLE: begin
                    SPI_MOSI <= 0;
                    busy     <= 0;
                    if (tx_valid) begin
                        tx_byte_latch <= tx_data;
                        last_tx       <= tx_data;
                        busy          <= 1;
                        // Pre-load shift reg and drive MSB before first SCK edge
                        shift_tx <= tx_data;
                        SPI_MOSI <= tx_data[7];
                        bit_cnt  <= 7;
                        state    <= SHIFTING;
                    end
                end

                // Mode 0: drive MOSI on falling edge, sample MISO on rising edge
                SHIFTING: begin
                    busy <= 1;
                    if (spi_fall) begin
                        // Shift out next bit
                        shift_tx <= {shift_tx[6:0], 1'b0};
                        SPI_MOSI <= shift_tx[6]; // bit that will be valid after the shift
                    end
                    if (spi_rise) begin
                        shift_rx <= {shift_rx[6:0], SPI_MISO};
                        if (bit_cnt == 0) begin
                            rx_data  <= {shift_rx[6:0], SPI_MISO};
                            rx_valid <= 1;
                            SPI_MOSI <= 0;
                            gap_cnt  <= 3'd4; // a few SCK cycles idle before next byte
                            state    <= GAP;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                // Short idle gap — SCK keeps running but MOSI is 0
                GAP: begin
                    busy <= 0;
                    if (spi_rise) begin
                        if (gap_cnt == 0)
                            state <= IDLE;
                        else
                            gap_cnt <= gap_cnt - 1;
                    end
                end

            endcase
        end
    end

endmodule
