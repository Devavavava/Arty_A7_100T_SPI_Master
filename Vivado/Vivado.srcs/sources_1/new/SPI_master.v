// SPI Master — FPGA to STM32
// SPI Mode 0: CPOL=0, CPHA=0
// Sends tx_data over MOSI; samples MISO for rx_data
// LED[7:0] shows the last transmitted byte
//
// Connections:
//   SPI_CLK  -> STM32 SPI_SCK
//   SPI_MOSI -> STM32 SPI_MOSI
//   SPI_MISO <- STM32 SPI_MISO
//   SPI_CS   -> STM32 SPI_NSS  (active-low)

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

    // SPI bus
    output reg        SPI_CLK,
    output reg        SPI_MOSI,
    input  wire       SPI_MISO,
    output reg        SPI_CS,    // active-low chip select

    // Debug
    output wire [7:0] LED        // shows last transmitted byte
);

    // ----------------------------------------------------------------
    // Clock divider
    // ----------------------------------------------------------------
    localparam HALF_PERIOD = CLK_FREQ / (2 * SPI_FREQ);

    integer clk_cnt;
    reg     spi_rise; // one-cycle pulse on SPI_CLK rising edge
    reg     spi_fall; // one-cycle pulse on SPI_CLK falling edge

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
                spi_rise <=  ~SPI_CLK; // will go high → rising edge
                spi_fall <=   SPI_CLK; // will go low  → falling edge
            end else begin
                clk_cnt <= clk_cnt + 1;
            end
        end
    end

    // ----------------------------------------------------------------
    // State machine
    // ----------------------------------------------------------------
    localparam IDLE     = 2'd0,
               CS_SETUP = 2'd1,
               SHIFTING = 2'd2,
               CS_HOLD  = 2'd3;

    reg [1:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] shift_tx;
    reg [7:0] shift_rx;
    reg [7:0] tx_byte_latch; // captured tx_data at start of transfer
    reg [1:0] gap_cnt;

    // LED shows the last byte that was loaded for transmission
    reg [7:0] last_tx;
    assign LED = last_tx;

    always @(posedge clk) begin
        if (rst) begin
            state        <= IDLE;
            SPI_CS       <= 1;
            SPI_MOSI     <= 0;
            bit_cnt      <= 7;
            shift_tx     <= 0;
            shift_rx     <= 0;
            rx_data      <= 0;
            rx_valid     <= 0;
            busy         <= 0;
            gap_cnt      <= 0;
            last_tx      <= 0;
            tx_byte_latch<= 0;
        end else begin
            rx_valid <= 0; // default

            case (state)

                IDLE: begin
                    SPI_CS   <= 1;
                    SPI_MOSI <= 0;
                    busy     <= 0;
                    if (tx_valid) begin
                        tx_byte_latch <= tx_data;
                        last_tx       <= tx_data;
                        busy          <= 1;
                        // Wait for a rising edge to align to SPI clock
                        state <= CS_SETUP;
                    end
                end

                // Pull CS low, wait one SPI half-period for setup time
                CS_SETUP: begin
                    busy   <= 1;
                    SPI_CS <= 0;
                    if (spi_rise) begin
                        shift_tx <= tx_byte_latch;
                        bit_cnt  <= 7;
                        state    <= SHIFTING;
                    end
                end

                // Mode 0: drive MOSI on falling edge, sample MISO on rising edge
                SHIFTING: begin
                    if (spi_fall) begin
                        // Drive next bit (MSB first)
                        SPI_MOSI <= shift_tx[7];
                        shift_tx <= {shift_tx[6:0], 1'b0};
                    end
                    if (spi_rise) begin
                        // Sample MISO
                        shift_rx <= {shift_rx[6:0], SPI_MISO};
                        if (bit_cnt == 0) begin
                            // All 8 bits done — first falling edge after this
                            // will be suppressed by moving to CS_HOLD
                            SPI_CS   <= 1;
                            SPI_MOSI <= 0;
                            rx_data  <= {shift_rx[6:0], SPI_MISO};
                            rx_valid <= 1;
                            gap_cnt  <= 2'd3;
                            state    <= CS_HOLD;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                // Brief CS-high gap between back-to-back transfers
                CS_HOLD: begin
                    SPI_CS <= 1;
                    busy   <= 0;
                    if (spi_rise) begin
                        if (gap_cnt == 0) begin
                            state <= IDLE;
                        end else begin
                            gap_cnt <= gap_cnt - 1;
                        end
                    end
                end

            endcase
        end
    end

endmodule
