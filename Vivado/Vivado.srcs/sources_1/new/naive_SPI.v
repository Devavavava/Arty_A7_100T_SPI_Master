// SPI Master - FPGA to RPi (TX only)
// SPI Mode 0: CPOL=0, CPHA=0
// Sends a continuously incrementing 8-bit counter over MOSI
//
// Connections:
//   SPI_CLK  -> RPi GPIO 11 (SCLK)
//   SPI_MOSI -> RPi GPIO 10 (MOSI)
//   SPI_CS   -> RPi GPIO  8 (CE0)
//   LED[7:0] -> Arty A7 LD7..LD0 (shows tx_byte >> LED_SHIFT)

module naive_SPI #(
    parameter CLK_FREQ  = 100_000_000, // FPGA system clock (Hz)
    parameter SPI_FREQ  =   1_000_000, // SPI clock (Hz)
    parameter LED_SHIFT =            8 // divide tx_byte by 2^LED_SHIFT for LED rate
)(
    input  wire clk,        // system clock
    input  wire rst,        // active-high synchronous reset

    output reg  SPI_CLK,
    output reg  SPI_MOSI,
    output reg  SPI_CS,     // active-low chip select
    output wire [7:0] LED   // debug: tx_byte >> LED_SHIFT, wraps every 256<<LED_SHIFT bytes
);

    // ----------------------------------------------------------------
    // Clock divider: toggle SPI_CLK every HALF_PERIOD system clocks
    // ----------------------------------------------------------------
    localparam HALF_PERIOD = CLK_FREQ / (2 * SPI_FREQ); // 50 for 1 MHz on 100 MHz

    integer clk_cnt;
    reg     spi_rise; // one-cycle pulse on every SPI_CLK rising edge

    always @(posedge clk) begin
        if (rst) begin
            clk_cnt  <= 0;
            SPI_CLK  <= 0;
            spi_rise <= 0;
        end else begin
            spi_rise <= 0;
            if (clk_cnt == HALF_PERIOD - 1) begin
                clk_cnt <= 0;
                SPI_CLK <= ~SPI_CLK;
                spi_rise <= ~SPI_CLK; // high when SPI_CLK is about to go high
            end else begin
                clk_cnt <= clk_cnt + 1;
            end
        end
    end

    // ----------------------------------------------------------------
    // TX state machine — advances on SPI rising edges
    // ----------------------------------------------------------------
    localparam IDLE     = 2'd0,
               CS_SETUP = 2'd1,
               SHIFTING = 2'd2,
               CS_HOLD  = 2'd3;

    reg [1:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] shift_reg;
    reg [7:0] tx_byte;        // increments after each byte
    reg [1:0] gap_cnt;

    // LED: show upper bits of a wider counter so the display scrolls slowly
    reg [7+LED_SHIFT:0] led_counter; // wide enough to hold tx_byte << LED_SHIFT
    assign LED = led_counter[7+LED_SHIFT -: 8]; // top 8 bits = tx_byte >> LED_SHIFT

    always @(posedge clk) begin
        if (rst) begin
            state       <= IDLE;
            SPI_CS      <= 1;
            SPI_MOSI    <= 0;
            bit_cnt     <= 7;
            shift_reg   <= 8'd0;
            tx_byte     <= 8'd0;
            gap_cnt     <= 0;
            led_counter <= 0;
        end else begin
            case (state)

                // Wait for first rising edge, then pull CS low
                IDLE: begin
                    SPI_CS   <= 1;
                    SPI_MOSI <= 0;
                    if (spi_rise) begin
                        SPI_CS    <= 0;
                        shift_reg <= tx_byte;
                        bit_cnt   <= 7;
                        state     <= CS_SETUP;
                    end
                end

                // One full SPI cycle with CS low before data — setup time
                CS_SETUP: begin
                    if (spi_rise) begin
                        SPI_MOSI <= shift_reg[7]; // MSB first
                        state    <= SHIFTING;
                    end
                end

                // Shift out 8 bits, MSB first, on each rising edge
                SHIFTING: begin
                    if (spi_rise) begin
                        if (bit_cnt == 0) begin
                            // All bits sent
                            SPI_CS      <= 1;
                            SPI_MOSI    <= 0;
                            tx_byte     <= tx_byte + 1;
                            led_counter <= led_counter + 1;
                            gap_cnt     <= 2'd3;
                            state       <= CS_HOLD;
                        end else begin
                            shift_reg <= {shift_reg[6:0], 1'b0};
                            SPI_MOSI  <= shift_reg[6];
                            bit_cnt   <= bit_cnt - 1;
                        end
                    end
                end

                // Short gap between bytes (CS high)
                CS_HOLD: begin
                    if (spi_rise) begin
                        if (gap_cnt == 0) begin
                            SPI_CS    <= 0;
                            shift_reg <= tx_byte;
                            bit_cnt   <= 7;
                            state     <= CS_SETUP;
                        end else begin
                            gap_cnt <= gap_cnt - 1;
                        end
                    end
                end

            endcase
        end
    end

endmodule
