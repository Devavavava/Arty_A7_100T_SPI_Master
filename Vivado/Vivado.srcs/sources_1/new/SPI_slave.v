// SPI Slave — FPGA receiving data from STM32
// SPI Mode 0: CPOL=0, CPHA=0
// Samples MOSI on SCK rising edge; drives MISO on SCK falling edge
// LED[7:0] shows the last fully received byte
//
// Connections:
//   SPI_SCK  <- STM32 SPI_SCK
//   SPI_MOSI <- STM32 SPI_MOSI
//   SPI_MISO -> STM32 SPI_MISO  (not driven in RX-only use; tie to 0)
//   SPI_CS   <- STM32 SPI_NSS   (active-low)

`timescale 1ns / 1ps

module SPI_slave (
    input  wire       clk,      // FPGA system clock (for synchronisation)
    input  wire       rst,      // active-high synchronous reset

    // User interface
    output reg  [7:0] rx_data,  // last fully received byte
    output reg        rx_valid, // one clk pulse when rx_data is updated

    // SPI bus (from master / STM32)
    input  wire       SPI_SCK,
    input  wire       SPI_MOSI,
    output reg        SPI_MISO, // MISO driven from tx_data (set before CS falls)
    input  wire       SPI_CS,   // active-low chip select

    // Optional TX path (slave → master)
    input  wire [7:0] tx_data,  // byte to send back to master on MISO

    // Debug
    output wire [7:0] LED       // shows last received byte
);

    // ----------------------------------------------------------------
    // Synchronise async SPI signals into the FPGA clock domain
    // ----------------------------------------------------------------
    reg [1:0] sck_sync, mosi_sync, cs_sync;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sck_sync  <= 2'b00;
            mosi_sync <= 2'b00;
            cs_sync   <= 2'b11; // CS idle = high
        end else begin
            sck_sync  <= {sck_sync[0],  SPI_SCK};
            mosi_sync <= {mosi_sync[0], SPI_MOSI};
            cs_sync   <= {cs_sync[0],   SPI_CS};
        end
    end

    wire sck_rose  = (sck_sync == 2'b01); // rising  edge of SCK
    wire sck_fell  = (sck_sync == 2'b10); // falling edge of SCK
    wire cs_active = ~cs_sync[1];         // CS is low (active)
    wire cs_fell   = (cs_sync == 2'b10);  // CS just went low  (start)
    wire cs_rose   = (cs_sync == 2'b01);  // CS just went high (end)
    wire mosi_d    = mosi_sync[1];        // stable MOSI sample

    // ----------------------------------------------------------------
    // Receive shift register
    // ----------------------------------------------------------------
    reg [2:0] bit_cnt;
    reg [7:0] shift_rx;
    reg [7:0] shift_tx; // pre-loaded MISO shift register

    assign LED = rx_data;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_cnt  <= 3'd7;
            shift_rx <= 8'd0;
            shift_tx <= 8'd0;
            rx_data  <= 8'd0;
            rx_valid <= 1'b0;
            SPI_MISO <= 1'b0;
        end else begin
            rx_valid <= 0; // default

            // Pre-load TX shift register when CS falls (transfer start)
            if (cs_fell) begin
                shift_tx <= tx_data;
                bit_cnt  <= 3'd7;
                SPI_MISO <= tx_data[7]; // drive MSB before first SCK edge
            end

            if (cs_active) begin
                // Mode 0: sample MOSI on rising edge
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

                // Drive MISO on falling edge (next bit ready for master to sample)
                if (sck_fell) begin
                    shift_tx <= {shift_tx[6:0], 1'b0};
                    SPI_MISO <= shift_tx[6]; // next bit (current MSB already driven)
                end
            end

            // Release MISO when CS deasserts
            if (cs_rose) begin
                SPI_MISO <= 1'b0;
            end
        end
    end

endmodule
