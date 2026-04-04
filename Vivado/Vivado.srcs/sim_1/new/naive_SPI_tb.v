`timescale 1ns / 1ps

module naive_SPI_tb;

    // ----------------------------------------------------------------
    // DUT parameters — small divider so simulation runs fast
    // ----------------------------------------------------------------
    localparam CLK_FREQ = 100_000_000;
    localparam SPI_FREQ =   1_000_000;
    localparam CLK_PERIOD = 10; // 100 MHz -> 10 ns

    // ----------------------------------------------------------------
    // Signals
    // ----------------------------------------------------------------
    reg  clk, rst;
    wire SPI_CLK, SPI_MOSI, SPI_CS;
    wire [7:0] LED;

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    naive_SPI #(
        .CLK_FREQ (CLK_FREQ),
        .SPI_FREQ (SPI_FREQ),
        .LED_SHIFT(4)          // same default as module
    ) dut (
        .clk     (clk),
        .rst     (rst),
        .SPI_CLK (SPI_CLK),
        .SPI_MOSI(SPI_MOSI),
        .SPI_CS  (SPI_CS),
        .LED     (LED)
    );

    // ----------------------------------------------------------------
    // Clock
    // ----------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ----------------------------------------------------------------
    // Slave receiver — captures bits on SPI_CLK rising edge
    // ----------------------------------------------------------------
    integer  bit_idx;
    reg [7:0] rx_byte;
    reg [7:0] expected;
    integer  byte_count;
    integer  errors;

    initial begin
        expected   = 8'd0;
        byte_count = 0;
        errors     = 0;
    end

    // Detect CS falling edge -> prepare to receive
    reg cs_prev;
    always @(posedge clk) cs_prev <= SPI_CS;

    // Sample MOSI on every SPI_CLK rising edge while CS is low
    reg spi_clk_prev;
    always @(posedge clk) begin
        spi_clk_prev <= SPI_CLK;

        if (!SPI_CS && SPI_CLK && !spi_clk_prev) begin
            // Rising edge of SPI_CLK with CS asserted
            rx_byte <= {rx_byte[6:0], SPI_MOSI};
        end

        // CS rising edge -> byte complete, check it
        if (SPI_CS && !cs_prev) begin
            if (byte_count > 0) begin // skip first partial transaction if any
                if (rx_byte !== expected) begin
                    $display("FAIL byte %0d: got 0x%02X, expected 0x%02X  LED=0x%02X",
                             byte_count, rx_byte, expected, LED);
                    errors = errors + 1;
                end else begin
                    $display("OK   byte %0d: 0x%02X  LED=0x%02X", byte_count, rx_byte, LED);
                end
                expected = expected + 1;
            end
            byte_count = byte_count + 1;
        end
    end

    // ----------------------------------------------------------------
    // Stimulus + termination
    // ----------------------------------------------------------------
    integer NUM_BYTES;
    initial NUM_BYTES = 8; // capture this many complete bytes

    initial begin
        $dumpfile("naive_SPI_tb.vcd");
        $dumpvars(0, naive_SPI_tb);

        // Reset
        rst = 1;
        repeat (4) @(posedge clk);
        rst = 0;

        // Wait for NUM_BYTES complete bytes (each byte = ~10 SPI clocks at 1 MHz
        // -> 10 us; plus gap. At 100 MHz sys clk that's ~1000 sys cycles/byte)
        // We wait generously.
        repeat (NUM_BYTES * (CLK_FREQ / SPI_FREQ) * 15) @(posedge clk);

        // Report
        if (errors == 0)
            $display("PASSED: all %0d bytes matched.", byte_count - 1);
        else
            $display("FAILED: %0d error(s) in %0d bytes.", errors, byte_count - 1);

        $finish;
    end

endmodule
