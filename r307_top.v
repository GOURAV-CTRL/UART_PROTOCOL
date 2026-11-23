// -----------------------------------------------------------------------------
// r307_top : r307_cmd_layer + uart_bd + uart_tx + uart_rx

module r307_top #(
    parameter CLK_FREQ_HZ  = 100_000_000,
    parameter BAUD_RATE    = 57_600,          // R307 default
    parameter OVERSAMPLE   = 16,
    parameter TIMEOUT_CLKS = 50_000_000       // ack timeout, 500 ms at 100 MHz
)(
    input  wire        clk,
    input  wire        rst,                   // sync, active high

    // user side (command layer)
    input  wire        start,                 // 1-clock pulse: run the 5-step sequence
    output wire        busy,
    output wire        done,                  // 1-clock pulse, sequence finished
    output wire        seq_ok,                // 1 = all 5 steps got code 00
    output wire        ack_ok,
    output wire        ack_timeout,
    output wire [7:0]  ack_code,
    output wire [15:0] ack_page_id,
    output wire [15:0] ack_score,
    output wire [2:0]  step,

    // serial pins to the R307 module
    output wire        tx,                    // -> R307 RXD
    input  wire        rx                     // <- R307 TXD
);

    // ---------------- internal wires ----------------
    wire [7:0] tx_data;
    wire       tx_start, tx_busy, tx_done;
    wire [7:0] rx_data;
    wire       rx_valid, frame_err;
    wire       tx_tick, rx_tick;

    // ---------------- command layer ----------------
    r307_cmd_layer #(
        .TIMEOUT_CLKS (TIMEOUT_CLKS)
    ) u_cmd (
        .clk         (clk),
        .rst         (rst),
        .start       (start),
        .busy        (busy),
        .done        (done),
        .seq_ok      (seq_ok),
        .ack_ok      (ack_ok),
        .ack_timeout (ack_timeout),
        .ack_code    (ack_code),
        .ack_page_id (ack_page_id),
        .ack_score   (ack_score),
        .step        (step),
        .tx_data     (tx_data),
        .tx_start    (tx_start),
        .tx_busy     (tx_busy),
        .tx_done     (tx_done),
        .rx_data     (rx_data),
        .rx_valid    (rx_valid),
        .frame_err   (frame_err)
    );

    // ---------------- baud-rate generator (shared) ----------------
    uart_bd #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .OVERSAMPLE  (OVERSAMPLE)
    ) u_bd (
        .clk     (clk),
        .rst     (rst),
        .tx_tick (tx_tick),
        .rx_tick (rx_tick)
    );

    // ---------------- UART transmitter ----------------
    uart_tx #(
        .DATA_BITS (8),
        .STOP_BITS (1)
    ) u_tx (
        .clk      (clk),
        .rst      (rst),
        .tx_tick  (tx_tick),
        .tx_data  (tx_data),
        .tx_start (tx_start),
        .tx_busy  (tx_busy),
        .tx_done  (tx_done),
        .tx       (tx)
    );

    // ---------------- UART receiver ----------------
    uart_rx #(
        .DATA_BITS  (8),
        .OVERSAMPLE (OVERSAMPLE)
    ) u_rx (
        .clk       (clk),
        .rst       (rst),
        .rx_tick   (rx_tick),
        .rx        (rx),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .frame_err (frame_err)
    );

endmodule
