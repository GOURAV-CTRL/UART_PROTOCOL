// -----------------------------------------------------------------------------
// uart_bd : shared baud-rate generator (TX bd + RX bd in one module)
//
//   tx_tick : 1-clock pulse once per bit time           (= BAUD_RATE)
//   rx_tick : 1-clock pulse OVERSAMPLE times per bit    (= BAUD_RATE * OVERSAMPLE)
//
// Both counters free-run; uart_tx and uart_rx only listen to their own tick.
//
//   TX_DIV = round(CLK_FREQ_HZ / BAUD_RATE)
//   RX_DIV = round(CLK_FREQ_HZ / (BAUD_RATE * OVERSAMPLE))
//
//   100 MHz, 57600 : TX_DIV = 1736 (0.01 % err), RX_DIV = 109 (0.5 % err, harmless
//                    because the receiver re-syncs on every start bit)
//--------------------------------------
module uart_bd #(
    parameter CLK_FREQ_HZ = 100_000_000,
    parameter BAUD_RATE   = 57_600,
    parameter OVERSAMPLE  = 16
)(
    input  wire clk,
    input  wire rst,          // sync, active high
    output reg  tx_tick,      // -> uart_tx
    output reg  rx_tick       // -> uart_rx
);

    localparam integer TX_DIV = (CLK_FREQ_HZ ) / BAUD_RATE;
    localparam integer RX_DIV = (CLK_FREQ_HZ ) / (BAUD_RATE*OVERSAMPLE);
    localparam integer TX_W   = $clog2(TX_DIV);
    localparam integer RX_W   = $clog2(RX_DIV);

    reg [TX_W-1:0] tx_cnt;
    reg [RX_W-1:0] rx_cnt;

    // ---------------- TX bd : one tick per bit ----------------
    always @(posedge clk) begin
        if (rst) begin
            tx_cnt  <= {TX_W{1'b0}};
            tx_tick <= 1'b0;
        end else if (tx_cnt == TX_DIV - 1) begin
            tx_cnt  <= {TX_W{1'b0}};
            tx_tick <= 1'b1;
        end else begin
            tx_cnt  <= tx_cnt + 1'b1;
            tx_tick <= 1'b0;
        end
    end

    // ---------------- RX bd : OVERSAMPLE ticks per bit ----------------
    always @(posedge clk) begin
        if (rst) begin
            rx_cnt  <= {RX_W{1'b0}};
            rx_tick <= 1'b0;
        end else if (rx_cnt == RX_DIV - 1) begin
            rx_cnt  <= {RX_W{1'b0}};
            rx_tick <= 1'b1;
        end else begin
            rx_cnt  <= rx_cnt + 1'b1;
            rx_tick <= 1'b0;
        end
    end

endmodule
