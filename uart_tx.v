// -----------------------------------------------------------------------------
// uart_tx : UART transmitter (8N1 by default, LSB first)
//
// Timing comes from outside: tx_tick (1 pulse per bit) from uart_bd.
//   tx_start : 1-clock pulse, tx_data captured on that clock (ignored if busy)
//   tx_busy  : high from the clock after tx_start until the stop bit has ended
//   tx_done  : 1-clock pulse, same clock that tx_busy falls
// The start bit begins on the first tx_tick after tx_start (<= 1 bit of latency).
// -----------------------------------------------------------------------------
module uart_tx #(
    parameter DATA_BITS = 8,
    parameter STOP_BITS = 1
)(
    input  wire                 clk,
    input  wire                 rst,        // sync, active high
    input  wire                 tx_tick,    // from uart_bd
    input  wire [DATA_BITS-1:0] tx_data,
    input  wire                 tx_start,
    output reg                  tx_busy,
    output reg                  tx_done,
    output reg                  tx
);

    localparam [2:0] S_IDLE  = 3'd0,
                     S_SYNC  = 3'd1,    // wait for the next tick, then start bit
                     S_START = 3'd2,
                     S_DATA  = 3'd3,
                     S_STOP  = 3'd4;

    reg [2:0]           state;
    reg [DATA_BITS-1:0] shreg;
    integer             bit_idx;
    integer             stop_idx;

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            tx       <= 1'b1;
            tx_busy  <= 1'b0;
            tx_done  <= 1'b0;
            shreg    <= {DATA_BITS{1'b0}};
            bit_idx  <= 0;
            stop_idx <= 0;
        end else begin
            tx_done <= 1'b0;                       // one-clock pulse

            case (state)
            S_IDLE: if (tx_start) begin
                shreg   <= tx_data;
                tx_busy <= 1'b1;
                state   <= S_SYNC;
            end

            S_SYNC: if (tx_tick) begin
                tx    <= 1'b0;                     // start bit begins
                state <= S_START;
            end

            S_START: if (tx_tick) begin            // start bit finished
                tx      <= shreg[0];               // data bit 0
                shreg   <= shreg >> 1;
                bit_idx <= 0;
                state   <= S_DATA;
            end

            S_DATA: if (tx_tick) begin             // data bit [bit_idx] finished
                if (bit_idx == DATA_BITS - 1) begin
                    tx       <= 1'b1;              // stop bit
                    stop_idx <= 0;
                    state    <= S_STOP;
                end else begin
                    tx      <= shreg[0];
                    shreg   <= shreg >> 1;
                    bit_idx <= bit_idx + 1;
                end
            end

            S_STOP: if (tx_tick) begin
                if (stop_idx == STOP_BITS - 1) begin
                    tx_busy <= 1'b0;
                    tx_done <= 1'b1;
                    state   <= S_IDLE;
                end else begin
                    stop_idx <= stop_idx + 1;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
