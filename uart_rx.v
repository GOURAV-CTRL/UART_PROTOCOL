// -----------------------------------------------------------------------------
// uart_rx : UART receiver (8N1 by default, LSB first), oversampling
//
// Timing comes from outside: rx_tick (OVERSAMPLE pulses per bit) from uart_bd.
//   rx_data   : received byte, valid while rx_valid = 1
//   rx_valid  : 1-clock pulse per frame (also pulsed on a framing error)
//   frame_err : 1-clock pulse together with rx_valid when the stop bit was 0
//
// After the start edge it counts OVERSAMPLE/2 ticks to the middle of the start
// bit (rejects glitches), then OVERSAMPLE ticks per bit to sample each bit centre.
// rx goes through a 2-flop synchronizer. DATA_BITS must be >= 2.
// -----------------------------------------------------------------------------
module uart_rx #(
    parameter DATA_BITS  = 8,
    parameter OVERSAMPLE = 16
)(
    input  wire                 clk,
    input  wire                 rst,        // sync, active high
    input  wire                 rx_tick,    // from uart_bd
    input  wire                 rx,         // serial in (idles high)
    output reg  [DATA_BITS-1:0] rx_data,
    output reg                  rx_valid,
    output reg                  frame_err
);

  

    // ---------------- FSM ----------------
    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0]           state;
    reg [DATA_BITS-1:0] shreg;
    integer             bit_idx;
    integer             os_cnt;                 // ticks inside the current bit

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            shreg     <= {DATA_BITS{1'b0}};
            bit_idx   <= 0;
            os_cnt    <= 0;
            rx_data   <= {DATA_BITS{1'b0}};
            rx_valid  <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            rx_valid  <= 1'b0;                  // one-clock pulses
            frame_err <= 1'b0;

            case (state)
            S_IDLE: 
                os_cnt <= 0;
                state  <= S_START;
            end

            S_START: if (rx_tick) begin
                if (os_cnt == OVERSAMPLE/2 - 1) begin   // middle of start bit
                    if (!rx) begin
                        os_cnt  <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else begin
                        state <= S_IDLE;                // glitch, ignore
                    end
                end else os_cnt <= os_cnt + 1;
            end

            S_DATA: if (rx_tick) begin
                if (os_cnt == OVERSAMPLE - 1) begin     // middle of data bit
                    os_cnt <= 0;
                    shreg  <= {rx_s, shreg[DATA_BITS-1:1]};   // LSB first
                    if (bit_idx == DATA_BITS - 1) state <= S_STOP;
                    else                          bit_idx <= bit_idx + 1;
                end else os_cnt <= os_cnt + 1;
            end

            S_STOP: if (rx_tick) begin
                if (os_cnt == OVERSAMPLE - 1) begin     // middle of stop bit
                    rx_data   <= shreg;
                    rx_valid  <= 1'b1;
                    frame_err <= ~rx;                 // stop bit must be 1
                    state     <= S_IDLE;
                end else os_cnt <= os_cnt + 1;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
