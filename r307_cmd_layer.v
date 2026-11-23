// -----------------------------------------------------------------------------
// R307 command layer - simple FSM
//
// On a 'start' pulse it runs this fixed sequence, one command + one ack each:
//   step 0 : VfyPwd            -> ack
//   step 1 : Img2Tz (buffer 1) -> ack
//   step 2 : RegModel          -> ack
//   step 3 : Store (buf1, page 1) -> ack
//   step 4 : TemplateNum       -> ack (count appears on ack_page_id)
//
// Stops early (seq_ok = 0) on timeout, malformed ack or ack_code != 00.
// Talks to uart_tx (tx_data/tx_start/tx_busy/tx_done) and
// uart_rx (rx_data/rx_valid/frame_err) as in the spec block diagram.
// -----------------------------------------------------------------------------
module r307_cmd_layer #(
    parameter TIMEOUT_CLKS = 50_000_000      // 500 ms at 100 MHz
)(
    input  wire        clk,
    input  wire        rst,                  // sync, active high

    // user side
    input  wire        start,                // 1-clock pulse
    output reg         busy,
    output reg         done,                 // 1-clock pulse, sequence finished
    output reg         seq_ok,               // 1 = all 5 steps got code 00
    output reg         ack_ok,               // last ack was well formed
    output reg         ack_timeout,          // last ack timed out
    output reg  [7:0]  ack_code,
    output reg  [15:0] ack_page_id,
    output reg  [15:0] ack_score,
    output reg  [2:0]  step,                 // current step (0..4)

    // uart_tx
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    input  wire        tx_busy,
    input  wire        tx_done,

    // uart_rx
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    input  wire        frame_err
);

    // ---------------- packet ROM ----------------
    function [4:0] pkt_len;                  // bytes in the command packet
        input [2:0] s;
        case (s)
            3'd0:    pkt_len = 5'd16;        // VfyPwd
            3'd1:    pkt_len = 5'd13;        // Img2Tz buf1
            3'd2:    pkt_len = 5'd12;        // RegModel
            3'd3:    pkt_len = 5'd15;        // Store
            default: pkt_len = 5'd12;        // TemplateNum
        endcase
    endfunction

    function [7:0] pkt_byte;
        input [2:0] s;
        input [4:0] i;
        case (s)
            3'd0: case (i)   // EF01 FFFFFFFF 01 0007 13 00000000 001B
                0:pkt_byte=8'hEF; 1:pkt_byte=8'h01; 2:pkt_byte=8'hFF; 3:pkt_byte=8'hFF;
                4:pkt_byte=8'hFF; 5:pkt_byte=8'hFF; 6:pkt_byte=8'h01; 7:pkt_byte=8'h00;
                8:pkt_byte=8'h07; 9:pkt_byte=8'h13; 10:pkt_byte=8'h00; 11:pkt_byte=8'h00;
                12:pkt_byte=8'h00; 13:pkt_byte=8'h00; 14:pkt_byte=8'h00; default:pkt_byte=8'h1B;
            endcase
            3'd1: case (i)   // EF01 FFFFFFFF 01 0004 02 01 0008
                0:pkt_byte=8'hEF; 1:pkt_byte=8'h01; 2:pkt_byte=8'hFF; 3:pkt_byte=8'hFF;
                4:pkt_byte=8'hFF; 5:pkt_byte=8'hFF; 6:pkt_byte=8'h01; 7:pkt_byte=8'h00;
                8:pkt_byte=8'h04; 9:pkt_byte=8'h02; 10:pkt_byte=8'h01; 11:pkt_byte=8'h00;
                default:pkt_byte=8'h08;
            endcase
            3'd2: case (i)   // EF01 FFFFFFFF 01 0003 05 0009
                0:pkt_byte=8'hEF; 1:pkt_byte=8'h01; 2:pkt_byte=8'hFF; 3:pkt_byte=8'hFF;
                4:pkt_byte=8'hFF; 5:pkt_byte=8'hFF; 6:pkt_byte=8'h01; 7:pkt_byte=8'h00;
                8:pkt_byte=8'h03; 9:pkt_byte=8'h05; 10:pkt_byte=8'h00; default:pkt_byte=8'h09;
            endcase
            3'd3: case (i)   // EF01 FFFFFFFF 01 0006 06 01 0001 000F
                0:pkt_byte=8'hEF; 1:pkt_byte=8'h01; 2:pkt_byte=8'hFF; 3:pkt_byte=8'hFF;
                4:pkt_byte=8'hFF; 5:pkt_byte=8'hFF; 6:pkt_byte=8'h01; 7:pkt_byte=8'h00;
                8:pkt_byte=8'h06; 9:pkt_byte=8'h06; 10:pkt_byte=8'h01; 11:pkt_byte=8'h00;
                12:pkt_byte=8'h01; 13:pkt_byte=8'h00; default:pkt_byte=8'h0F;
            endcase
            default: case (i) // EF01 FFFFFFFF 01 0003 1D 0021
                0:pkt_byte=8'hEF; 1:pkt_byte=8'h01; 2:pkt_byte=8'hFF; 3:pkt_byte=8'hFF;
                4:pkt_byte=8'hFF; 5:pkt_byte=8'hFF; 6:pkt_byte=8'h01; 7:pkt_byte=8'h00;
                8:pkt_byte=8'h03; 9:pkt_byte=8'h1D; 10:pkt_byte=8'h00; default:pkt_byte=8'h21;
            endcase
        endcase
    endfunction

    // ---------------- FSM ----------------
    localparam [2:0] S_IDLE    = 3'd0,
                     S_SEND    = 3'd1,   // hand one byte to uart_tx
                     S_WAIT_TX = 3'd2,   // wait for tx_done
                     S_WAIT_ACK= 3'd3,   // collect ack bytes
                     S_CHECK   = 3'd4,   // look at ack_code, pick next step
                     S_DONE    = 3'd5;

    reg [2:0]  state;
    reg [4:0]  tx_idx;                   // byte index being sent
    reg [4:0]  rx_idx;                   // byte index being received
    reg [7:0]  len_lo;                   // ack Length (low byte)
    reg [25:0] to_cnt;                   // timeout counter

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            seq_ok      <= 1'b0;
            ack_ok      <= 1'b0;
            ack_timeout <= 1'b0;
            ack_code    <= 8'h00;
            ack_page_id <= 16'h0000;
            ack_score   <= 16'h0000;
            step        <= 3'd0;
            tx_data     <= 8'h00;
            tx_start    <= 1'b0;
            tx_idx      <= 5'd0;
            rx_idx      <= 5'd0;
            len_lo      <= 8'h00;
            to_cnt      <= 26'd0;
        end else begin
            // defaults: one-clock pulses
            tx_start <= 1'b0;
            done     <= 1'b0;

            case (state)
            // ------------------------------------------------ wait for start
            S_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    busy        <= 1'b1;
                    seq_ok      <= 1'b0;
                    ack_ok      <= 1'b0;
                    ack_timeout <= 1'b0;
                    step        <= 3'd0;
                    tx_idx      <= 5'd0;
                    state       <= S_SEND;
                end
            end

            // ------------------------------------------------ send one byte
            S_SEND: begin
                if (!tx_busy) begin
                    tx_data  <= pkt_byte(step, tx_idx);
                    tx_start <= 1'b1;
                    state    <= S_WAIT_TX;
                end
            end

            S_WAIT_TX: begin
                if (tx_done) begin
                    if (tx_idx == pkt_len(step) - 5'd1) begin
                        // whole packet sent -> go listen for the ack
                        rx_idx      <= 5'd0;
                        to_cnt      <= 26'd0;
                        ack_page_id <= 16'h0000;
                        ack_score   <= 16'h0000;
                        state       <= S_WAIT_ACK;
                    end else begin
                        tx_idx <= tx_idx + 5'd1;
                        state  <= S_SEND;
                    end
                end
            end

            // ------------------------------------------------ receive the ack
            S_WAIT_ACK: begin
                if (rx_valid) begin
                    to_cnt <= 26'd0;                       // restart on every byte
                    rx_idx <= rx_idx + 5'd1;

                    if (frame_err ||
                        (rx_idx == 5'd0 && rx_data != 8'hEF) ||
                        (rx_idx == 5'd1 && rx_data != 8'h01) ||
                        (rx_idx == 5'd6 && rx_data != 8'h07)) begin
                        // malformed: ack_ok = 0, ack_timeout = 0
                        ack_ok <= 1'b0;
                        state  <= S_DONE;
                    end else begin
                        case (rx_idx)
                            5'd8:  len_lo <= rx_data;                       // Length low
                            5'd9:  ack_code <= rx_data;                     // confirmation code
                            5'd10: if (len_lo >= 8'd5) ack_page_id[15:8] <= rx_data;
                            5'd11: if (len_lo >= 8'd5) ack_page_id[7:0]  <= rx_data;
                            5'd12: if (len_lo >= 8'd7) ack_score[15:8]   <= rx_data;
                            5'd13: if (len_lo >= 8'd7) ack_score[7:0]    <= rx_data;
                            default: ;
                        endcase
                        // last byte index = 8 + Length (checksum low byte)
                        if (rx_idx > 5'd8 && rx_idx == 5'd8 + len_lo[4:0]) begin
                            ack_ok <= 1'b1;
                            state  <= S_CHECK;
                        end
                    end
                end else if (to_cnt == TIMEOUT_CLKS - 1) begin
                    ack_ok      <= 1'b0;
                    ack_timeout <= 1'b1;
                    state       <= S_DONE;
                end else begin
                    to_cnt <= to_cnt + 26'd1;
                end
            end

            // ------------------------------------------------ decide next step
            S_CHECK: begin
                if (ack_code != 8'h00) begin
                    state <= S_DONE;                // command failed, stop
                end else if (step == 3'd4) begin
                    seq_ok <= 1'b1;                 // all five steps ok
                    state  <= S_DONE;
                end else begin
                    step   <= step + 3'd1;
                    tx_idx <= 5'd0;
                    state  <= S_SEND;
                end
            end

            // ------------------------------------------------ finish
            S_DONE: begin
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
