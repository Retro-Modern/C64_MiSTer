// WiC64 bridge for the MiSTer C64 core.
//
// Copyright (C) 2026 Retro-Modern - https://retro-modern.net
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Emulates the user port side of a WiC64 (PB0-7, PA2, PC2, FLAG2) and passes
// the bytes over the MiSTer's serial line (/dev/ttyS1, 1 562 500 baud, 8N1)
// to the service wic64d.py running on MiSTer Linux.
//
// Handshake as in WiC64 firmware 2.x (userport.cpp):
//  * C64 sends (PA2 high): every PC2 pulse (sta $dd01) delivers a byte; the
//    bridge acknowledges it itself with a FLAG2 pulse as soon as the byte is
//    in the event queue.
//  * C64 receives (PA2 low): Linux sends HS ("response ready") and then
//    DATA. Every PC2 pulse (lda $dd01, including the dummy read after the
//    direction change) requests the next byte; the bridge puts it on PB and
//    pulses FLAG2. If the byte has not arrived yet, it waits for it.
//
// Linux -> FPGA (commands, in the 4 KB queue):
//   10 n d0..   DATA, n bytes (0 = 256) for the C64
//   11          HS, one FLAG2 pulse right away
//   12          PING, answered with 06 VERSION
//   13          release PB once the C64 has fetched the last byte
//   14          MARK, answered with 07 00 once everything before it is at the C64
//   DTR edge    discard everything (queue, parser), answered with 04 00
//
// FPGA -> Linux (events, two bytes each):
//   01 b  byte from the C64    02 l  PA2 changed (l = new level)
//   03 k  256 command bytes consumed (k counts up, credit)
//   04 00 discarded (after DTR)   05 00 reset (C64 reset or switched on)
//   06 v  PONG      07 00 MARK reached

module wic64_bridge #(
	parameter VERSION   = 8'h01,
	parameter PULSE_CLK = 160,      // FLAG2 low, ~5 us at 32 MHz
	parameter GAP_CLK   = 32        // gap between two pulses
)(
	input            clk,
	input            reset,         // C64 reset or bridge off
	input     [23:0] baud_inc,      // round(1562500 / f_clk * 2^24)

	input      [7:0] pb_o,          // CIA 2 Port B (prb | ~ddrb)
	output     [7:0] pb_i,
	input            pa2_o,
	input            pc2_n,
	output           flag2_n,

	input            uart_rx,
	output           uart_tx,
	input            uart_flush     // DTR from the HPS, every edge discards
);

// ------------------------------------------------------------------ UART RX
reg [1:0]  rx_s = 2'b11;
reg [23:0] rx_acc;
reg [3:0]  rx_bit;
reg [7:0]  rx_sh;
reg        rx_busy = 0;
reg        rx_stb;
reg [7:0]  rx_byte;

always @(posedge clk) begin
	reg [24:0] sum;
	rx_s   <= {rx_s[0], uart_rx};
	rx_stb <= 0;
	if(!rx_busy) begin
		if(!rx_s[1]) begin             // start bit
			rx_busy <= 1;
			rx_acc  <= 24'h800000;      // first sample after half a bit
			rx_bit  <= 0;
		end
	end
	else begin
		sum = {1'b0, rx_acc} + {1'b0, baud_inc};
		rx_acc <= sum[23:0];
		if(sum[24]) begin
			rx_bit <= rx_bit + 1'd1;
			if(rx_bit == 0) begin
				if(rx_s[1]) rx_busy <= 0; // glitch, no start bit
			end
			else if(rx_bit <= 8) rx_sh <= {rx_s[1], rx_sh[7:1]};
			else begin
				rx_busy <= 0;
				if(rx_s[1]) begin        // valid stop bit
					rx_stb  <= 1;
					rx_byte <= rx_sh;
				end
			end
		end
	end
end

// DTR edge
reg [2:0] fl_s;
always @(posedge clk) fl_s <= {fl_s[1:0], uart_flush};
wire fl_edge = fl_s[2] ^ fl_s[1];

// ------------------------------------------------- command FIFO, 4096 bytes
reg  [7:0] cmd_ram[4096];
reg [12:0] cmd_wp, cmd_rp;
wire       cmd_empty = (cmd_wp == cmd_rp);
reg        cmd_rd;                  // start a read
reg  [7:0] cmd_q;                   // valid one clock after cmd_rd

always @(posedge clk) begin
	if(rx_stb && (cmd_wp - cmd_rp) != 13'd4096) cmd_ram[cmd_wp[11:0]] <= rx_byte;
	cmd_q <= cmd_ram[cmd_rp[11:0]];
end

always @(posedge clk) begin
	if(reset | fl_edge) begin           // in the same clock as the parser
		cmd_wp <= 0;
		cmd_rp <= 0;
	end
	else begin
		if(rx_stb && (cmd_wp - cmd_rp) != 13'd4096) cmd_wp <= cmd_wp + 1'd1;
		if(cmd_rd) cmd_rp <= cmd_rp + 1'd1;
	end
end

// --------------------------------------------- event FIFO, 64 x 2 bytes
reg [15:0] ev_ram[64];
reg  [6:0] ev_wp, ev_rp;
wire       ev_full  = (ev_wp - ev_rp) == 7'd64;
wire       ev_empty = (ev_wp == ev_rp);
reg        ev_put;
reg [15:0] ev_d;

// ---------------------------------------------------------------- UART TX
reg [23:0] tx_acc;
reg  [9:0] tx_sh = 10'h3FF;
reg  [3:0] tx_cnt = 0;
reg        tx_hi;                   // second byte of the event pending
reg  [7:0] tx_lo;
reg        ev_get;
assign uart_tx = tx_sh[0];

always @(posedge clk) begin
	reg [24:0] sum;
	ev_get <= 0;
	if(tx_cnt) begin
		sum = {1'b0, tx_acc} + {1'b0, baud_inc};
		tx_acc <= sum[23:0];
		if(sum[24]) begin
			tx_sh  <= {1'b1, tx_sh[9:1]};
			tx_cnt <= tx_cnt - 1'd1;
		end
	end
	else if(tx_hi) begin
		tx_sh  <= {1'b1, tx_lo, 1'b0};
		tx_cnt <= 10;
		tx_acc <= 0;
		tx_hi  <= 0;
	end
	else if(!ev_empty && !ev_get) begin
		tx_sh  <= {1'b1, ev_ram[ev_rp[5:0]][15:8], 1'b0};
		tx_lo  <= ev_ram[ev_rp[5:0]][7:0];
		tx_cnt <= 10;
		tx_acc <= 0;
		tx_hi  <= 1;
		ev_get <= 1;
	end
end

always @(posedge clk) begin
	if(ev_put & ~ev_full) ev_ram[ev_wp[5:0]] <= ev_d;
	if(reset) begin
		ev_wp <= 0;
		ev_rp <= 0;
	end
	else begin
		if(ev_put & ~ev_full) ev_wp <= ev_wp + 1'd1;
		if(ev_get) ev_rp <= ev_rp + 1'd1;
	end
end

// ------------------------------------------------------------ FLAG2 pulses
reg  [7:0] pulse_cnt = 0;
reg        pulse_low = 0;
reg        pulse_req;
wire       pulse_idle = !pulse_cnt && !pulse_req;
assign flag2_n = ~pulse_low;

always @(posedge clk) begin
	if(reset) begin
		pulse_cnt <= 0;
		pulse_low <= 0;
	end
	else if(pulse_cnt) begin
		pulse_cnt <= pulse_cnt - 1'd1;
		if(pulse_cnt == GAP_CLK[7:0]) pulse_low <= 0;
	end
	else if(pulse_req) begin
		pulse_low <= 1;
		pulse_cnt <= PULSE_CLK[7:0] + GAP_CLK[7:0];
	end
end

// ------------------------------------------------------------ Userport
reg  [7:0] out_byte = 8'hFF;
reg        drive = 0;
assign pb_i = drive ? out_byte : 8'hFF;

reg pc2_d = 1, pa2_d = 1, pa2_seen = 0;
wire pc2_end = ~pc2_d & pc2_n;      // end of the PC2 pulse


// parser and handshake
localparam P_CMD = 0, P_WAIT = 1, P_DEC = 2, P_HS = 3, P_LEN = 4, P_LENW = 5, P_LENQ = 6,
           P_DATA = 7, P_DATAW = 8, P_DATAQ = 9;
reg  [3:0] pst;
reg  [8:0] dcount;
reg        need_byte;               // C64 requested the next byte
reg        rx_pend;                 // byte from the C64 awaits acknowledge
reg  [7:0] rx_val;
reg        rst_pend, fl_pend, pa2_pend, pong_pend, cred_pend, mark_pend;
reg  [7:0] consumed, credit;
reg        rel_pb;

always @(posedge clk) begin
	cmd_rd    <= 0;
	ev_put    <= 0;
	pulse_req <= 0;
	pc2_d     <= pc2_n;

	if(reset) begin
		pst       <= P_CMD;
		need_byte <= 0;
		rx_pend   <= 0;
		drive     <= 0;
		rst_pend  <= 1;
		fl_pend   <= 0;
		pa2_pend  <= 0;
		pong_pend <= 0;
		mark_pend <= 0;
		cred_pend <= 0;
		consumed  <= 0;
		credit    <= 0;
		pa2_d     <= pa2_o;
		pa2_seen  <= 0;
		rel_pb   <= 0;
	end
	else if(fl_edge) begin
		pst       <= P_CMD;
		need_byte <= 0;
		fl_pend   <= 1;
		mark_pend <= 0;
		cred_pend <= 0;
		consumed  <= 0;
		credit    <= 0;
	end
	else begin
		// PA2 change
		pa2_d <= pa2_o;
		if(pa2_d != pa2_o) begin
			pa2_pend <= 1;
			if(pa2_o) begin             // C64 wants to send
				need_byte <= 0;
				drive     <= 0;
				rel_pb   <= 0;
			end
		end

		// PC2: byte from the C64 or request for the next byte
		if(pc2_end) begin
			if(pa2_o) begin
				rx_pend <= 1;
				rx_val  <= pb_o;
			end
			else if(rel_pb) begin
				drive   <= 0;
				rel_pb <= 0;
			end
			else need_byte <= 1;
		end

		// commands
		case(pst)
			P_CMD:
				if(!cmd_empty) begin
					cmd_rd <= 1;
					pst    <= P_WAIT;
				end
			P_WAIT:
				pst <= P_DEC;           // one clock read latency
			P_DEC: begin
				pst <= P_CMD;
				case(cmd_q)
					8'h10: pst <= P_LEN;
					8'h11: pst <= P_HS;     // HS, waits for a free pulse stage
					8'h12: pong_pend <= 1;
					8'h13: rel_pb <= 1;    // after the next PC2
					8'h14: mark_pend <= 1;
					default: ;
				endcase
			end
			P_HS:
				if(pulse_idle && !rx_pend) begin
					pulse_req <= 1;
					pst <= P_CMD;
				end
			P_LEN:
				if(!cmd_empty) begin
					cmd_rd <= 1;
					pst    <= P_LENW;
				end
			P_LENW:
				pst <= P_LENQ;
			P_LENQ: begin
				dcount <= cmd_q ? {1'b0, cmd_q} : 9'd256;
				pst    <= P_DATA;
			end
			P_DATA:
				if(!dcount) pst <= P_CMD;
				else if(need_byte && !pa2_o && pulse_idle && !cmd_empty) begin
					cmd_rd    <= 1;
					need_byte <= 0;
					pst       <= P_DATAW;
				end
			P_DATAW:
				pst <= P_DATAQ;
			P_DATAQ: begin
				out_byte  <= cmd_q;
				drive     <= 1;
				pulse_req <= 1;
				dcount    <= dcount - 1'd1;
				pst       <= P_DATA;
			end
			default: pst <= P_CMD;
		endcase

		// credit: every 256 consumed command bytes
		if(cmd_rd) begin
			consumed <= consumed + 1'd1;
			if(consumed == 8'hFF) cred_pend <= 1;
		end

		// acknowledge the byte from the C64 once it is queued
		if(rx_pend && pulse_idle && !ev_full && !ev_put && pst != P_HS && pst != P_DATAW && pst != P_DATAQ) begin
			ev_put    <= 1;
			ev_d      <= {8'h01, rx_val};
			rx_pend   <= 0;
			pulse_req <= 1;
		end
		else if(!ev_full && !ev_put && !rx_pend) begin
			if(rst_pend)       begin ev_put <= 1; ev_d <= 16'h0500;            rst_pend  <= 0; end
			else if(fl_pend)   begin ev_put <= 1; ev_d <= 16'h0400;            fl_pend   <= 0; end
			else if(pa2_pend)  begin ev_put <= 1; ev_d <= {8'h02, 7'd0, pa2_o}; pa2_pend <= 0; end
			else if(cred_pend) begin ev_put <= 1; ev_d <= {8'h03, credit + 1'd1}; credit <= credit + 1'd1; cred_pend <= 0; end
			else if(pong_pend) begin ev_put <= 1; ev_d <= {8'h06, VERSION};    pong_pend <= 0; end
			else if(mark_pend) begin ev_put <= 1; ev_d <= 16'h0700;            mark_pend <= 0; end
		end
	end
end

endmodule
