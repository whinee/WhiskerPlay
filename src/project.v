/*
 * QR code (version 1, 21x21 modules) on VGA - Tiny Tapeout, 1x1 tile
 *
 * The chip is a QR *display engine*.  It stores the 26 finished codewords
 * (data + Reed-Solomon parity, 208 bits), and draws finder/timing patterns,
 * format info, zig-zag data placement and the data mask on the fly, pixel by
 * pixel.  The host (RP2040 / PC) does the text -> codeword encoding, see
 * qr_host.py.
 *
 * INPUT PROTOCOL  (ui_in[7:6] = address, ui_in[5:0] = data)
 *
 *   00 dddddd   idle / "clock low".  Every command needs a 00 before it.
 *   01 ....ee   set error-correction LEVEL    ee: 0=L 1=M 2=Q 3=H
 *   10 ...mmm   set data MASK                 mmm: 0..7
 *   11 dddddd   shift 6 data bits into the codeword shift register
 *
 * A command fires once, on the first cycle where the address goes from 00 to
 * non-zero and the inputs have been stable for 2 clocks (glitch filter, so
 * 00 -> 11 with some pin skew can't fire a fake 01 or 10).  Hold each value
 * for at least ~4 clock cycles.
 *
 * Data: 35 pushes = 210 bits = [2 dummy 0 bits] + 208 codeword bits, MSB
 * first.  The first 2 bits fall off the far end of the 208-bit register.
 * After reset the chip shows a built-in demo code (HTTPS://TINYTAPEOUT.COM,
 * level L, mask 5), so you can see something before you send anything.
 *
 * Pins: uo_out is the Tiny VGA PMOD:  {hsync, B0, G0, R0, vsync, B1, G1, R1}
 *
 * NOTE: the ecc level you set with 01 only changes the *format bits*.  It
 * must match the level the host used when it computed the parity.
 */

`default_nettype none

module tt_um_whinee (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // 25.175 MHz pixel clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ------------------------------------------------------------------
  // power-up demo:  "HTTPS://TINYTAPEOUT.COM", level L, mask 5
  // (delete the reset values below if you want to save a bit of area)
  // ------------------------------------------------------------------
  localparam [207:0] DEFAULT_SR   = 208'h20bb1aa65463dd52b85b48e39c56a868d160ecba9456988a668c;
  localparam [1:0]   DEFAULT_ECC  = 2'd0;
  localparam [2:0]   DEFAULT_MASK = 3'd5;

  // ------------------------------------------------------------------
  // small helper functions
  // ------------------------------------------------------------------
  function [1:0] mod3;              // v % 3, v = 0..20
    input [4:0] v;
    begin
      case (v)
        5'd0, 5'd3, 5'd6, 5'd9,  5'd12, 5'd15, 5'd18: mod3 = 2'd0;
        5'd1, 5'd4, 5'd7, 5'd10, 5'd13, 5'd16, 5'd19: mod3 = 2'd1;
        default:                                      mod3 = 2'd2;
      endcase
    end
  endfunction

  function d3par;                   // parity of floor(v / 3)
    input [4:0] v;
    begin
      case (v)
        5'd3, 5'd4, 5'd5, 5'd9, 5'd10, 5'd11, 5'd15, 5'd16, 5'd17: d3par = 1'b1;
        default:                                                    d3par = 1'b0;
      endcase
    end
  endfunction

  function fdark;                   // 7x7 finder pattern, a/b = 0..6
    input [2:0] a;
    input [2:0] b;
    begin
      fdark = (a == 3'd0) || (a == 3'd6) || (b == 3'd0) || (b == 3'd6) ||
              ((a >= 3'd2) && (a <= 3'd4) && (b >= 3'd2) && (b <= 3'd4));
    end
  endfunction

  // 15 format bits: BCH(15,5) of {ecc indicator, mask}, xor 0x5412
  function [14:0] fmt_bits;
    input [4:0] d;
    integer i;
    reg [14:0] v;
    begin
      v = {d, 10'b0};
      for (i = 14; i >= 10; i = i - 1)
        if (v[i]) v = v ^ (15'h0537 << (i - 10));
      fmt_bits = ({d, 10'b0} | {5'b0, v[9:0]}) ^ 15'h5412;
    end
  endfunction

  // ------------------------------------------------------------------
  // input side: synchroniser, glitch filter, 00-strobe command decoder
  // ------------------------------------------------------------------
  reg [7:0]   in_a, in_b;
  reg         armed;
  reg [1:0]   ecc;                  // 0=L 1=M 2=Q 3=H
  reg [2:0]   mask;
  reg [207:0] sr;                   // first codeword bit lives in sr[207]

  wire       stable = (in_a == in_b);   // same value on two consecutive clocks
  wire [1:0] cmd    = in_b[7:6];

  always @(posedge clk) begin
    if (!rst_n) begin
      in_a  <= 8'd0;
      in_b  <= 8'd0;
      armed <= 1'b0;
      ecc   <= DEFAULT_ECC;
      mask  <= DEFAULT_MASK;
      sr    <= DEFAULT_SR;
    end else begin
      in_a <= ui_in;
      in_b <= in_a;
      if (stable) begin
        if (cmd == 2'b00) begin
          armed <= 1'b1;                       // saw the "clock low" phase
        end else if (armed) begin
          armed <= 1'b0;                       // fire exactly once
          case (cmd)
            2'b01:   ecc  <= in_b[1:0];
            2'b10:   mask <= in_b[2:0];
            default: sr   <= {sr[201:0], in_b[5:0]};
          endcase
        end
      end
    end
  end

  wire [14:0] fmt = fmt_bits({ecc ^ 2'b01, mask});   // L=01 M=00 Q=11 H=10 in the spec

  // ------------------------------------------------------------------
  // VGA timing, 640x480 @ 60 Hz, negative sync
  // ------------------------------------------------------------------
  reg [9:0] hc, vc;
  always @(posedge clk) begin
    if (!rst_n) begin
      hc <= 10'd0;
      vc <= 10'd0;
    end else if (hc == 10'd799) begin
      hc <= 10'd0;
      vc <= (vc == 10'd524) ? 10'd0 : vc + 10'd1;
    end else begin
      hc <= hc + 10'd1;
    end
  end

  wire hsync_c = ~((hc >= 10'd656) && (hc < 10'd752));
  wire vsync_c = ~((vc >= 10'd490) && (vc < 10'd492));
  wire active  = (hc < 10'd640) && (vc < 10'd480);

  // ------------------------------------------------------------------
  // 16x16 pixel modules; the code occupies modules 10..30 / 4..24 of the
  // 40x30 module screen  ->  pixels x 160..495, y 64..399 (quiet zone >= 4)
  // ------------------------------------------------------------------
  wire [5:0] hb = hc[9:4];
  wire [5:0] vb = vc[9:4];
  wire in_qr = (hb >= 6'd10) && (hb < 6'd31) && (vb >= 6'd4) && (vb < 6'd25);
  wire [4:0] mx = hb[4:0] - 5'd10;      // 0..20 inside the code
  wire [4:0] my = vb[4:0] - 5'd4;

  // ---- function modules: finders, separators, timing, format, dark module
  wire tl = (mx <= 5'd8)  && (my <= 5'd8);
  wire tr = (mx >= 5'd13) && (my <= 5'd8);
  wire bl = (mx <= 5'd8)  && (my >= 5'd13);
  wire is_func = tl | tr | bl | (mx == 5'd6) | (my == 5'd6);

  wire [4:0] mxr = mx - 5'd14;          // column inside the top-right finder
  wire [4:0] myb = my - 5'd14;          // row inside the bottom-left finder
  wire [4:0] fi_a = 5'd14 - mx;         // format bit index, row 8, left part
  wire [4:0] fi_b = 5'd20 - mx;         // format bit index, row 8, right part
  wire [4:0] fi_c = my - 5'd6;          // format bit index, column 8, lower part

  reg fd;                               // colour of a function module
  always @* begin
    fd = 1'b0;                          // separators stay light
    if      (mx < 5'd7  && my < 5'd7)                       fd = fdark(mx[2:0],  my[2:0]);
    else if (mx >= 5'd14 && my < 5'd7)                      fd = fdark(mxr[2:0], my[2:0]);
    else if (mx < 5'd7  && my >= 5'd14)                     fd = fdark(mx[2:0],  myb[2:0]);
    else if (my == 5'd6 && mx >= 5'd8 && mx <= 5'd12)       fd = ~mx[0];      // timing row
    else if (mx == 5'd6 && my >= 5'd8 && my <= 5'd12)       fd = ~my[0];      // timing column
    else if (mx == 5'd8 && my == 5'd13)                     fd = 1'b1;        // the always-dark module
    else if (mx == 5'd8 && my <= 5'd5)                      fd = fmt[my[3:0]];
    else if (mx == 5'd8 && my == 5'd7)                      fd = fmt[6];
    else if (mx == 5'd8 && my == 5'd8)                      fd = fmt[7];
    else if (mx == 5'd7 && my == 5'd8)                      fd = fmt[8];
    else if (my == 5'd8 && mx <= 5'd5)                      fd = fmt[fi_a[3:0]];
    else if (my == 5'd8 && mx >= 5'd13)                     fd = fmt[fi_b[3:0]];
    else if (mx == 5'd8 && my >= 5'd14)                     fd = fmt[fi_c[3:0]];
  end

  // ---- data modules: which codeword bit sits here?
  // 10 column pairs (right column 20,18,..,8 then 5,3,1), snaking up/down.
  wire [4:0] u  = (mx > 5'd6) ? (5'd20 - mx) : (5'd19 - mx);
  wire [3:0] p  = u[4:1];               // column pair 0..9
  wire       cb = u[0];                 // 0 = right column of the pair, 1 = left

  reg [7:0] base;                       // number of data modules in earlier pairs
  reg [4:0] r;                          // row position within this pair's walk
  always @* begin
    base = 8'd0;
    r    = 5'd0;
    case (p)
      4'd0:    begin base = 8'd0;   r = 5'd20 - my; end
      4'd1:    begin base = 8'd24;  r = my - 5'd9;  end
      4'd2:    begin base = 8'd48;  r = 5'd20 - my; end
      4'd3:    begin base = 8'd72;  r = my - 5'd9;  end
      4'd4:    begin base = 8'd96;  r = (my > 5'd6) ? (5'd20 - my) : (5'd19 - my); end
      4'd5:    begin base = 8'd136; r = (my > 5'd6) ? (my - 5'd1)  : my;          end
      4'd6:    begin base = 8'd176; r = 5'd12 - my; end
      4'd7:    begin base = 8'd184; r = my - 5'd9;  end
      4'd8:    begin base = 8'd192; r = 5'd12 - my; end
      default: begin base = 8'd200; r = my - 5'd9;  end
    endcase
  end

  wire [7:0] idx     = base + {2'b00, r, 1'b0} + {7'b0, cb};
  wire [7:0] idx_rev = 8'd207 - idx;
  wire       cwbit   = sr[idx_rev];

  // ---- data mask
  wire [1:0] xm  = mod3(mx);
  wire [1:0] ym  = mod3(my);
  wire [2:0] xys = {1'b0, xm} + {1'b0, ym};
  wire       p3z = (xm == 2'd0) | (ym == 2'd0);                        // (x*y) % 3 == 0
  wire       p3o = ~p3z & (xm == ym);                                  // low bit of (x*y) % 3
  wire       x0  = mx[0];
  wire       y0  = my[0];

  reg inv;
  always @* begin
    inv = 1'b0;
    case (mask)
      3'd0:    inv = ~(x0 ^ y0);
      3'd1:    inv = ~y0;
      3'd2:    inv = (xm == 2'd0);
      3'd3:    inv = (xys == 3'd0) | (xys == 3'd3);
      3'd4:    inv = ~(d3par(mx) ^ my[1]);
      3'd5:    inv = ~(x0 & y0) & p3z;
      3'd6:    inv = ~((x0 & y0) ^ p3o);
      default: inv = ~((x0 ^ y0) ^ p3o);
    endcase
  end

  wire dark = is_func ? fd : (cwbit ^ inv);

  // ------------------------------------------------------------------
  // outputs: black modules on a white screen, registered
  // ------------------------------------------------------------------
  reg hs_r, vs_r, lit_r;
  always @(posedge clk) begin
    hs_r  <= hsync_c;
    vs_r  <= vsync_c;
    lit_r <= active & ~(in_qr & dark);
  end

  wire [1:0] R = {2{lit_r}};
  wire [1:0] G = {2{lit_r}};
  wire [1:0] B = {2{lit_r}};

  assign uo_out  = {hs_r, B[0], G[0], R[0], vs_r, B[1], G[1], R[1]};
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  wire _unused = &{ena, uio_in, 1'b0};

endmodule