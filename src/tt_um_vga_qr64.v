`default_nettype none

// Tiny Tapeout VGA QR-64
//
// Payload entry:
//   ui_in[7:0]   = one payload byte
//   uio_in[1:0]  = error correction: 00=L, 01=M, 10=Q, 11=H
//   uio_in[4:2]  = mask override: 000=auto, 001..111=force mask 1..7
//   uio_in[5]    = LOAD byte (rising edge)
//   uio_in[6]    = GO / encode (rising edge)
//   uio_in[7]    = unused
//
// Eight LOAD pulses, MSB byte first, form the 64-bit payload.  Version 2 is
// used for every EC level; 64 payload bits fit in all four EC configurations.
// The encoder creates byte-mode QR data, Reed-Solomon ECC, all eight masks,
// and chooses the lowest standard QR penalty when mask override is 000.
//
// The VGA renderer displays the 25x25 version-2 symbol as a 16-pixel module
// grid.  The screen itself supplies the white quiet/background area.

module tt_um_vga_qr64 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire hsync, vsync, display_on;
    wire [10:0] hpos;
    wire [9:0]  vpos;

    hvsync_generator hvsync_gen(
        .clk(clk),
        .reset(~rst_n),
        .mode(2'd0),
        .hsync(hsync),
        .vsync(vsync),
        .display_on(display_on),
        .hpos(hpos),
        .vpos(vpos)
    );

    // TinyVGA PMOD: {HSync, B1, G1, R1, VSync, B0, G0, R0}
    wire qr_in_box = (hpos >= 11'd112) && (hpos < 11'd512) &&
                     (vpos >= 10'd40)  && (vpos < 10'd440);
    wire [4:0] qr_x = (hpos - 11'd112) >> 4;
    wire [4:0] qr_y = (vpos - 10'd40)  >> 4;

    reg [7:0] cw [0:43];
    reg [63:0] payload;
    reg [3:0]  load_index;
    reg        payload_ready;
    reg        load_d, go_d;

    reg [1:0] ec_sel;
    reg [2:0] mask_sel;
    reg [2:0] selected_mask;
    reg       display_ready;

    reg [7:0] parity [0:27];
    reg [5:0] data_count, ec_count;
    reg [5:0] data_index_r;
    reg [5:0] rs_index;
    reg [7:0] rs_feedback_r;
    reg [5:0] ec_index;

    reg [15:0] score_pen;
    reg [9:0]  dark_count;
    reg [2:0]  score_mask;
    reg [4:0]  score_x, score_y;
    reg        score_col_pass;
    reg        score_prev;
    reg [5:0]  score_run;
    reg [24:0] score_row_prev;
    reg [24:0] score_row_cur;
    reg [10:0] score_win;
    reg        score_n3_skip;
    reg [15:0] best_pen;
    reg [2:0]  best_mask;

    localparam ST_START      = 5'd0;
    localparam ST_BUILD      = 5'd1;
    localparam ST_RS         = 5'd2;
    localparam ST_RS_APPEND  = 5'd3;
    localparam ST_SCORE_INIT = 5'd4;
    localparam ST_SCORE_RUN  = 5'd5;
    localparam ST_IDLE       = 5'd7;
    reg [4:0] state;

    wire load_rise = uio_in[5] & ~load_d;
    wire go_rise   = uio_in[6] & ~go_d;

    assign uio_out = 8'd0;
    assign uio_oe  = 8'd0;

    // White background, black QR modules.
    wire qr_dark = qr_module(qr_x, qr_y, selected_mask);
    wire [5:0] RGB = (display_on && qr_in_box && display_ready) ?
                     (qr_dark ? 6'b000000 : 6'b111111) : 6'b111111;

    assign uo_out = {hsync, RGB[0], RGB[2], RGB[4], vsync, RGB[1], RGB[3], RGB[5]};

    // ---------------------------------------------------------------------
    // GF(256), QR polynomial x^8+x^4+x^3+x^2+1 (0x11d)
    // ---------------------------------------------------------------------
    function [7:0] gf_mul;
        input [7:0] a_in;
        input [7:0] b_in;
        reg [7:0] a, b, p;
        integer i;
        begin
            a = a_in;
            b = b_in;
            p = 8'd0;
            for (i = 0; i < 8; i = i + 1) begin
                if (b[0]) p = p ^ a;
                a = {a[6:0],1'b0} ^ (a[7] ? 8'h1d : 8'h00);
                b = b >> 1;
            end
            gf_mul = p;
        end
    endfunction

    // Generator polynomial coefficients for 10/16/22/28 ECC codewords.
    function [7:0] gen_coeff;
        input [1:0] ec;
        input [5:0] n;
        begin
            gen_coeff = 8'h00;
            case (ec)
                2'd0: case (n)
                    0:gen_coeff=8'h01; 1:gen_coeff=8'hd8; 2:gen_coeff=8'hc2; 3:gen_coeff=8'h9f;
                    4:gen_coeff=8'h6f; 5:gen_coeff=8'hc7; 6:gen_coeff=8'h5e; 7:gen_coeff=8'h5f;
                    8:gen_coeff=8'h71; 9:gen_coeff=8'h9d; 10:gen_coeff=8'hc1; default:gen_coeff=0;
                endcase
                2'd1: case (n)
                    0:gen_coeff=8'h01; 1:gen_coeff=8'h3b; 2:gen_coeff=8'h0d; 3:gen_coeff=8'h68;
                    4:gen_coeff=8'hbd; 5:gen_coeff=8'h44; 6:gen_coeff=8'hd1; 7:gen_coeff=8'h1e;
                    8:gen_coeff=8'h08; 9:gen_coeff=8'ha3; 10:gen_coeff=8'h41; 11:gen_coeff=8'h29;
                    12:gen_coeff=8'he5; 13:gen_coeff=8'h62; 14:gen_coeff=8'h32; 15:gen_coeff=8'h24;
                    16:gen_coeff=8'h3b; default:gen_coeff=0;
                endcase
                2'd2: case (n)
                    0:gen_coeff=8'h01; 1:gen_coeff=8'h59; 2:gen_coeff=8'hb3; 3:gen_coeff=8'h83;
                    4:gen_coeff=8'hb0; 5:gen_coeff=8'hb6; 6:gen_coeff=8'hf4; 7:gen_coeff=8'h13;
                    8:gen_coeff=8'hbd; 9:gen_coeff=8'h45; 10:gen_coeff=8'h28; 11:gen_coeff=8'h1c;
                    12:gen_coeff=8'h89; 13:gen_coeff=8'h1d; 14:gen_coeff=8'h7b; 15:gen_coeff=8'h43;
                    16:gen_coeff=8'hfd; 17:gen_coeff=8'h56; 18:gen_coeff=8'hda; 19:gen_coeff=8'he6;
                    20:gen_coeff=8'h1a; 21:gen_coeff=8'h91; 22:gen_coeff=8'hf5; default:gen_coeff=0;
                endcase
                default: case (n)
                    0:gen_coeff=8'h01; 1:gen_coeff=8'hfc; 2:gen_coeff=8'h09; 3:gen_coeff=8'h1c;
                    4:gen_coeff=8'h0d; 5:gen_coeff=8'h12; 6:gen_coeff=8'hfb; 7:gen_coeff=8'hd0;
                    8:gen_coeff=8'h96; 9:gen_coeff=8'h67; 10:gen_coeff=8'hae; 11:gen_coeff=8'h64;
                    12:gen_coeff=8'h29; 13:gen_coeff=8'ha7; 14:gen_coeff=8'h0c; 15:gen_coeff=8'hf7;
                    16:gen_coeff=8'h38; 17:gen_coeff=8'h75; 18:gen_coeff=8'h77; 19:gen_coeff=8'he9;
                    20:gen_coeff=8'h7f; 21:gen_coeff=8'hb5; 22:gen_coeff=8'h64; 23:gen_coeff=8'h79;
                    24:gen_coeff=8'h93; 25:gen_coeff=8'hb0; 26:gen_coeff=8'h4a; 27:gen_coeff=8'h3a;
                    28:gen_coeff=8'hc5; default:gen_coeff=0;
                endcase
            endcase
        end
    endfunction

    // ---------------------------------------------------------------------
    // QR function-module geometry for version 2 (25x25)
    // ---------------------------------------------------------------------
    function qr_is_function;
        input [4:0] x, y;
        reg [5:0] dx, dy;
        begin
            qr_is_function = 1'b0;
            // Finder + separator regions.
            if ((x <= 5'd7) && (y <= 5'd7)) qr_is_function = 1'b1;
            if ((x >= 5'd17) && (y <= 5'd7)) qr_is_function = 1'b1;
            if ((x <= 5'd7) && (y >= 5'd17)) qr_is_function = 1'b1;
            // Alignment pattern centered at (18,18).
            if ((x >= 5'd16) && (x <= 5'd20) && (y >= 5'd16) && (y <= 5'd20)) qr_is_function = 1'b1;
            // Timing strips.
            if ((y == 5'd6) && (x >= 5'd8) && (x <= 5'd16)) qr_is_function = 1'b1;
            if ((x == 5'd6) && (y >= 5'd8) && (y <= 5'd16)) qr_is_function = 1'b1;
            // Format information.
            if (x == 5'd8 && ((y <= 5'd5) || (y == 5'd7) || (y == 5'd8) || ((y >= 5'd18) && (y <= 5'd24)))) qr_is_function = 1'b1;
            if (y == 5'd8 && ((x >= 5'd17) && (x <= 5'd24) || (x == 5'd7) || ((x >= 5'd0) && (x <= 5'd5)))) qr_is_function = 1'b1;
            // Always-dark module.
            if ((x == 5'd8) && (y == 5'd17)) qr_is_function = 1'b1;
        end
    endfunction

    function [14:0] format_code;
        input [1:0] ec;
        input [2:0] mask;
        begin
            case ({ec,mask})
                5'b00_000: format_code=15'h77c4; 5'b00_001: format_code=15'h72f3;
                5'b00_010: format_code=15'h7daa; 5'b00_011: format_code=15'h789d;
                5'b00_100: format_code=15'h662f; 5'b00_101: format_code=15'h6318;
                5'b00_110: format_code=15'h6c41; 5'b00_111: format_code=15'h6976;
                5'b01_000: format_code=15'h5412; 5'b01_001: format_code=15'h5125;
                5'b01_010: format_code=15'h5e7c; 5'b01_011: format_code=15'h5b4b;
                5'b01_100: format_code=15'h45f9; 5'b01_101: format_code=15'h40ce;
                5'b01_110: format_code=15'h4f97; 5'b01_111: format_code=15'h4aa0;
                5'b10_000: format_code=15'h355f; 5'b10_001: format_code=15'h3068;
                5'b10_010: format_code=15'h3f31; 5'b10_011: format_code=15'h3a06;
                5'b10_100: format_code=15'h24b4; 5'b10_101: format_code=15'h2183;
                5'b10_110: format_code=15'h2eda; 5'b10_111: format_code=15'h2bed;
                5'b11_000: format_code=15'h1689; 5'b11_001: format_code=15'h13be;
                5'b11_010: format_code=15'h1ce7; 5'b11_011: format_code=15'h19d0;
                5'b11_100: format_code=15'h0762; 5'b11_101: format_code=15'h0255;
                5'b11_110: format_code=15'h0d0c; 5'b11_111: format_code=15'h083b;
                default:    format_code=15'h5412;
            endcase
        end
    endfunction

    function qr_function_value;
        input [4:0] x, y;
        input [1:0] ec;
        input [2:0] mask;
        reg [4:0] ox, oy;
        reg [3:0] dx, dy;
        reg [14:0] fmt;
        integer i;
        begin
            qr_function_value = 1'b0;
            fmt = format_code(ec,mask);

            // Finders.  The 8x8 region includes the one-module separator.
            if ((x <= 5'd7) && (y <= 5'd7)) begin
                if ((x == 5'd7) || (y == 5'd7)) qr_function_value = 1'b0;
                else begin
                    dx=x; dy=y;
                    qr_function_value = (dx==0 || dx==6 || dy==0 || dy==6 ||
                                         ((dx>=2)&&(dx<=4)&&(dy>=2)&&(dy<=4)));
                end
            end else if ((x >= 5'd17) && (y <= 5'd7)) begin
                if ((x == 5'd17) || (y == 5'd7)) qr_function_value = 1'b0;
                else begin
                    dx=x-5'd18; dy=y;
                    qr_function_value = (dx==0 || dx==6 || dy==0 || dy==6 ||
                                         ((dx>=2)&&(dx<=4)&&(dy>=2)&&(dy<=4)));
                end
            end else if ((x <= 5'd7) && (y >= 5'd17)) begin
                if ((x == 5'd7) || (y == 5'd17)) qr_function_value = 1'b0;
                else begin
                    dx=x; dy=y-5'd18;
                    qr_function_value = (dx==0 || dx==6 || dy==0 || dy==6 ||
                                         ((dx>=2)&&(dx<=4)&&(dy>=2)&&(dy<=4)));
                end
            end else if ((x >= 5'd16) && (x <= 5'd20) && (y >= 5'd16) && (y <= 5'd20)) begin
                // 5x5 alignment pattern: dark border, light ring, dark center.
                if ((x==5'd16)||(x==5'd20)||(y==5'd16)||(y==5'd20)||
                    ((x==5'd18)&&(y==5'd18)))
                    qr_function_value = 1'b1;
                else
                    qr_function_value = 1'b0;
            end else if ((y == 5'd6) && (x >= 5'd8) && (x <= 5'd16)) begin
                qr_function_value = ~x[0];
            end else if ((x == 5'd6) && (y >= 5'd8) && (y <= 5'd16)) begin
                qr_function_value = ~y[0];
            end else if (x == 5'd8 && y <= 5'd5) begin
                qr_function_value = fmt[y];
            end else if (x == 5'd8 && y == 5'd7) begin
                qr_function_value = fmt[6];
            end else if (x == 5'd8 && y == 5'd8) begin
                qr_function_value = fmt[7];
            end else if (x == 5'd8 && y >= 5'd18 && y <= 5'd24) begin
                qr_function_value = fmt[y-5'd10];
            end else if (y == 5'd8 && x >= 5'd17 && x <= 5'd24) begin
                qr_function_value = fmt[24-x];
            end else if (y == 5'd8 && x == 5'd7) begin
                qr_function_value = fmt[8];
            end else if (y == 5'd8 && x <= 5'd5) begin
                qr_function_value = fmt[14-x];
            end else if ((x == 5'd8) && (y == 5'd17)) begin
                qr_function_value = 1'b1;
            end
        end
    endfunction

    // Data-cell order for the standard QR zig-zag path.  The masks below are
    // the fixed version-2 function-cell exclusions for each two-column stripe.
    function [8:0] data_index;
        input [4:0] x, y;
        reg [4:0] pc;
        reg [8:0] base;
        reg [24:0] mr, ml, both;
        reg desc;
        integer i;
        integer cnt;
        begin
            pc=0; base=0; mr=0; ml=0; desc=0;
            case (x)
                5'd24,5'd23: begin pc=5'd24; base=0;   mr=25'h1fffe00; ml=25'h1fffe00; desc=1; end
                5'd22,5'd21: begin pc=5'd22; base=32;  mr=25'h1fffe00; ml=25'h1fffe00; desc=0; end
                5'd20,5'd19: begin pc=5'd20; base=64;  mr=25'h1e0fe00; ml=25'h1e0fe00; desc=1; end
                5'd18,5'd17: begin pc=5'd18; base=86;  mr=25'h1e0fe00; ml=25'h1e0fe00; desc=0; end
                5'd16,5'd15: begin pc=5'd16; base=108; mr=25'h1e0ffbf; ml=25'h1ffffbf; desc=1; end
                5'd14,5'd13: begin pc=5'd14; base=151; mr=25'h1ffffbf; ml=25'h1ffffbf; desc=0; end
                5'd12,5'd11: begin pc=5'd12; base=199; mr=25'h1ffffbf; ml=25'h1ffffbf; desc=1; end
                5'd10,5'd9:  begin pc=5'd10; base=247; mr=25'h1ffffbf; ml=25'h1ffffbf; desc=0; end
                5'd8,5'd7:   begin pc=5'd8;  base=295; mr=25'h001fe00; ml=25'h001fe00; desc=1; end
                5'd5,5'd4:   begin pc=5'd5;  base=311; mr=25'h001fe00; ml=25'h001fe00; desc=0; end
                5'd3,5'd2:   begin pc=5'd3;  base=327; mr=25'h001fe00; ml=25'h001fe00; desc=1; end
                default:     begin pc=5'd1;  base=343; mr=25'h001fe00; ml=25'h001fe00; desc=0; end
            endcase
            both = mr | ml;
            cnt = 0;
            if (desc) begin
                for (i=24; i>=0; i=i-1)
                    if (i > y) cnt = cnt + mr[i] + ml[i];
            end else begin
                for (i=0; i<25; i=i+1)
                    if (i < y) cnt = cnt + mr[i] + ml[i];
            end
            if ((x == pc-1) && mr[y]) cnt = cnt + 1;
            data_index = base + cnt;
        end
    endfunction

    function qr_mask_hit;
        input [4:0] x, y;
        input [2:0] mask;
        reg [9:0] p;
        begin
            p = x*y;
            case (mask)
                3'd0: qr_mask_hit = ((x+y)&1)==0;
                3'd1: qr_mask_hit = (y&1)==0;
                3'd2: qr_mask_hit = (x%3)==0;
                3'd3: qr_mask_hit = ((x+y)%3)==0;
                3'd4: qr_mask_hit = (((y>>1)+(x/3))&1)==0;
                3'd5: qr_mask_hit = ((p&1)+(p%3))==0;
                3'd6: qr_mask_hit = ((((p&1)+(p%3))&1)==0);
                default: qr_mask_hit = (((p%3)+((x+y)&1))&1)==0;
            endcase
        end
    endfunction

    function qr_module;
        input [4:0] x, y;
        input [2:0] mask;
        reg [8:0] di;
        reg raw;
        begin
            if (!qr_is_function(x,y)) begin
                di = data_index(x,y);
                if (di < 9'd352)
                    raw = cw[di[8:3]][7-di[2:0]];
                else
                    raw = 1'b0; // 7 QR v2 remainder bits
                qr_module = raw ^ qr_mask_hit(x,y,mask);
            end else begin
                qr_module = qr_function_value(x,y,ec_sel,mask);
            end
        end
    endfunction

    // Payload bytes: 0100 + 8-bit length + 64 payload bits + terminator.
    function [7:0] payload_codeword;
        input [5:0] n;
        begin
            case (n)
                6'd0: payload_codeword = 8'h40;
                6'd1: payload_codeword = {4'h8,payload[63:60]};
                6'd2: payload_codeword = payload[59:52];
                6'd3: payload_codeword = payload[51:44];
                6'd4: payload_codeword = payload[43:36];
                6'd5: payload_codeword = payload[35:28];
                6'd6: payload_codeword = payload[27:20];
                6'd7: payload_codeword = payload[19:12];
                6'd8: payload_codeword = payload[11:4];
                6'd9: payload_codeword = {payload[3:0],4'h0};
                default: payload_codeword = 8'h00;
            endcase
        end
    endfunction

    // One GF multiplier is reused for Reed-Solomon ECC.  This costs more
    // cycles than a fully parallel LFSR, but is much friendlier to a Tiny Tapeout tile.
    wire [7:0] rs_feedback = (rs_index == 0) ? (cw[data_index_r] ^ parity[0]) : rs_feedback_r;

    // Scoring increments for the current real module.
    wire score_bit = qr_module(score_x,score_y,score_mask);
    wire score_same = (score_x != 0) && (score_bit == score_prev);
    wire [5:0] score_run_next = score_same ? score_run + 6'd1 : 6'd1;
    wire [5:0] score_n1_inc = (score_x != 0 && score_same && score_run_next >= 6'd5) ?
                               ((score_run_next == 6'd5) ? 6'd3 : 6'd1) : 6'd0;
    wire score_n2_hit = (!score_col_pass && (score_x != 0) && (score_y != 0) &&
                         (score_bit == score_row_prev[score_x]) &&
                         (score_bit == score_row_cur[score_x-1]) &&
                         (score_bit == score_row_prev[score_x-1]));
    wire [5:0] score_n2_inc = score_n2_hit ? 6'd3 : 6'd0;
    wire [10:0] score_win_next = {score_win[9:0],score_bit};
    wire score_n3_hit = (score_x >= 5'd10) && !score_n3_skip &&
                        ((score_win_next == 11'b10111010000) ||
                         (score_win_next == 11'b00001011101));
    wire [5:0] score_n3_inc = score_n3_hit ? 6'd40 : 6'd0;
    wire [15:0] score_pen_next = score_pen + score_n1_inc + score_n2_inc + score_n3_inc;
    wire [10:0] dark_total = dark_count + score_bit;

    function [7:0] n4_penalty;
        input [9:0] dark;
        integer diff, k, j;
        begin
            diff = (dark*100 >= 31250) ? (dark*100-31250) : (31250-dark*100);
            k = 0;
            for (j=0; j<11; j=j+1) begin
                if (diff >= 3125) begin diff=diff-3125; k=k+1; end
            end
            n4_penalty = k*10;
        end
    endfunction

    // ---------------------------------------------------------------------
    // Control / encoder / mask scorer
    // ---------------------------------------------------------------------
    integer ri;
    always @(posedge clk) begin
        load_d <= uio_in[5];
        go_d   <= uio_in[6];

        if (!rst_n) begin
            payload <= 64'h4c59524120504841; // "LYRA PHA" default demo payload
            load_index <= 0;
            payload_ready <= 1'b1;
            ec_sel <= 2'd0;
            mask_sel <= 3'd0;
            selected_mask <= 3'd0;
            display_ready <= 1'b0;
            data_count <= 0;
            ec_count <= 0;
            data_index_r <= 0;
            rs_index <= 0;
            rs_feedback_r <= 0;
            ec_index <= 0;
            score_pen <= 0;
            dark_count <= 0;
            score_mask <= 0;
            score_x <= 0;
            score_y <= 0;
            score_col_pass <= 0;
            score_prev <= 0;
            score_run <= 0;
            score_row_prev <= 0;
            score_row_cur <= 0;
            score_win <= 0;
            score_n3_skip <= 1'b0;
            best_pen <= 16'hffff;
            best_mask <= 0;
            state <= ST_START;
            for (ri=0; ri<44; ri=ri+1) cw[ri] <= 0;
            for (ri=0; ri<28; ri=ri+1) parity[ri] <= 0;
        end else begin
            case (state)
                ST_START: begin
                    case (ec_sel)
                        2'd0: begin data_count<=6'd34; ec_count<=6'd10; end
                        2'd1: begin data_count<=6'd28; ec_count<=6'd16; end
                        2'd2: begin data_count<=6'd22; ec_count<=6'd22; end
                        default: begin data_count<=6'd16; ec_count<=6'd28; end
                    endcase
                    data_index_r <= 0;
                    state <= ST_BUILD;
                end

                ST_BUILD: begin
                    if (data_index_r < 6'd10)
                        cw[data_index_r] <= payload_codeword(data_index_r);
                    else if (data_index_r < data_count)
                        cw[data_index_r] <= ((data_index_r-10) & 1) ? 8'h11 : 8'hec;

                    if (data_index_r == data_count-1) begin
                        for (ri=0; ri<28; ri=ri+1) parity[ri] <= 0;
                        data_index_r <= 0;
                        rs_index <= 0;
                        state <= ST_RS;
                    end else begin
                        data_index_r <= data_index_r + 1;
                    end
                end

                ST_RS: begin
                    if (rs_index == 0) rs_feedback_r <= rs_feedback;
                    if (rs_index < ec_count-1)
                        parity[rs_index] <= parity[rs_index+1] ^ gf_mul(rs_feedback,gen_coeff(ec_sel,rs_index+1));
                    else
                        parity[rs_index] <= gf_mul(rs_feedback,gen_coeff(ec_sel,ec_count));

                    if (rs_index == ec_count-1) begin
                        ec_index <= 0;
                        if (data_index_r == data_count-1) begin
                            state <= ST_RS_APPEND;
                        end else begin
                            data_index_r <= data_index_r + 1;
                            rs_index <= 0;
                        end
                    end else begin
                        rs_index <= rs_index + 1;
                    end
                end

                ST_RS_APPEND: begin
                    cw[data_count + ec_index] <= parity[ec_index];
                    if (ec_index == ec_count-1) begin
                        if (mask_sel != 0) begin
                            selected_mask <= mask_sel;
                            display_ready <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            score_mask <= 0;
                            state <= ST_SCORE_INIT;
                        end
                    end else begin
                        ec_index <= ec_index + 1;
                    end
                end

                ST_SCORE_INIT: begin
                    score_pen <= 0;
                    dark_count <= 0;
                    score_x <= 0;
                    score_y <= 0;
                    score_col_pass <= 0;
                    score_prev <= 0;
                    score_run <= 0;
                    score_row_prev <= 0;
                    score_row_cur <= 0;
                    score_win <= 0;
                    score_n3_skip <= 1'b0;
                    if (score_mask == 0) begin
                        best_pen <= 16'hffff;
                        best_mask <= 0;
                    end
                    state <= ST_SCORE_RUN;
                end

                ST_SCORE_RUN: begin
                    score_pen <= score_pen_next;
                    score_win <= score_win_next;

                    if (score_x >= 5'd10) begin
                        if (!score_n3_skip && score_n3_hit)
                            score_n3_skip <= (score_win_next == 11'b00001011101);
                        else if (score_n3_skip)
                            score_n3_skip <= 1'b0;
                    end

                    score_prev <= score_bit;
                    score_run <= score_same ? score_run_next : 6'd1;

                    if (!score_col_pass) begin
                        score_row_cur[score_x] <= score_bit;
                        if (score_bit) dark_count <= dark_count + 1;
                    end

                    if (score_x == 5'd24) begin
                        if (!score_col_pass) begin
                            score_row_prev <= score_row_cur;
                            // Include the final bit of this row when moving to the next row.
                            score_row_prev[24] <= score_bit;
                            score_row_cur <= 0;
                            score_x <= 0;
                            score_y <= score_y + 1;
                            score_prev <= 0;
                            score_run <= 0;
                            score_win <= 0;
                            score_n3_skip <= 1'b0;
                            if (score_y == 5'd24) begin
                                score_col_pass <= 1;
                                score_x <= 0;
                                score_y <= 0;
                                score_prev <= 0;
                                score_run <= 0;
                                score_win <= 0;
                                score_n3_skip <= 1'b0;
                            end
                        end else begin
                            // score_pen_next contains every N1/N2/N3 contribution
                            // from the final module of this candidate.
                            if (score_mask == 3'd7) begin
                                if ((score_pen_next + n4_penalty(dark_count)) < best_pen) begin
                                    best_pen <= score_pen_next + n4_penalty(dark_count);
                                    best_mask <= score_mask;
                                    selected_mask <= score_mask;
                                end else begin
                                    selected_mask <= best_mask;
                                end
                                display_ready <= 1'b1;
                                state <= ST_IDLE;
                            end else begin
                                if ((score_pen_next + n4_penalty(dark_count)) < best_pen) begin
                                    best_pen <= score_pen_next + n4_penalty(dark_count);
                                    best_mask <= score_mask;
                                end
                                score_mask <= score_mask + 1;
                                state <= ST_SCORE_INIT;
                            end
                        end
                    end else begin
                        score_x <= score_x + 1;
                    end
                end

                ST_IDLE: begin
                    if (go_rise && payload_ready) begin
                        ec_sel <= uio_in[1:0];
                        mask_sel <= uio_in[4:2];
                        display_ready <= 1'b0;
                        state <= ST_START;
                    end
                end

                default: state <= ST_START;
            endcase

            if (load_rise) begin
                payload <= {payload[55:0],ui_in};
                if (load_index == 4'd7) begin
                    load_index <= 0;
                    payload_ready <= 1'b1;
                end else begin
                    load_index <= load_index + 1;
                    if (load_index == 0) payload_ready <= 1'b0;
                end
            end
        end
    end

endmodule
`default_nettype wire
