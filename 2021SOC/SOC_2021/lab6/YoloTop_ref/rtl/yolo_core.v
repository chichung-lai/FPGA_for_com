// ============================================================================
// Copyright (C) 2019 NARLabs TSRI. All rights reserved.
//
// Designer : Liu Yi-Jun
// Date     : 2019.10.31
// Ver      : 1.2
// Module   : yolo_core
// Func     : 
//            1.) Bypass  
//            2.) adder: incoming every two operand, output one result
//
//
// ============================================================================

`timescale 1 ns / 1 ps

module yolo_core #(
        parameter TBITS = 32 ,
        parameter TBYTE = 4
) (

        //
        input  wire [TBITS-1:0] isif_data_dout ,  // {last,user,strb,data}
        input  wire [TBYTE-1:0] isif_strb_dout ,  //non
        input  wire [1 - 1:0]   isif_last_dout ,  // 
        input  wire [1 - 1:0]   isif_user_dout ,  // 
        input  wire             isif_empty_n , //有值是1
        output wire             isif_read ,

        //
        output wire [TBITS-1:0] osif_data_din ,
        output wire [TBYTE-1:0] osif_strb_din , //強制全給1
        output wire [1 - 1:0]   osif_last_din ,
        output wire [1 - 1:0]   osif_user_din ,
        input  wire             osif_full_n ,
        output wire             osif_write ,
        output reg [5:0]        w_cnt,
        output wire [2:0]       w_CS,

        //
        input  wire             rst ,
        input  wire             clk
);  // filter_core


// Parameter
localparam height = 8;
localparam width = 8;
localparam pi =  32'b0000000_0011001001000011111101101; //pi/(2*8)=0.19634953141212463

localparam pi2 = 32'b0000011_0010010000111111011010101; //pi=3.14

localparam c00 = 32'b0_0010000000000000000000000000000; // 1/(4(√2*√2))
localparam c10 = 32'b0_0010110101000001001111001100110; //  1/(4√2)
localparam c11 = 32'b0_0100000000000000000000000000000; // 1/(4)



// state
reg [2:0] CS;
localparam         IDLE   = 0;  // zero or second op received
localparam         GET    = 1;  // first op received
localparam         CAL    = 2;
localparam         WRITE  = 3;
localparam         FINISH = 4;
// ============================================================================
// local signal
//
// 
// ============================================================================
// Body
//
reg  [2:0]col,i;
reg  [31:0] col_32,row_32,i_32,j_32;
reg  [2:0]row,j;
reg signed [31:0] matrix [0:63];
reg signed [31:0] matrix_ans [0:63];
reg  [5:0] cnt; 
wire [TBITS-1:0] result;
reg read,write_tri;
reg gocal,goget;
reg [5:0] matrix_cnt;
reg cal_stop;
reg signed [TBITS-1:0] w_result;
reg stop;
wire [3:0] col_2,col_2_1;
wire [3:0] row_2,row_2_1;
wire [31:0] col_i,cosA_fix;
wire [6:0] test,test2;
wire [31:0] row_j,cosB_fix;
wire [63:0] cosA,cosB;
wire [31:0] returnA_1,returnA_2,returnA_3,returnA_4,returnA_5,returnA_6;
wire [31:0] returnB_1,returnB_2,returnB_3,returnB_4,returnB_5,returnB_6;
wire [31:0] cosA_fix_re,cosB_fix_re,cosA_out_neg,cosB_out_neg;
wire [2:0] cosA_tri,cosB_tri;
wire [31:0] cosA_fix_re_cos,cosB_fix_re_cos;
wire signed [31:0] cosA_out_cor,cosB_out_cor;
wire signed [63:0] cosxcos_64;
wire signed [31:0] cosxcos_32;
wire [0:0] sincosA_valid,sincosB_valid;
wire signed [31:0] cosA_out,cosB_out;
wire [63:0] sincosA_data;
wire [63:0] sincosB_data;
wire [5:0] matrix_add_cr,matrix_add;
wire [31:0] added;
wire add_sign;
wire [31:0] c;
wire signed [31:0] DCT;
/*pipeline reg*/
reg [3:0] pip_col_2,pip_row_2;
reg [3:0] pip_col_2_1,pip_row_2_1;
reg [6:0] pip_test ,pip_test2;
reg [31:0] pip_col_i,pip_row_j;
reg [63:0] pip_cosA,pip_cosB;
reg [31:0] pip_returnA_1,pip_returnA_2,pip_returnA_3,pip_returnA_4,pip_returnA_5,pip_returnA_6,pip_cosA_fix;
reg [31:0] pip_returnB_1,pip_returnB_2,pip_returnB_3,pip_returnB_4,pip_returnB_5,pip_returnB_6,pip_cosB_fix;
reg [31:0] pip_cosA_fix_re,pip_cosB_fix_re;
reg [2:0] pip_cosA_tri,pip_cosB_tri;
reg [31:0] pip_cosA_fix_re_cos,pip_cosB_fix_re_cos;
reg [31:0] pip_cosA_out,pip_cosB_out;
reg [31:0] pip_cosA_out_neg,pip_cosB_out_neg;
reg signed [31:0] pip_cosA_out_cor,pip_cosB_out_cor;
reg signed [31:0] pip_cosxcos_32;
reg signed [63:0] mult_res;
reg signed [31:0] add_sum;
reg signed [31:0] pip_c;
reg signed [63:0] DCT_process;
reg write_last;
reg gofinish;
integer a;
//assign xfer_en = isif_empty_n & osif_full_n ;

//assign opA = isif_data_dout ;

//assign result = opA + opB ;

assign osif_data_din = w_result ;

//assign osif_write = cnt[0] & xfer_en ;

assign isif_read  = read ;
assign osif_write = write_tri;

assign osif_last_din =  write_last ;

assign osif_user_din = 0 ;

assign osif_strb_din = {TBYTE{1'b1}};

assign matrix_add = 8*i+j;

assign added = {{3{mult_res[63]}}, mult_res[63:35]}; //13bit整數 19bit小數

assign DCT = DCT_process[62:31];

assign W_CS = CS;
        
always @(posedge clk,posedge rst)
begin
    if(rst)
    begin
        CS <= 0;
    end
    else begin
        case(CS)
            IDLE  : if(isif_empty_n)
                        CS <= GET;
                    else
                        CS <= IDLE;
            GET   : if(gocal)
                        CS <= CAL;
                    else
                        CS <= GET;
            CAL   : if(cal_stop)
                        CS <= WRITE;
                    else
                        CS <= CAL;
            WRITE : if(gofinish)
                        CS <= FINISH;
                    else
                        CS <= WRITE;
            FINISH :if(stop)
                        CS <= IDLE;
                    else
                        CS <= FINISH;
        endcase
    end
end
/*-------------------control------------------------*/
always @(posedge clk,posedge rst)
begin
     if(rst)
    begin
        read <= 0;
        write_tri <= 0;
        stop <= 0;
    end
    else begin
        case(CS)
            IDLE:   begin
                        read <= 0;
                        write_tri <= 0;
                        stop <= 0;
                    end
            GET :   begin
                        read <= 1;
                    end
            CAL :   begin
                        read <= 0;
                    end
            WRITE : begin
                        if(write_last)
                            write_tri <= 0;
                        else
                            write_tri <= 1;
                    end
            FINISH :begin
                        write_tri <= 0;
                        stop <= 1;
                    end
                    
        endcase
     end
end

/*-------------------data,matrix--------------------------*/
always @(posedge clk,posedge rst)
begin
    if(rst)
    begin
        matrix_cnt <= 0;
        gocal <= 0;
        add_sum <= 0;
        DCT_process <= 0;
        w_result <= 0;
    end
    else begin
        case(CS)
            IDLE:   begin
                        for(a=0;a<64;a=a+1)
		                begin
			                 matrix[a]<=0;
			                 matrix_ans[a]<=0;
		                end
                    end
            GET :   begin
                        if(read & isif_empty_n)
                        begin
                            matrix[matrix_cnt] <= isif_data_dout;
                            if(matrix_cnt==62)
						    begin
							    matrix_cnt <= 63;
							    gocal <= 1;
						    end
						    else begin	
							    matrix_cnt <= matrix_cnt+1;
					        end 
					    end
					    else begin
					           matrix_cnt<= matrix_cnt+0;
					    end
                    end
            CAL :   begin
                        if(cnt == 60)begin
                          add_sum <= matrix_ans[matrix_add] + added;
                        end
                        else if (cnt == 61)begin
                           matrix_ans[matrix_add] <= add_sum;
                           end
                        else if (col==7 && row==7 && cnt==62)begin
                          DCT_process <= matrix_ans[matrix_add] * pip_c;
                          end
                        else if (col==7 && row==7 && cnt==63)begin
                          matrix_ans[matrix_add] <= DCT;
                          end
                        else if (cnt == 1)begin
                            DCT_process <= 0;
                        end
                    end
            WRITE : begin
                        if(osif_full_n == 1)begin
                            w_result <= matrix_ans[w_cnt];
                        end
                        else begin
                            w_result <= w_result;
                        end
                    end
        endcase            
    end
end
/*----------------------------------control col ,row------------------------------------------------*/
always @(posedge clk,posedge rst)
begin
    if(rst)
    begin
        col <= 0;
        row <= 0;
        i <= 0;
        j <= 0;
        cal_stop <= 0;
        cnt <= 0;
    end
    else if(CS == CAL && cal_stop == 0)
    begin
        if(i==7 && j==7 && col==7 && row==7 && cnt==63)
        begin
            cal_stop <= 1;
        end
        else if (i!=7 && j==7 && col==7 && row==7 && cnt==63)
        begin
            i <= i+1;
            j <= 0;
            cnt <= 0;
            row <= 0;
            col <= 0;
        end
        else if (col==7 && row==7 && cnt==63)
        begin
            j <= j + 1;
            row <= 0;
            col <= 0;
            cnt <= 0;
        end
        else if(col!=7 && row == 7 && cnt==63)
        begin
            col <= col+1;
            row <= 0;
            cnt <= 0;
        end
        else if(cnt==63) //調整lantancy 記得條register和else if判斷式
        begin
            row <= row+1;
            cnt <= 0;
        end
        else begin
            cnt<=cnt+1;
        end         
    end
end

/*                  pi*col*i, pi*row*j                   */
//-----------------------------------第一級-------------------------------------------
assign col_2 = col<<1;
assign row_2 = row<<1;

always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_col_2 <= 0;
        pip_row_2 <= 0;
        end
    else if(CS==CAL && cnt == 1)begin
        pip_col_2 <= col_2;
        pip_row_2 <= row_2;
    end
    else begin
        pip_col_2 <= pip_col_2;
        pip_row_2 <= pip_row_2;
    end
end
//-----------------------------------第二級-------------------------------------------
assign col_2_1 = pip_col_2 +1;
assign row_2_1 = pip_row_2 +1;

assign c = (i != 0 && j != 0)?c11:
           (i == 0 && j == 0)?c00:c10;
           
always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_col_2_1 <= 0;
        pip_row_2_1 <= 0;
        pip_c <= 0;
        end
    else if(CS==CAL && cnt == 2)begin
        pip_col_2_1 <= col_2_1;
        pip_row_2_1 <= row_2_1;
        pip_c <= c;
    end
    else begin
        pip_col_2_1 <= pip_col_2_1;
        pip_row_2_1 <= pip_row_2_1;
        pip_c <= pip_c;      
    end
end
//-----------------------------------第三級-------------------------------------------
assign test = pip_col_2_1*i;  //最高7bit
assign test2 = pip_row_2_1*j;

always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_test <= 0;
        pip_test2 <= 0;
        end
    else if(CS==CAL && cnt == 3)begin
        pip_test <= test;
        pip_test2 <= test2;
    end
    else begin
        pip_test <= pip_test; 
        pip_test2 <= pip_test2; 
    end
end
//-----------------------------------第四級--------------------------------------------------
assign col_i = {{pip_test},{25'b0000000000000000000000000}};
assign row_j = {{pip_test2},{25'b0000000000000000000000000}};

always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_col_i <= 0;
        pip_row_j <= 0;
        end
    else if(CS==CAL && cnt == 4)begin
        pip_col_i <= col_i;
        pip_row_j <= row_j;
    end
   else begin
        pip_col_i <= pip_col_i;
        pip_row_j <= pip_row_j;
    end
end
//-----------------------------------第五級--------------------------------------------------
assign cosA = pi * pip_col_i;
assign cosB = pi * pip_row_j;

always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_cosA <= 0;
        pip_cosB <= 0;
        end
    else if(CS==CAL && cnt == 5)begin
        pip_cosA <= cosA;
        pip_cosB <= cosB;
    end
    else begin
        pip_cosA <= pip_cosA;
        pip_cosB <= pip_cosB;
    end
end
//-----------------------------------第六級--------------------------------------------------

assign cosA_fix = pip_cosA[56:25];
assign cosB_fix = pip_cosB[56:25];
//             把角度校正回歸到-3.14~3.14
assign returnA_1 = cosA_fix - 32'b0000011_0010010000111111011010101; 
assign returnA_2 = cosA_fix - 32'b0000110_0100100001111110110101010;
assign returnA_3 = cosA_fix - 32'b0001001_0110110010111110001111111;
assign returnA_4 = cosA_fix - 32'b0001100_1001000011111101101010100;
assign returnA_5 = cosA_fix - 32'b0001111_1011010100111101000101001;
assign returnA_6 = cosA_fix - 32'b0010010_1101100101111100011111110;

assign returnB_1 = cosB_fix - 32'b0000011_0010010000111111011010101;
assign returnB_2 = cosB_fix - 32'b0000110_0100100001111110110101010;
assign returnB_3 = cosB_fix - 32'b0001001_0110110010111110001111111;
assign returnB_4 = cosB_fix - 32'b0001100_1001000011111101101010100;
assign returnB_5 = cosB_fix - 32'b0001111_1011010100111101000101001;
assign returnB_6 = cosB_fix - 32'b0010010_1101100101111100011111110;

always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_returnA_1 <= 0;
        pip_returnA_2 <= 0;
        pip_returnA_3 <= 0;
        pip_returnA_4 <= 0;
        pip_returnA_5 <= 0;
        pip_returnA_6 <= 0;
        pip_returnB_1 <= 0;
        pip_returnB_2 <= 0;
        pip_returnB_3 <= 0;
        pip_returnB_4 <= 0;
        pip_returnB_5 <= 0;
        pip_returnB_6 <= 0;
        pip_cosA_fix <= 0;
        pip_cosB_fix <= 0;
        end
    else if(CS==CAL && cnt == 6)begin
        pip_returnA_1 <= returnA_1;
        pip_returnA_2 <= returnA_2;
        pip_returnA_3 <= returnA_3;
        pip_returnA_4 <= returnA_4;
        pip_returnA_5 <= returnA_5;
        pip_returnA_6 <= returnA_6;
        pip_returnB_1 <= returnB_1;
        pip_returnB_2 <= returnB_2;
        pip_returnB_3 <= returnB_3;
        pip_returnB_4 <= returnB_4;
        pip_returnB_5 <= returnB_5;
        pip_returnB_6 <= returnB_6;
        pip_cosA_fix <= cosA_fix;
        pip_cosB_fix <= cosB_fix;
    end
    else begin
        pip_returnA_1 <= pip_returnA_1;
        pip_returnA_2 <= pip_returnA_2;
        pip_returnA_3 <= pip_returnA_3;
        pip_returnA_4 <= pip_returnA_4;
        pip_returnA_5 <= pip_returnA_5;
        pip_returnA_6 <= pip_returnA_6;
        pip_returnB_1 <= pip_returnB_1;
        pip_returnB_2 <= pip_returnB_2;
        pip_returnB_3 <= pip_returnB_3;
        pip_returnB_4 <= pip_returnB_4;
        pip_returnB_5 <= pip_returnB_5;
        pip_returnB_6 <= pip_returnB_6;
        pip_cosA_fix <= pip_cosA_fix; 
        pip_cosB_fix <= pip_cosB_fix; 
    end
end
//-----------------------------------第七級--------------------------------------------------

assign cosA_fix_re = (pip_cosA_fix < pi2)? pip_cosA_fix :
                     (pip_returnA_1 < pi2)? pip_returnA_1 :
                     (pip_returnA_2 < pi2)? pip_returnA_2 :
                     (pip_returnA_3 < pi2)? pip_returnA_3 :
                     (pip_returnA_4 < pi2)? pip_returnA_4 :
                     (pip_returnA_5 < pi2)? pip_returnA_5 : pip_returnA_6;
                     
                     
assign cosB_fix_re = (pip_cosB_fix < pi2)? pip_cosB_fix :
                     (pip_returnB_1 < pi2)? pip_returnB_1 :
                     (pip_returnB_2 < pi2)? pip_returnB_2 :
                     (pip_returnB_3 < pi2)? pip_returnB_3 :
                     (pip_returnB_4 < pi2)? pip_returnB_4 :
                     (pip_returnB_5 < pi2)? pip_returnB_5 : pip_returnB_6;
                     
assign cosA_tri =    (pip_cosA_fix < pi2)? 0 :
                     (pip_returnA_1 < pi2)? 1 :
                     (pip_returnA_2 < pi2)? 2 :
                     (pip_returnA_3 < pi2)? 3 :
                     (pip_returnA_4 < pi2)? 4 :
                     (pip_returnA_5 < pi2)? 5 : 6;
                     
assign cosB_tri =    (pip_cosB_fix < pi2)? 0 :
                     (pip_returnB_1 < pi2)? 1 :
                     (pip_returnB_2 < pi2)? 2 :
                     (pip_returnB_3 < pi2)? 3 :
                     (pip_returnB_4 < pi2)? 4 :
                     (pip_returnB_5 < pi2)? 5 : 6;

always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_cosA_fix_re <= 0;
        pip_cosB_fix_re <= 0;
        pip_cosA_tri <= 0;
        pip_cosB_tri <= 0;
        end
    else if(CS==CAL && cnt == 7)begin
        pip_cosA_fix_re <= cosA_fix_re;
        pip_cosB_fix_re <= cosB_fix_re;
        pip_cosA_tri <= cosA_tri;
        pip_cosB_tri <= cosB_tri;

    end
    else begin
        pip_cosA_fix_re <= pip_cosA_fix_re;
        pip_cosB_fix_re <= pip_cosB_fix_re;
        pip_cosA_tri <= pip_cosA_tri;   
        pip_cosB_tri <= pip_cosB_tri;   
    end
end          

//-----------------------------------第八級--------------------------------------------------                    

assign cosA_fix_re_cos = {pip_cosA_fix_re[27:0],4'b0000};//32改成16要改這，記得pip_reg也要改
assign cosB_fix_re_cos = {pip_cosB_fix_re[27:0],4'b0000};

always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_cosA_fix_re_cos <= 0;
        end
    else if(CS==CAL && cnt == 8)begin
        pip_cosA_fix_re_cos <= cosA_fix_re_cos;
        end
    else begin
        pip_cosA_fix_re_cos <= pip_cosA_fix_re_cos;
    end
end

always@(posedge clk,posedge rst)
begin
    if(rst)begin
        pip_cosB_fix_re_cos <= 0;
    end
    else if(CS==CAL && cnt == 8)begin
        pip_cosB_fix_re_cos <= cosB_fix_re_cos;
    end
    else begin
        pip_cosB_fix_re_cos <= pip_cosB_fix_re_cos;
    end
end
//-----------------------------------第九級--------------------------------------------------      

assign cosA_out = sincosA_data[31:0];
assign cosB_out = sincosB_data[31:0];

cordic_0 sincosA(                    
        .aclk(clk),
        .s_axis_phase_tdata(pip_cosA_fix_re_cos),
        .s_axis_phase_tvalid(1),
        .m_axis_dout_tdata(sincosA_data), // Y_OUT, X_OUT
        .m_axis_dout_tvalid(sincosA_valid)
    );

cordic_1 sincosB(                    
        .aclk(clk),
        .s_axis_phase_tdata(pip_cosB_fix_re_cos),
        .s_axis_phase_tvalid(1),
        .m_axis_dout_tdata(sincosB_data), // Y_OUT, X_OUT
        .m_axis_dout_tvalid(sincosB_valid)
    );
    
always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_cosA_out <= 0;
        end
    else if(CS==CAL && cnt==55)
    begin
        pip_cosA_out <= cosA_out;
    end
    else begin
        pip_cosA_out <= pip_cosA_out;
    end
end

always@(posedge clk ,posedge rst)
begin
    if(rst)begin
        pip_cosB_out <= 0;
        end
    else if(CS==CAL && cnt==55)
    begin
        pip_cosB_out <= cosB_out;
    end
    else begin
        pip_cosB_out <= pip_cosB_out;
    end
end
//-----------------------------------第十級--------------------------------------------------             
assign cosA_out_neg = ~pip_cosA_out+1;
assign cosB_out_neg = ~pip_cosB_out+1;

always@(posedge clk, posedge rst)
begin
    if(rst)begin
        pip_cosA_out_neg <= 0;
        pip_cosB_out_neg <= 0;
    end
    else if(CS==CAL && cnt == 56)
    begin
        pip_cosA_out_neg <= cosA_out_neg;
        pip_cosB_out_neg <= cosB_out_neg;
    end
    else begin
        pip_cosA_out_neg <= pip_cosA_out_neg; 
        pip_cosB_out_neg <= pip_cosB_out_neg; 
    end
end
//-----------------------------------第十一級--------------------------------------------------       
assign cosA_out_cor = (pip_cosA_tri == 1)?pip_cosA_out_neg:
                      (pip_cosA_tri == 3)?pip_cosA_out_neg:
                      (pip_cosA_tri == 5)?pip_cosA_out_neg:pip_cosA_out;
                    
assign cosB_out_cor = (pip_cosB_tri == 1)?pip_cosB_out_neg:
                      (pip_cosB_tri == 3)?pip_cosB_out_neg:
                      (pip_cosB_tri == 5)?pip_cosB_out_neg:pip_cosB_out;

always@(posedge clk, posedge rst)
begin
    if(rst)begin
        pip_cosA_out_cor <= 0;
        pip_cosB_out_cor <= 0;
    end
    else if(CS==CAL && cnt == 57)
    begin
         pip_cosA_out_cor <= cosA_out_cor;
         pip_cosB_out_cor <= cosB_out_cor;
    end
    else begin
        pip_cosA_out_cor <= pip_cosA_out_cor;
        pip_cosB_out_cor <= pip_cosB_out_cor;
    end
end
//-----------------------------------第十二級--------------------------------------------------                  
assign cosxcos_64 = pip_cosA_out_cor * pip_cosB_out_cor;
assign cosxcos_32 = cosxcos_64[61:30];

always@(posedge clk, posedge rst)
begin
    if(rst)begin
        pip_cosxcos_32 <= 0;
    end
    else if(CS==CAL && cnt == 58)
    begin
         pip_cosxcos_32 <= cosxcos_32;
    end
    else begin
        pip_cosxcos_32 <= pip_cosxcos_32;
    end
end
//------------------------------------------------------------------------------------------------       
assign matrix_add_cr = 8*col+row;
always@(posedge clk,posedge rst)
begin
    if(rst)begin
        mult_res <= 0;
    end
    else if(CS==CAL && cnt==59)
    begin
        mult_res <= matrix[matrix_add_cr]*pip_cosxcos_32;
    end
    else begin
        mult_res <= mult_res;
    end
end
//------------------------------------------------------------------------------------------------    
always@(posedge clk,posedge rst)begin
    if(rst)begin
        w_cnt <= 0;
        gofinish <= 0;
        write_last <= 0;
    end
    else if(CS==WRITE)begin
        if(w_cnt == 63)begin
           gofinish <= 1;
           write_last <= 1;
        end
        else begin
            w_cnt <= w_cnt+1;
        end
    end
end

endmodule