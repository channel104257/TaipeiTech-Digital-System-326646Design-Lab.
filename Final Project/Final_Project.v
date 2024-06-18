module Final_Project(Dbus, calcBT, CLK, CalcFinish, seg7_7, seg7_6, seg7_5, seg7_4, seg7_3, seg7_2, seg7_1, seg7_0, CalcState, bcd_R, calcResult);

	input 			[15:0] Dbus;
	input				[3:0]	 calcBT;
	input 			 		 CLK;
	output reg		[1:0]  CalcState;
	output reg		 		 CalcFinish = 0;
	output			[6:0]	 seg7_7;
	output			[6:0]	 seg7_6;
	output			[6:0]	 seg7_5;
	output			[6:0]	 seg7_4;
	output			[6:0]	 seg7_3;
	output			[6:0]	 seg7_2;
	output			[6:0]	 seg7_1;
	output			[6:0]	 seg7_0;
	
	wire 		[15:0] Out_A;
	wire 		[15:0] Out_S;
	wire				 Co_A, Co_S;
	wire		[31:0] Product;
	wire 		[15:0] Quotient;
	wire 		[15:0] Remainder;
	wire				 Done, V, Rdy;
	wire		[39:0] bcd_in;
	output		[39:0] bcd_R;
	wire		[39:0] bcd_RegA;
	wire		[39:0] bcd_RegB;
	wire		[3:0]  calcBT_De;
	wire				 CLK_Div;
	output reg		[31:0] calcResult = 0;
	reg		[15:0] RegA = 0;
	reg		[15:0] RegB = 0;
	reg		[2:0]  State = 0; 
	reg				 St = 0;
	reg		[39:0] bcd_out = 0;

	CLA16 Adder(RegA, RegB, 1'b0, Out_A, Co_A);
	CLA16 Subtractor(RegA, ~RegB, 1'b1, Out_S, Co_S);
	Mult16(CLK, St, RegA, RegB, CalcFinish, Product, Done);
	Div16 (CLK, St, RegA, RegB, Quotient, Remainder, V, CalcFinish, Rdy);
	
	bin2bcd bcd_result(calcResult, bcd_R);
	bin2bcd bcd_Dbus({16'b0, Dbus}, bcd_in);
	
	BCD2segment7 s7_0(bcd_out[3:0], seg7_0);
	BCD2segment7 s7_1(bcd_out[7:4], seg7_1);
	BCD2segment7 s7_2(bcd_out[11:8], seg7_2);
	BCD2segment7 s7_3(bcd_out[15:12], seg7_3);
	BCD2segment7 s7_4(bcd_out[19:16], seg7_4);
	BCD2segment7 s7_5(bcd_out[23:20], seg7_5);
	BCD2segment7 s7_6(bcd_out[27:24], seg7_6);
	BCD2segment7 s7_7(bcd_out[31:28], seg7_7);
	
//	div50M Pre_Div(CLK, 1'b1, CLK_Div);
	KEY_Debounce Bt1(CLK, 1'b1, calcBT[0], calcBT_De[0]);
	KEY_Debounce Bt2(CLK, 1'b1, calcBT[1], calcBT_De[1]);
	KEY_Debounce Bt3(CLK, 1'b1, calcBT[2], calcBT_De[2]);
	KEY_Debounce Bt4(CLK, 1'b1, calcBT[3], calcBT_De[3]);
	
	always @(posedge CLK) begin 
		case (State)
			0 : begin
				bcd_out = bcd_in;
				CalcFinish = 0;
				if (calcBT_De != 4'b1111) begin 
					case (calcBT_De)
						4'b1110 : CalcState = 2'b00; // Add
						4'b1101 : CalcState = 2'b01; // Sub
						4'b1011 : CalcState = 2'b10; // Mul
						4'b0111 : CalcState = 2'b11; // Div
					endcase
					RegA = Dbus;
					State = 1;
				end
			end
			1 : begin 
				if (calcBT_De == 4'b1111) begin 
					State = 2;
				end
			end
			2 : begin 
				bcd_out = bcd_in;
				if (calcBT_De == 4'b0111) begin  
					RegB = Dbus;
					St = 1;
					State = 3;
				end
				else if (calcBT_De == 4'b1011) begin  // Reset
					State = 7;
				end
			end
			3 : begin 
				CalcFinish = Done & Rdy;
				St = 0;
				State = 4;
				if (!CalcFinish) begin 
					State = 3 ;
				end
			end 
			4 : begin 
				case (CalcState)
					2'b00 : calcResult = {Co_A, Out_A}; // Add
					2'b01 : calcResult = Out_S; // Sub
					2'b10 : calcResult = Product; // Mul
					2'b11 : calcResult = Quotient; // Div
				endcase
				State = 5;
			end 
			5 : begin 
				bcd_out = bcd_R;
				State = 6;
			end
			6 : begin 
				if (calcBT_De != 4'b1111) begin 
					case (calcBT_De)
						4'b1011 : begin // Reset
							State = 7;
						end
					endcase
				end
			end
			7 : begin 
				if (calcBT_De == 4'b1111) begin 
					State = 0;
				end
			end	
		endcase
	end
endmodule 

// 16bits Carry Look-Ahead Adder

module CLA16(A, B, Ci, S, Co, PG, GG);

	input 	[15:0] A;
	input 	[15:0] B;
	input				Ci;
	output 	[15:0] S;
	output			Co;
	output			PG; // group propagate
	output			GG; // group generate
	
	wire		[3:0]	G;
	wire		[3:0]	P;
	wire		[3:1]	C;
	
	CLALogic BlockCarryLogic(G, P, Ci, C, Co, PG, GG);
	CarryLookAheadAdder4 CLA0 (A[3:0], B[3:0], Ci, S[3:0], P[0], G[0]);
	CarryLookAheadAdder4 CLA1 (A[7:4], B[7:4], C[1], S[7:4], P[1], G[1]);
	CarryLookAheadAdder4 CLA2 (A[11:8], B[11:8], C[2], S[11:8], P[2], G[2]);
	CarryLookAheadAdder4 CLA3 (A[15:12], B[15:12], C[3], S[15:12], P[3], G[3]);
	
endmodule

module CarryLookAheadAdder4 (A, B, Ci, S, PG, GG);
	input 	[3:0] A;
	input 	[3:0] B;
	input				Ci;
	output 	[3:0] S;
	output			PG; // group propagate
	output			GG; // group generate
	
	wire		[3:0]	G;
	wire		[3:0]	P;
	wire		[3:1]	C;
	
	CLALogic CarryLogic(G, P, Ci, C, Co, PG, GG);
	GPFullAdder FA0(A[0], B[0], Ci, G[0], P[0], S[0]);
	GPFullAdder FA1(A[1], B[1], C[1], G[1], P[1], S[1]);
	GPFullAdder FA2(A[2], B[2], C[2], G[2], P[2], S[2]);
	GPFullAdder FA3(A[3], B[3], C[3], G[3], P[3], S[3]);
	
endmodule 

module CLALogic (G, P, Ci, C, Co, PG, GG);

	input		[3:0]	G;
	input		[3:0]	P;
	input				Ci;
	output	[3:1]	C;
	output			Co;
	output			PG;
	output			GG;
	
	wire GG_int, PG_int;
	
	assign C[1] = G[0] | (P[0] & Ci);
	assign C[2] = G[1] | (P[1] & G[0]) | (P[1] & P[0] & Ci);
	assign C[3] = G[2] | (P[2] & G[1]) | (P[2] & P[1] & G[0]) | (P[2] & P[1] & P[0] & Ci);
	assign Co = GG_int | (PG_int & Ci);
	assign PG_int = P[3] & P[2] & P[1] & P[0];
	assign GG_int = G[3] | (P[3] & G[2]) | (P[3] & P[2] & G[1]) | (P[3] & P[2] & P[1] & G[0]);
	assign PG = PG_int;
	assign GG = GG_int;
	
endmodule

module GPFullAdder (X, Y, Cin, G, P, Sum);

	input X, Y, Cin;
	output G, P, Sum;
	
	wire P_int;

	assign G = X & Y;
	assign P = P_int;
	assign P_int = X ^ Y;
	assign Sum = P_int ^ Cin;

endmodule  

module Mult16 (CLK, St, Mplier, Mcand, CalcFinish, Product, Done);
	input CLK;
	input St;
	input[15:0] Mplier;
	input[15:0] Mcand;
	input CalcFinish;
	output[31:0] Product;
	output Done;
	
	reg Done = 0;
	reg[5:0] State = 0;
	reg[5:0] Nextstate = 0;
	reg[15:0] A = 0;
	reg[15:0] B = 0;
	wire[15:0] compout;
	wire[15:0] addout;
	reg AdSh = 0;
	reg Sh = 0;
	reg Load = 0;
	reg Cm = 0;
	
	always @(State, St, B[0]) begin
		Load = 1'b0; AdSh = 1'b0; Sh = 1'b0; Cm = 1'b0; Done = 1'b0; Nextstate = 1'b0;
		case (State)
			0 : begin
				if (St == 1'b1) begin
					Load = 1'b1 ;
					Nextstate = 1 ;
				end
				else begin
					Load = 1'b0 ;
					Nextstate = 0 ;
				end
			end
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 : begin
				if (B[0] == 1'b1) begin
					AdSh = 1'b1 ;
				end
				else begin
					Sh = 1'b1 ;
				end
				Nextstate = State + 1 ;
			end
			16 : begin
				if (B[0] == 1'b1) begin
					Cm = 1'b1 ;
					AdSh = 1'b1 ;
				end
				else begin
					Sh = 1'b1 ;
				end
				Nextstate = 17 ;
			end
			17 : begin
				Done = 1'b1 ;
				Nextstate = 0 ;
				if (!CalcFinish) begin 
					Nextstate = 17 ;
				end
			end
			default : begin
				Done = 1'b0 ;
				Nextstate = 0 ;
			end
		endcase
	end
 
	assign compout = (Cm == 1'b1) ? ~Mcand : Mcand ;
	assign addout = A + compout + Cm ;
	
	always @(posedge CLK) begin
		if (Load == 1'b1) begin
			A <= 0 ;
			B <= Mplier ;
		end
		if (AdSh == 1'b1) begin
			A <= {compout[15], addout[15:1]} ;
			B <= {addout[0], B[15:1]} ;
		end
		if (Sh == 1'b1) begin
			A <= {A[15], A[15:1]} ;
			B <= {A[0], B[15:1]} ;
		end
		State <= Nextstate ;
	end
	
	assign Product = {A[14:0], B} ;
endmodule

module Div16 (CLK, St, Dividend_in, Divisor_in, Quotient, Remainder, V, CalcFinish, Rdy);
	input CLK;
	input St;
	input CalcFinish;
	input[15:0] Dividend_in;
	input[15:0] Divisor_in;
	output[15:0] Quotient;
	output[15:0] Remainder;
	output V; // Overflow
	output reg Rdy; // Ready Signal
	
	reg 				V = 0;	
	reg	[2:0] 	State;
	reg	[3:0] 	Count;
	reg 				Sign;
	wire 				C, Cm2;
	reg	[15:0] 	Divisor;
	wire	[15:0] 	Sum;
	wire	[15:0] 	Compout;
	reg	[31:0] 	Dividend;
	
	assign Cm2 = ~Divisor[15] ;
	assign Compout = (Cm2 == 1'b0) ? Divisor : ~Divisor ;
	assign Sum = Dividend[31:16] + Compout + Cm2;
	assign C = ~Sum[15] ;
	assign Quotient = Dividend[15:0] ;
	assign Remainder = Dividend[31:16];
	
	initial begin
		State = 0;
	end
	
	always @(posedge CLK) begin
		case (State)
			0 : begin
				Rdy = 0;
				if (St == 1'b1) begin
					Dividend[31:0] <= {16'b0, Dividend_in} ;
					Sign <= Dividend_in[15] ;
					V <= 1'b0 ;
					State <= 1 ;
					Count <= 4'b0000 ;
				end
			end
			1 : begin
				Divisor <= Divisor_in ;
				if (Sign == 1'b1) begin
					Dividend <= ~Dividend + 1 ;
				end
				State <= 2 ;
			end
			2 : begin
				Dividend <= {Dividend[30:0], 1'b0} ;
				Count <= Count + 1 ;
				State <= 3 ;
			end
			3 : begin
				if (C == 1'b1) begin
					V <= 1'b1 ;
					State <= 0 ;
				end
				else begin
					Dividend <= {Dividend[30:0], 1'b0} ;
					Count <= Count + 1 ;
					State <= 4 ;
				end
			end
			4 : begin
				if (C == 1'b1) begin
					Dividend[31:16] <= Sum ;
					Dividend[0] <= 1'b1 ;
				end
				else begin
					Dividend <= {Dividend[30:0], 1'b0} ;
					if (Count == 15) begin
						State <= 5 ;
					end
					Count <= Count + 1 ;
				end
			end
			5 : begin
				State <= 6 ;
				if (C == 1'b1) begin
					Dividend[31:16] <= Sum ;
					Dividend[0] <= 1'b1 ;
					State <= 5 ;
				end
				else if ((Sign ^ Divisor[15]) == 1'b1) begin
					Dividend[15:0] <= ~Dividend[15:0] + 1 ;
					if(Sign == 1) begin 
						Dividend [31:16] <= ~Dividend[31:16] + 1;
					end
					Rdy = 1;
				end
				else begin 
					if(Sign && Divisor[15]) begin 
						Dividend [31:16] <= ~Dividend[31:16] + 1;
					end
					Rdy = 1;
				end
			end
			6 : begin // Wait
				State <= 0 ;
				if (!CalcFinish) begin 
					State <= 6 ;
				end
			end
		endcase
	end
endmodule

module bin2bcd(bin, bcd);
    input [31:0] bin;
    output reg [39:0] bcd;
   
    integer i;
    
    always @(bin) begin
        bcd = 0;
        for (i = 0; i < 32; i = i + 1) begin
            if (bcd[3:0] >= 5) bcd[3:0] = bcd[3:0] + 3;
            if (bcd[7:4] >= 5) bcd[7:4] = bcd[7:4] + 3;
            if (bcd[11:8] >= 5) bcd[11:8] = bcd[11:8] + 3;
            if (bcd[15:12] >= 5) bcd[15:12] = bcd[15:12] + 3;
            if (bcd[19:16] >= 5) bcd[19:16] = bcd[19:16] + 3;
            if (bcd[23:20] >= 5) bcd[23:20] = bcd[23:20] + 3;
            if (bcd[27:24] >= 5) bcd[27:24] = bcd[27:24] + 3;
            if (bcd[31:28] >= 5) bcd[31:28] = bcd[31:28] + 3;
            if (bcd[35:32] >= 5) bcd[35:32] = bcd[35:32] + 3;
            if (bcd[39:36] >= 5) bcd[39:36] = bcd[39:36] + 3;
            
            // 移位並從輸入二進制數字中移入相應的位
            bcd = {bcd[38:0], bin[31 - i]};
        end
    end
endmodule


module BCD2segment7(bcd, seg);
     
	//Declare inputs,outputs and internal variables.
	input [3:0] bcd;
	output reg [6:0] seg;

	//always block for converting bcd digit into 7 segment format
	always @(bcd) begin
	case (bcd) //case statement
			0 : seg = 7'b0000001;
         1 : seg = 7'b1001111;
         2 : seg = 7'b0010010;
         3 : seg = 7'b0000110;
         4 : seg = 7'b1001100;
         5 : seg = 7'b0100100;
         6 : seg = 7'b0100000;
         7 : seg = 7'b0001111;
         8 : seg = 7'b0000000;
         9 : seg = 7'b0000100;
         //switch off 7 segment character when the bcd digit is not a decimal number.
         default : seg = 7'b1111111;
		endcase
    end  
endmodule

module KEY_Debounce(CLK, RST, KEY_In, KEY_Out);

    parameter    DeB_Num = 4;        // 取樣次數
    parameter    DeB_SET = 4'b0000;  // 設置
    parameter    DeB_RST = 4'b1111;  // 重置

    input   CLK, RST;
    input   KEY_In;
    output  KEY_Out;
    reg     KEY_Out = 1'b1;
    reg     [DeB_Num-1:0] Bounce = 4'b1111; // 初始化

    always @( posedge CLK, negedge RST ) begin  // 一次約200Hz 5ms
        if( !RST )
            Bounce <= DeB_RST;    // Bounce重置
        else begin    // 取樣4次
            integer    i;
            Bounce[0] <= KEY_In;
            for( i=0; i<DeB_Num-1; i=i+1 )
                Bounce[i+1] <= Bounce[i];
        end
        case( Bounce )
            DeB_SET:    KEY_Out <= 1'b0;
            default:    KEY_Out <= 1'b1;
        endcase
    end

endmodule

module div50M(clk, rst_n, CLK_8Hz);

	input clk, rst_n;
	output CLK_8Hz;
	reg [25:0] cnt_n;

	always@(negedge clk or negedge rst_n) begin
		if (!rst_n) begin
			cnt_n <= 0;
		end
		else if (cnt_n == 26'b10111110101111000010000000) begin// 0 ~ 50M
			cnt_n <= 0;
		end
		else begin
			cnt_n <= cnt_n + 1;
		end
	end

	assign CLK_8Hz = cnt_n[20];

endmodule
