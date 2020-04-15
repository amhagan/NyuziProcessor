//
// LCD Controller
//

module lcd_controller
    #(parameter BASE_ADDRESS = 0)

    (input                    clk,
     input                    reset,

    // Host
    input [8:0] char_code,
    input       char_strobe,
    output reg  done,

    // IO bus interface
    io_bus_interface.slave    io_bus,
    
    // LED
    output logic[17:0]        red_led,
    output logic[8:0]         green_led,
    
    // To LCD Controller
    output lcd_on,          // LCD Power ON/OFF
    output lcd_blon,        // LCD Back Light ON/OFF
    output lcd_rw,          // LCD Read/Write Select, 0 = Write, 1 = Read
    output lcd_en,          // LCD Enable
    output lcd_rs,          // LCD Command/Data Select, 0 = Command, 1 = Data
    inout  [7:0] lcd_data); // LCD Data bus 8 bits

    //    Internal Wires/Registers
    reg[5:0]   LUT_INDEX;
    reg[8:0]   LUT_DATA;
    reg[5:0]   mLCD_ST;
    reg[17:0]  mDLY;
    reg        mLCD_Start;
    reg[7:0]   mLCD_DATA;
    reg        mLCD_RS;
    wire       mLCD_Done;
    reg  internal_strobe, last_char_strobe ;

    parameter    LCD_INTIAL    =    0;
    parameter    LCD_LINE1     =    8;
    parameter    LCD_CH_LINE   =    LCD_LINE1+16;
    parameter    LCD_LINE2     =    LCD_LINE1+16+1;
    parameter    LUT_SIZE      =    LCD_LINE1+32+1;

    always_ff @(posedge clk, posedge reset)
    begin
        if (reset)
        begin
            red_led          <= 0;
            green_led        <= 0;
            LUT_INDEX        <= 0;
            mLCD_ST          <= 0;
            mDLY             <= 0;
            mLCD_Start       <= 0;
            mLCD_DATA        <= 0;
            mLCD_RS          <= 0;
            internal_strobe  <= 0; 
            last_char_strobe <= 0;
            done             <= 0;
        end
        else
        begin
            red_led   <= 7'b1101111;
            green_led <= 7'b1011111;

            if (char_strobe & ~last_char_strobe & ~internal_strobe & (LUT_INDEX==LUT_SIZE)) 
            begin
                    internal_strobe <= 1; 
                    done <= 0 ;
            end
            
            if (char_strobe) last_char_strobe <= 1;
            else  last_char_strobe <= 0 ;
            
            if(LUT_INDEX<LUT_SIZE)
            begin
                case(mLCD_ST)
                0:    begin
                        mLCD_DATA    <=    LUT_DATA[7:0];
                        mLCD_RS      <=    LUT_DATA[8];
                        mLCD_Start   <=    1;
                        mLCD_ST      <=    1;
                    end
                1:    begin
                        if(mLCD_Done)
                        begin
                            mLCD_Start    <=    0;
                            mLCD_ST       <=    2;                    
                        end
                    end
                2:    begin
                        if(mDLY<18'h3FFFE) // 5 mSec
                        mDLY    <=    mDLY + 1'b1;
                        else
                        begin
                            mDLY    <= 0;
                            mLCD_ST <= 3;
                        end
                    end
                3:    begin
                        LUT_INDEX <= LUT_INDEX + 1'b1;
                        mLCD_ST   <= 0;
                        if (LUT_INDEX == LUT_SIZE-1) done <= 1; // is this ending correct?
                    end
                endcase
            end //if(LUT_INDEX<LUT_SIZE)
            
            else if (internal_strobe )
            begin
                case(mLCD_ST)
                0:    begin
                        mLCD_DATA    <=    char_code;
                        mLCD_RS        <=    char_code[8];
                        mLCD_Start    <=    1;
                        mLCD_ST        <=    1;
                    end
                1:    begin
                        if(mLCD_Done)
                        begin
                            mLCD_Start    <=    0;
                            mLCD_ST        <=    2;                    
                        end
                    end
                2:    begin
                        if(mDLY<18'hFFFE) // 5msec==3FFFE
                        mDLY    <=    mDLY + 1'b1;
                        else
                        begin
                            mDLY    <=    0;
                            mLCD_ST    <=    3;
                        end
                    end
                3:    begin
                        //LUT_INDEX    <=    LUT_INDEX + 1'b1;
                        mLCD_ST    <=    0;
                            internal_strobe <= 0;
                            done <= 1;
                    end
                endcase		  
            end // if (internal_strobe)
        end
    end

    always @(LUT_INDEX)
    begin
        case(LUT_INDEX)
        //    Initial
        LCD_INTIAL+0:    LUT_DATA    <=    9'h038; // need to send initial cmd several times to synch
        LCD_INTIAL+1:    LUT_DATA    <=    9'h038;
        LCD_INTIAL+2:    LUT_DATA    <=    9'h038;
        LCD_INTIAL+3:    LUT_DATA    <=    9'h038;
        LCD_INTIAL+4:    LUT_DATA    <=    9'h00C;
        LCD_INTIAL+5:    LUT_DATA    <=    9'h001;
        LCD_INTIAL+6:    LUT_DATA    <=    9'h006;
        LCD_INTIAL+7:    LUT_DATA    <=    9'h080;
        //    Line 1
        LCD_LINE1+0:    LUT_DATA    <=    9'h141;    //    <Altera DE2 Kit>
        LCD_LINE1+1:    LUT_DATA    <=    9'h16C;
        LCD_LINE1+2:    LUT_DATA    <=    9'h174;
        LCD_LINE1+3:    LUT_DATA    <=    9'h165;
        LCD_LINE1+4:    LUT_DATA    <=    9'h172;
        LCD_LINE1+5:    LUT_DATA    <=    9'h161;
        LCD_LINE1+6:    LUT_DATA    <=    9'h120;
        LCD_LINE1+7:    LUT_DATA    <=    9'h144;
        LCD_LINE1+8:    LUT_DATA    <=    9'h145;
        LCD_LINE1+9:    LUT_DATA    <=    9'h132;
        LCD_LINE1+10:    LUT_DATA    <=    9'h120;
        LCD_LINE1+11:    LUT_DATA    <=    9'h142;
        LCD_LINE1+12:    LUT_DATA    <=    9'h16F;
        LCD_LINE1+13:    LUT_DATA    <=    9'h161;
        LCD_LINE1+14:    LUT_DATA    <=    9'h172;
        LCD_LINE1+15:    LUT_DATA    <=    9'h164;
        //    Change Line
        LCD_CH_LINE:    LUT_DATA    <=    9'h0C0;
        //    Line 2
        LCD_LINE1+0:    LUT_DATA    <=    9'h141;    //    <Altera DE2 Kit>
        LCD_LINE1+1:    LUT_DATA    <=    9'h16C;
        LCD_LINE1+2:    LUT_DATA    <=    9'h174;
        LCD_LINE1+3:    LUT_DATA    <=    9'h165;
        LCD_LINE1+4:    LUT_DATA    <=    9'h172;
        LCD_LINE1+5:    LUT_DATA    <=    9'h161;
        LCD_LINE1+6:    LUT_DATA    <=    9'h120;
        LCD_LINE1+7:    LUT_DATA    <=    9'h144;
        LCD_LINE1+8:    LUT_DATA    <=    9'h145;
        LCD_LINE1+9:    LUT_DATA    <=    9'h132;
        LCD_LINE1+10:    LUT_DATA    <=    9'h120;
        LCD_LINE1+11:    LUT_DATA    <=    9'h142;
        LCD_LINE1+12:    LUT_DATA    <=    9'h16F;
        LCD_LINE1+13:    LUT_DATA    <=    9'h161;
        LCD_LINE1+14:    LUT_DATA    <=    9'h172;
        LCD_LINE1+15:    LUT_DATA    <=    9'h164;
        default:        LUT_DATA    <=    9'h121 ;
        endcase
    end

    LCD_Controller u0(
    //    Host Side
    .iDATA(mLCD_DATA),
    .iRS(mLCD_RS),
    .iStart(mLCD_Start),
    .oDone(mLCD_Done),
    .iCLK(clk),
    .iRST_N(reset),
    //    LCD Interface
    .LCD_DATA(lcd_data),
    .LCD_RW(lcd_rw),
    .LCD_EN(lcd_en),
    .LCD_RS(lcd_rs)    );
endmodule
 
/////////////////////////////////////////////////////
/// LCD controller //////////////////////////////////
//////////////////////////////////////////////////////
module LCD_Controller (    
    //    Host Side
    input [7:0] iDATA,
    input iRS,
    input iStart,
    output reg oDone,
    input iCLK,iRST_N,

    //    LCD Interface
    output [7:0] LCD_DATA,
    output LCD_RW,
    output reg LCD_EN,
    output LCD_RS    );

    parameter    CLK_Divide    =    16;

    //    Internal Register
    reg        [4:0]    Cont;
    reg        [1:0]    ST;
    reg        preStart,mStart;

    /////////////////////////////////////////////
    //    Only write to LCD, bypass iRS to LCD_RS
    assign    LCD_DATA    =    iDATA; 
    assign    LCD_RW        =    1'b0;
    assign    LCD_RS        =    iRS;
    /////////////////////////////////////////////

    always@(posedge iCLK or negedge iRST_N)
    begin
        if(!iRST_N)
        begin
            oDone    <=    1'b0;
            LCD_EN    <=    1'b0;
            preStart<=    1'b0;
            mStart    <=    1'b0;
            Cont    <=    0;
            ST        <=    0;
        end
        else
        begin
            //////    Input Start Detect ///////
            preStart<=    iStart;
            if({preStart,iStart}==2'b01)
            begin
                mStart    <=    1'b1;
                oDone    <=    1'b0;
            end
            //////////////////////////////////
            if(mStart)
            begin
                case(ST)
                0:    ST    <=    1;    //    Wait Setup
                1:    begin
                        LCD_EN    <=    1'b1;
                        ST        <=    2;
                    end
                2:    begin                    
                        if(Cont<CLK_Divide)
                        Cont    <=    Cont + 1'b1;
                        else
                        ST        <=    3;
                    end
                3:    begin
                        LCD_EN    <=    1'b0;
                        mStart    <=    1'b0;
                        oDone    <=    1'b1;
                        Cont    <=    0;
                        ST        <=    0;
                    end
                endcase
            end
        end
    end

endmodule