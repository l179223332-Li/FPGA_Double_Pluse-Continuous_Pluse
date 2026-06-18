`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/07/07 11:10:10
// Design Name: 
// Module Name: tb_PWM_test
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module tb_PWM_test();

    // 测试信号定义
    reg sys_clk;
    reg rst_n;
    reg key1;
    reg key2;
    reg reset_key;
    reg st1_off;      
    reg st1_on;       
    reg st2_off;       
    reg st2_on;        
    wire PWM1;
    wire PWM2;
    
    // 实例化被测模块
    Double_PWM uut (
        .sys_clk(sys_clk),
        .rst_n(rst_n),
        .key1(key1),
        .key2(key2),
        .reset_key(reset_key),
        .st1_off(st1_off),
        .st1_on(st1_on),
        .st2_off(st2_off),
        .st2_on(st2_on),
        .PWM1(PWM1),
        .PWM2(PWM2)
    );
    
    // 时钟生成
    initial begin
        sys_clk = 0;
        forever #10 sys_clk = ~sys_clk; // 50MHz时钟，周期20ns
    end
    
    // 测试序列
    initial begin
        // 初始化
        rst_n = 0;
        key1 = 0;
        key2 = 0;
        reset_key = 0;
        st1_off = 0;
        st1_on = 0;
        st2_off = 0;
        st2_on = 0;
        // 复位释放
        #100;
        rst_n = 1;
        #100;
        
        // ===== 测试1：按键功能测试 =====
        $display("=== 测试1：按键功能测试 ===");
        
        // 1.1 key1按下，再按key2，PWM波形不受影响
        $display("1.1 key1按下，再按key2，PWM波形不受影响");
        key1 = 1;
        #20000; // 保持1ms
        key1 = 0;
        #30000; // 等待1ms
        
        key2 = 1;
        #10000; // 保持1ms
        key2 = 0;
        #10000; // 等待1ms
        
        // 复位
        reset_key = 1;
        #10000; // 保持1ms
        reset_key = 0;
        #10000; // 等待1ms
        
        // 1.2 按下key2，随后按下key1，波形也不受影响
        $display("1.2 按下key2，随后按下key1，波形也不受影响");
        key2 = 1;
        #5000; // 保持1ms
        key2 = 0;
        #5000; // 等待1ms
        key2 = 1;
        #5000; // 保持1ms
        key2 = 0;
        #5000; // 等待1ms
        #5000; // 保持1ms
        key2 = 0;
        #5000; // 等待1ms
        
        key1 = 1;
        #10000; // 保持1ms
        key1 = 0;
        #30000; // 等待1ms
        
        // 复位
        reset_key = 1;
        #10000; // 保持1ms
        reset_key = 0;
        #10000; // 等待1ms
        
        // ===== 测试2：连续脉冲测试 =====
        $display("=== 测试2：连续脉冲测试（10个周期） ===");
        
        // 启动连续PWM模式
        key1 = 1;
        #10000; // 保持1ms
        key1 = 0;
        #1200;
        st1_on = 1;
        #10;
        st2_on = 1;
        #5;
        st1_on = 0;
        #13;
        st2_on = 0;
        #2500;
        st1_off = 1;
        #10;
        st2_off = 1;
        #5;
        st1_off = 0;
        #13;
        st2_off = 0;
        
        #2500;
        st2_on = 1;
        #10;
        st1_on = 1;
        #5;
        st2_on = 0;
        #13;
        st1_on = 0;
        #2500;
        st2_off = 1;
        #10;
        st1_off = 1;
        #5;
        st2_off = 0;
        #13;
        st1_off = 0;
        
        // 等待10个PWM周期（5us × 10 = 50us）
        #500000; // 等待50us
        
        // 复位
        reset_key = 1;
        #10000; // 保持1ms
        reset_key = 0;
        #10000; // 等待1ms
        
        // ===== 测试3：双脉冲测试 =====
        $display("=== 测试3：双脉冲测试（80us总持续时间） ===");
        
        // 启动双脉冲模式
        key2 = 1;
        #10000; // 保持1ms
        key2 = 0;
        
        // 等待双脉冲完成（30us + 5us + 10us = 45us，加上一些余量到80us）
        #800000; // 等待80us
        
        // 复位
        reset_key = 1;
        #10000; // 保持1ms
        reset_key = 0;
        #10000; // 等待1ms
        
        // 结束仿真
        $display("=== 所有测试完成 ===");
        $finish;
    end
    
    // 监控PWM输出
    reg [31:0] pwm_high_count;
    reg [31:0] pwm_low_count;
    reg [31:0] pwm_cycle_count;
    reg pwm_prev;
    
    initial begin
        pwm_high_count = 0;
        pwm_low_count = 0;
        pwm_cycle_count = 0;
        pwm_prev = 0;
    end
    
    // PWM波形分析
    always @(posedge sys_clk) begin
        pwm_prev <= PWM1;
        
        if (PWM1) begin
            pwm_high_count <= pwm_high_count + 1;
        end else begin
            pwm_low_count <= pwm_low_count + 1;
        end
        
        // 检测PWM周期
        if (PWM1 && !pwm_prev) begin
            pwm_cycle_count <= pwm_cycle_count + 1;
            $display("PWM周期 %0d: 高电平=%0d个时钟周期, 低电平=%0d个时钟周期", 
                     pwm_cycle_count, pwm_high_count, pwm_low_count);
            pwm_high_count <= 0;
            pwm_low_count <= 0;
        end
    end
    
    // 生成波形文件
    initial begin
        $dumpfile("tb_PWM_test.vcd");
        $dumpvars(0, tb_PWM_test);
    end

endmodule
