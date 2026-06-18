`timescale 1ns / 1ps

module Double_PWM (
    input wire sys_clk,       // 50MHz时钟
    input wire rst_n,         // 复位信号，低电平有效
    input wire key1,          // 连续PWM控制按键（高电平有效）
    input wire key2,          // 双脉冲控制按键（高电平有效）
    input wire reset_key,     // 复位按键（高电平有效）
    input wire st1_off,       // 上管关断脉冲
    input wire st1_on,        // 上管开通脉冲
    input wire st2_off,       // 下管关断脉冲
    input wire st2_on,        // 下管开通脉冲
    output reg PWM1,          // PWM1输出
    output reg PWM2,           // PWM2输出
    output wire led_out1,
    output wire led_out2,
    output wire led_out3,
    output wire led_out4
);

    // 250MHz时钟倍频
    wire clk;
    wire locked;
    clk_wiz_0 clk_inst(
        .clk_in1(sys_clk),
        .clk_out1(clk),
        .reset(!rst_n),
        .locked(locked)
    );

    // 添加LED闪烁测试
    reg [25:0] test_counter;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) test_counter <= 0;
        else test_counter <= test_counter + 1;
    end

    assign led_out1 = test_counter[25]; // 0.5Hz闪烁
    assign led_out2 = ~locked;

    // 原始参数设定
    parameter CLK_FREQ = 200;                    // 200MHz
    parameter PWM_PERIOD = 50;                   // 50us周期
    parameter PWM_DUTY = 50;                     // 50%占空比
    parameter HIGH_PULSE1 = 30;                  // 第一个高脉冲30us
    parameter LOW_PULSE = 15;                    // 低脉冲15us
    parameter HIGH_PULSE2 = 10;                  // 第二个高脉冲10us
    parameter DEBOUNCE_TIME = 20;               // 消抖时间20ms
    parameter ENABLE_DEBOUNCE = 1;               // 消抖使能：1=启用，0=停用
    parameter CONTINUOUS_MAX_CYCLES = 80;        // 连续PWM最大输出周期数
    parameter ENABLE_ADJUST = 0;                 // PWM2微调使能

    // 时间-周期数精确映射
    parameter CLK_CYCLES_PER_US = CLK_FREQ;
    parameter PWM_PERIOD_CYCLES = PWM_PERIOD * CLK_CYCLES_PER_US;
    parameter PWM_HIGH_CYCLES = (PWM_PERIOD_CYCLES * PWM_DUTY) / 100;
    parameter HIGH_PULSE1_CYCLES = HIGH_PULSE1 * CLK_CYCLES_PER_US;
    parameter LOW_PULSE_CYCLES = LOW_PULSE * CLK_CYCLES_PER_US;
    parameter HIGH_PULSE2_CYCLES = HIGH_PULSE2 * CLK_CYCLES_PER_US;
    parameter DEBOUNCE_CYCLES = DEBOUNCE_TIME * 1000 * CLK_CYCLES_PER_US;

    // 状态机定义
    localparam IDLE = 3'b000;
    localparam CONTINUOUS_PWM = 3'b001;
    localparam DUAL_PULSE_HIGH1 = 3'b010;
    localparam DUAL_PULSE_LOW = 3'b011;
    localparam DUAL_PULSE_HIGH2 = 3'b100;
    localparam WAIT_LOCK = 3'b111;

    // 寄存器定义
    reg [2:0] current_state, next_state;
    reg [31:0] pwm_counter;
    reg [31:0] dual_pulse_counter;
    reg [31:0] continuous_cycle_counter;

    assign led_out3 = ~(current_state == IDLE);
    assign led_out4 = ~(current_state != IDLE);

    // ST采样相关寄存器
    reg [31:0] st1_off_rise, st1_off_high;
    reg [31:0] st1_on_rise, st1_on_high;
    reg [31:0] st2_off_rise, st2_off_high;
    reg [31:0] st2_on_rise, st2_on_high;
    reg [1:0] st1_off_sync, st1_on_sync, st2_off_sync, st2_on_sync;
    reg st1_off_prev, st1_on_prev, st2_off_prev, st2_on_prev;
    reg signed [31:0] delta_t1, delta_t2, delta_t3, delta_t4;

    // PWM微调相关寄存器
    reg signed [31:0] pwm2_rise, pwm2_fall;

    // 按键消抖相关寄存器
    reg [31:0] key1_debounce_counter;
    reg [31:0] key2_debounce_counter;
    reg [31:0] reset_debounce_counter;
    reg key1_debounced;
    reg key2_debounced;
    reg reset_debounced;
    reg key1_prev, key2_prev, reset_prev;
    wire key1_rising, key2_rising, reset_rising;

    // 按键消抖分频优化
    reg [7:0] debounce_clk_div;
    wire debounce_clk_en = (debounce_clk_div == 8'd199);
    localparam DEBOUNCE_CYCLES_DIV = DEBOUNCE_TIME * 1000;

    // ST采样状态机定义
    localparam SAMPLE_IDLE = 3'b000;
    localparam SAMPLE_RISING_DETECTED = 3'b001;
    localparam SAMPLE_WAIT_FALL = 3'b010;
    localparam SAMPLE_COMPLETE = 3'b011;

    reg [2:0] st1_off_sample_state, st1_on_sample_state;
    reg [2:0] st2_off_sample_state, st2_on_sample_state;

    // Delta计算流水线寄存器
    reg signed [31:0] st1_st2_off_rise_diff;
    reg signed [31:0] st1_st2_off_high_diff;
    reg signed [31:0] st1_st2_on_rise_diff;
    reg signed [31:0] st1_st2_on_high_diff;
    reg calc_enable_d1;
    reg [31:0] continuous_cycle_counter_d1;
    reg signed [31:0] delta_t1_d1, delta_t2_d1, delta_t3_d1, delta_t4_d1;

    // 将按键上升沿检测做成组合信号
    assign key1_rising = key1_debounced & ~key1_prev;
    assign key2_rising = key2_debounced & ~key2_prev;
    assign reset_rising = reset_debounced & ~reset_prev;

    // -------------------------
    // 按键消抖逻辑 - 分频优化版
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debounce_clk_div <= 8'd0;
            key1_debounce_counter <= 32'd0;
            key2_debounce_counter <= 32'd0;
            reset_debounce_counter <= 32'd0;
            key1_debounced <= 1'b0;
            key2_debounced <= 1'b0;
            reset_debounced <= 1'b0;
        end else if (!locked) begin
            debounce_clk_div <= 8'd0;
            key1_debounce_counter <= 32'd0;
            key2_debounce_counter <= 32'd0;
            reset_debounce_counter <= 32'd0;
            key1_debounced <= 1'b0;
            key2_debounced <= 1'b0;
            reset_debounced <= 1'b0;
        end else begin
            debounce_clk_div <= debounce_clk_div + 1'b1;
            
            if (debounce_clk_en && ENABLE_DEBOUNCE) begin
                // key1消抖
                if (key1 != key1_debounced) begin
                    if (key1_debounce_counter == DEBOUNCE_CYCLES_DIV) begin
                        key1_debounced <= key1;
                        key1_debounce_counter <= 32'd0;
                    end else begin
                        key1_debounce_counter <= key1_debounce_counter + 1'b1;
                    end
                end else begin
                    key1_debounce_counter <= 32'd0;
                end

                // key2消抖
                if (key2 != key2_debounced) begin
                    if (key2_debounce_counter == DEBOUNCE_CYCLES_DIV) begin
                        key2_debounced <= key2;
                        key2_debounce_counter <= 32'd0;
                    end else begin
                        key2_debounce_counter <= key2_debounce_counter + 1'b1;
                    end
                end else begin
                    key2_debounce_counter <= 32'd0;
                end

                // 复位按键消抖
                if (reset_key != reset_debounced) begin
                    if (reset_debounce_counter == DEBOUNCE_CYCLES_DIV) begin
                        reset_debounced <= reset_key;
                        reset_debounce_counter <= 32'd0;
                    end else begin
                        reset_debounce_counter <= reset_debounce_counter + 1'b1;
                    end
                end else begin
                    reset_debounce_counter <= 32'd0;
                end
            end else if (!ENABLE_DEBOUNCE) begin
                key1_debounced <= key1;
                key2_debounced <= key2;
                reset_debounced <= reset_key;
            end
        end
    end

    // prev更新逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key1_prev <= 1'b0;
            key2_prev <= 1'b0;
            reset_prev <= 1'b0;
        end else if (!locked) begin
            key1_prev <= 1'b0;
            key2_prev <= 1'b0;
            reset_prev <= 1'b0;
        end else begin
            key1_prev <= key1_debounced;
            key2_prev <= key2_debounced;
            reset_prev <= reset_debounced;
        end
    end

// -------------------------
// ST信号同步和边沿检测 - 时序优化版
// -------------------------

// 增加流水线寄存器来降低组合逻辑深度
reg st1_off_sync1, st1_on_sync1, st2_off_sync1, st2_on_sync1;
reg st1_off_prev_d, st1_on_prev_d, st2_off_prev_d, st2_on_prev_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st1_off_sync <= 2'b00; st1_on_sync <= 2'b00;
        st2_off_sync <= 2'b00; st2_on_sync <= 2'b00;
        st1_off_sync1 <= 1'b0; st1_on_sync1 <= 1'b0;
        st2_off_sync1 <= 1'b0; st2_on_sync1 <= 1'b0;
        st1_off_prev <= 1'b0; st1_on_prev <= 1'b0;
        st2_off_prev <= 1'b0; st2_on_prev <= 1'b0;
        st1_off_prev_d <= 1'b0; st1_on_prev_d <= 1'b0;
        st2_off_prev_d <= 1'b0; st2_on_prev_d <= 1'b0;
    end else if (!locked) begin
        st1_off_sync <= 2'b00; st1_on_sync <= 2'b00;
        st2_off_sync <= 2'b00; st2_on_sync <= 2'b00;
        st1_off_sync1 <= 1'b0; st1_on_sync1 <= 1'b0;
        st2_off_sync1 <= 1'b0; st2_on_sync1 <= 1'b0;
        st1_off_prev <= 1'b0; st1_on_prev <= 1'b0;
        st2_off_prev <= 1'b0; st2_on_prev <= 1'b0;
        st1_off_prev_d <= 1'b0; st1_on_prev_d <= 1'b0;
        st2_off_prev_d <= 1'b0; st2_on_prev_d <= 1'b0;
    end else if (ENABLE_ADJUST) begin
        // 三级同步器减少亚稳态
        st1_off_sync <= {st1_off_sync[0], st1_off};
        st1_on_sync <= {st1_on_sync[0], st1_on};
        st2_off_sync <= {st2_off_sync[0], st2_off};
        st2_on_sync <= {st2_on_sync[0], st2_on};
        
        st1_off_sync1 <= st1_off_sync[1];
        st1_on_sync1 <= st1_on_sync[1];
        st2_off_sync1 <= st2_off_sync[1];
        st2_on_sync1 <= st2_on_sync[1];
        
        // 增加流水线延迟，减少组合逻辑路径
        st1_off_prev_d <= st1_off_sync1;
        st1_on_prev_d <= st1_on_sync1;
        st2_off_prev_d <= st2_off_sync1;
        st2_on_prev_d <= st2_on_sync1;
        
        st1_off_prev <= st1_off_prev_d;
        st1_on_prev <= st1_on_prev_d;
        st2_off_prev <= st2_off_prev_d;
        st2_on_prev <= st2_on_prev_d;
    end
end

// 边沿检测信号 - 寄存器输出优化
reg st1_off_rising_r, st1_off_falling_r;
reg st1_on_rising_r, st1_on_falling_r;
reg st2_off_rising_r, st2_off_falling_r;
reg st2_on_rising_r, st2_on_falling_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st1_off_rising_r <= 1'b0; st1_off_falling_r <= 1'b0;
        st1_on_rising_r <= 1'b0; st1_on_falling_r <= 1'b0;
        st2_off_rising_r <= 1'b0; st2_off_falling_r <= 1'b0;
        st2_on_rising_r <= 1'b0; st2_on_falling_r <= 1'b0;
    end else if (!locked) begin
        st1_off_rising_r <= 1'b0; st1_off_falling_r <= 1'b0;
        st1_on_rising_r <= 1'b0; st1_on_falling_r <= 1'b0;
        st2_off_rising_r <= 1'b0; st2_off_falling_r <= 1'b0;
        st2_on_rising_r <= 1'b0; st2_on_falling_r <= 1'b0;
    end else if (ENABLE_ADJUST) begin
        // 使用寄存器的边沿检测，避免长组合逻辑路径
        st1_off_rising_r <= st1_off_sync1 && !st1_off_prev_d;
        st1_off_falling_r <= !st1_off_sync1 && st1_off_prev_d;
        st1_on_rising_r <= st1_on_sync1 && !st1_on_prev_d;
        st1_on_falling_r <= !st1_on_sync1 && st1_on_prev_d;
        st2_off_rising_r <= st2_off_sync1 && !st2_off_prev_d;
        st2_off_falling_r <= !st2_off_sync1 && st2_off_prev_d;
        st2_on_rising_r <= st2_on_sync1 && !st2_on_prev_d;
        st2_on_falling_r <= !st2_on_sync1 && st2_on_prev_d;
    end
end

// 采样复位条件 - 流水线优化
reg sample_reset_condition_r;
reg [31:0] pwm_counter_d1, pwm_counter_d2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pwm_counter_d1 <= 32'd0;
        pwm_counter_d2 <= 32'd0;
        sample_reset_condition_r <= 1'b0;
    end else if (!locked) begin
        pwm_counter_d1 <= 32'd0;
        pwm_counter_d2 <= 32'd0;
        sample_reset_condition_r <= 1'b0;
    end else begin
        // 对pwm_counter进行流水线处理
        pwm_counter_d1 <= pwm_counter;
        pwm_counter_d2 <= pwm_counter_d1;
        
        // 预先计算比较条件，减少组合逻辑深度
        sample_reset_condition_r <= 
            (pwm_counter_d1 == PWM_PERIOD_CYCLES - 200) || 
            (pwm_counter_d1 == PWM_PERIOD_CYCLES / 4) || 
            (pwm_counter_d1 == PWM_PERIOD_CYCLES * 3 / 4) ||
            (current_state != CONTINUOUS_PWM) ||
            reset_rising ||
            !ENABLE_ADJUST;
    end
end

// 全局采样使能 - 寄存器输出
reg sample_enable_r;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) sample_enable_r <= 1'b0;
    else if (!locked) sample_enable_r <= 1'b0;
    else sample_enable_r <= ENABLE_ADJUST && (current_state == CONTINUOUS_PWM) && locked;
end

// -------------------------
// ST采样状态机 - 时序优化版
// -------------------------

// 增加中间寄存器来存储计算值
reg [31:0] pwm_counter_minus_rise_st1_off, pwm_counter_minus_rise_st1_on;
reg [31:0] pwm_counter_minus_rise_st2_off, pwm_counter_minus_rise_st2_on;

// ST1关断脉冲采样状态机 - 优化版
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st1_off_sample_state <= SAMPLE_IDLE;
        st1_off_rise <= 0;
        st1_off_high <= 0;
        pwm_counter_minus_rise_st1_off <= 0;
    end else if (!locked) begin
        st1_off_sample_state <= SAMPLE_IDLE;
        st1_off_rise <= 0;
        st1_off_high <= 0;
        pwm_counter_minus_rise_st1_off <= 0;
    end else if (sample_reset_condition_r) begin
        st1_off_sample_state <= SAMPLE_IDLE;
        if (pwm_counter_d1 == PWM_PERIOD_CYCLES - 200) begin
            st1_off_rise <= 0;
            st1_off_high <= 0;
        end
    end else if (sample_enable_r) begin
        // 预先计算差值，减少关键路径
        pwm_counter_minus_rise_st1_off <= pwm_counter_d1 - st1_off_rise;
        
        case (st1_off_sample_state)
            SAMPLE_IDLE: begin
                if (st1_off_rising_r) begin
                    st1_off_rise <= pwm_counter_d1;
                    st1_off_sample_state <= SAMPLE_RISING_DETECTED;
                end
            end
            
            SAMPLE_RISING_DETECTED: begin
                st1_off_sample_state <= SAMPLE_WAIT_FALL;
            end
            
            SAMPLE_WAIT_FALL: begin
                if (st1_off_falling_r) begin
                    st1_off_high <= pwm_counter_minus_rise_st1_off;
                    st1_off_sample_state <= SAMPLE_COMPLETE;
                end
                else if (pwm_counter_minus_rise_st1_off > PWM_PERIOD_CYCLES / 2) begin
                    st1_off_sample_state <= SAMPLE_IDLE;
                end
            end
            
            SAMPLE_COMPLETE: begin
                // 保持完成状态
            end
            
            default: begin
                st1_off_sample_state <= SAMPLE_IDLE;
            end
        endcase
    end
end

// ST1开通脉冲采样状态机 - 优化版
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st1_on_sample_state <= SAMPLE_IDLE;
        st1_on_rise <= 0;
        st1_on_high <= 0;
        pwm_counter_minus_rise_st1_on <= 0;
    end else if (!locked) begin
        st1_on_sample_state <= SAMPLE_IDLE;
        st1_on_rise <= 0;
        st1_on_high <= 0;
        pwm_counter_minus_rise_st1_on <= 0;
    end else if (sample_reset_condition_r) begin
        st1_on_sample_state <= SAMPLE_IDLE;
        if (pwm_counter_d1 == PWM_PERIOD_CYCLES - 200) begin
            st1_on_rise <= 0;
            st1_on_high <= 0;
        end
    end else if (sample_enable_r) begin
        pwm_counter_minus_rise_st1_on <= pwm_counter_d1 - st1_on_rise;
        
        case (st1_on_sample_state)
            SAMPLE_IDLE: begin
                if (st1_on_rising_r) begin
                    st1_on_rise <= pwm_counter_d1;
                    st1_on_sample_state <= SAMPLE_RISING_DETECTED;
                end
            end
            
            SAMPLE_RISING_DETECTED: begin
                st1_on_sample_state <= SAMPLE_WAIT_FALL;
            end
            
            SAMPLE_WAIT_FALL: begin
                if (st1_on_falling_r) begin
                    st1_on_high <= pwm_counter_minus_rise_st1_on;
                    st1_on_sample_state <= SAMPLE_COMPLETE;
                end
                else if (pwm_counter_minus_rise_st1_on > PWM_PERIOD_CYCLES / 2) begin
                    st1_on_sample_state <= SAMPLE_IDLE;
                end
            end
            
            SAMPLE_COMPLETE: begin
                // 保持完成状态
            end
            
            default: begin
                st1_on_sample_state <= SAMPLE_IDLE;
            end
        endcase
    end
end

// ST2关断脉冲采样状态机 - 优化版
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st2_off_sample_state <= SAMPLE_IDLE;
        st2_off_rise <= 0;
        st2_off_high <= 0;
        pwm_counter_minus_rise_st2_off <= 0;
    end else if (!locked) begin
        st2_off_sample_state <= SAMPLE_IDLE;
        st2_off_rise <= 0;
        st2_off_high <= 0;
        pwm_counter_minus_rise_st2_off <= 0;
    end else if (sample_reset_condition_r) begin
        st2_off_sample_state <= SAMPLE_IDLE;
        if (pwm_counter_d1 == PWM_PERIOD_CYCLES - 200) begin
            st2_off_rise <= 0;
            st2_off_high <= 0;
        end
    end else if (sample_enable_r) begin
        pwm_counter_minus_rise_st2_off <= pwm_counter_d1 - st2_off_rise;
        
        case (st2_off_sample_state)
            SAMPLE_IDLE: begin
                if (st2_off_rising_r) begin
                    st2_off_rise <= pwm_counter_d1;
                    st2_off_sample_state <= SAMPLE_RISING_DETECTED;
                end
            end
            
            SAMPLE_RISING_DETECTED: begin
                st2_off_sample_state <= SAMPLE_WAIT_FALL;
            end
            
            SAMPLE_WAIT_FALL: begin
                if (st2_off_falling_r) begin
                    st2_off_high <= pwm_counter_minus_rise_st2_off;
                    st2_off_sample_state <= SAMPLE_COMPLETE;
                end
                else if (pwm_counter_minus_rise_st2_off > PWM_PERIOD_CYCLES / 2) begin
                    st2_off_sample_state <= SAMPLE_IDLE;
                end
            end
            
            SAMPLE_COMPLETE: begin
                // 保持完成状态
            end
            
            default: begin
                st2_off_sample_state <= SAMPLE_IDLE;
            end
        endcase
    end
end

// ST2开通脉冲采样状态机 - 优化版
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st2_on_sample_state <= SAMPLE_IDLE;
        st2_on_rise <= 0;
        st2_on_high <= 0;
        pwm_counter_minus_rise_st2_on <= 0;
    end else if (!locked) begin
        st2_on_sample_state <= SAMPLE_IDLE;
        st2_on_rise <= 0;
        st2_on_high <= 0;
        pwm_counter_minus_rise_st2_on <= 0;
    end else if (sample_reset_condition_r) begin
        st2_on_sample_state <= SAMPLE_IDLE;
        if (pwm_counter_d1 == PWM_PERIOD_CYCLES - 200) begin
            st2_on_rise <= 0;
            st2_on_high <= 0;
        end
    end else if (sample_enable_r) begin
        pwm_counter_minus_rise_st2_on <= pwm_counter_d1 - st2_on_rise;
        
        case (st2_on_sample_state)
            SAMPLE_IDLE: begin
                if (st2_on_rising_r) begin
                    st2_on_rise <= pwm_counter_d1;
                    st2_on_sample_state <= SAMPLE_RISING_DETECTED;
                end
            end
            
            SAMPLE_RISING_DETECTED: begin
                st2_on_sample_state <= SAMPLE_WAIT_FALL;
            end
            
            SAMPLE_WAIT_FALL: begin
                if (st2_on_falling_r) begin
                    st2_on_high <= pwm_counter_minus_rise_st2_on;
                    st2_on_sample_state <= SAMPLE_COMPLETE;
                end
                else if (pwm_counter_minus_rise_st2_on > PWM_PERIOD_CYCLES / 2) begin
                    st2_on_sample_state <= SAMPLE_IDLE;
                end
            end
            
            SAMPLE_COMPLETE: begin
                // 保持完成状态
            end
            
            default: begin
                st2_on_sample_state <= SAMPLE_IDLE;
            end
        endcase
    end
end

// -------------------------
// Delta计算逻辑（深度流水线优化版）
// -------------------------

// 增加更多流水线阶段
reg signed [31:0] st1_off_rise_d1, st1_off_high_d1, st1_on_rise_d1, st1_on_high_d1;
reg signed [31:0] st2_off_rise_d1, st2_off_high_d1, st2_on_rise_d1, st2_on_high_d1;

// 第一阶段：输入寄存器
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st1_off_rise_d1 <= 0; st1_off_high_d1 <= 0;
        st1_on_rise_d1 <= 0; st1_on_high_d1 <= 0;
        st2_off_rise_d1 <= 0; st2_off_high_d1 <= 0;
        st2_on_rise_d1 <= 0; st2_on_high_d1 <= 0;
        calc_enable_d1 <= 1'b0;
        continuous_cycle_counter_d1 <= 32'd0;
    end else if (!locked) begin
        st1_off_rise_d1 <= 0; st1_off_high_d1 <= 0;
        st1_on_rise_d1 <= 0; st1_on_high_d1 <= 0;
        st2_off_rise_d1 <= 0; st2_off_high_d1 <= 0;
        st2_on_rise_d1 <= 0; st2_on_high_d1 <= 0;
        calc_enable_d1 <= 1'b0;
        continuous_cycle_counter_d1 <= 32'd0;
    end else begin
        // 输入数据打拍
        st1_off_rise_d1 <= st1_off_rise; st1_off_high_d1 <= st1_off_high;
        st1_on_rise_d1 <= st1_on_rise; st1_on_high_d1 <= st1_on_high;
        st2_off_rise_d1 <= st2_off_rise; st2_off_high_d1 <= st2_off_high;
        st2_on_rise_d1 <= st2_on_rise; st2_on_high_d1 <= st2_on_high;
        
        calc_enable_d1 <= ENABLE_ADJUST && (current_state == CONTINUOUS_PWM);
        continuous_cycle_counter_d1 <= continuous_cycle_counter;
    end
end

// 第二阶段：减法计算
reg signed [31:0] st1_st2_off_rise_diff_d, st1_st2_off_high_diff_d;
reg signed [31:0] st1_st2_on_rise_diff_d, st1_st2_on_high_diff_d;
reg calc_enable_d2, calc_enable_d3;
reg [31:0] continuous_cycle_counter_d2, continuous_cycle_counter_d3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st1_st2_off_rise_diff_d <= 0; st1_st2_off_high_diff_d <= 0;
        st1_st2_on_rise_diff_d <= 0; st1_st2_on_high_diff_d <= 0;
        calc_enable_d2 <= 1'b0;
        continuous_cycle_counter_d2 <= 32'd0;
    end else if (!locked) begin
        st1_st2_off_rise_diff_d <= 0; st1_st2_off_high_diff_d <= 0;
        st1_st2_on_rise_diff_d <= 0; st1_st2_on_high_diff_d <= 0;
        calc_enable_d2 <= 1'b0;
        continuous_cycle_counter_d2 <= 32'd0;
    end else begin
        if (calc_enable_d1) begin
            st1_st2_off_rise_diff_d <= $signed(st1_off_rise_d1) - $signed(st2_off_rise_d1);
            st1_st2_off_high_diff_d <= $signed(st1_off_high_d1) - $signed(st2_off_high_d1);
            st1_st2_on_rise_diff_d <= $signed(st1_on_rise_d1) - $signed(st2_on_rise_d1);
            st1_st2_on_high_diff_d <= $signed(st1_on_high_d1) - $signed(st2_on_high_d1);
        end else begin
            st1_st2_off_rise_diff_d <= 0; st1_st2_off_high_diff_d <= 0;
            st1_st2_on_rise_diff_d <= 0; st1_st2_on_high_diff_d <= 0;
        end
        
        calc_enable_d2 <= calc_enable_d1;
        continuous_cycle_counter_d2 <= continuous_cycle_counter_d1;
    end
end

// 第三阶段：累加运算
reg signed [31:0] delta_sum_t1, delta_sum_t2, delta_sum_t3, delta_sum_t4;
reg calc_enable_d4;
reg [31:0] continuous_cycle_counter_d4;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        delta_sum_t1 <= 0; delta_sum_t2 <= 0; delta_sum_t3 <= 0; delta_sum_t4 <= 0;
        calc_enable_d3 <= 1'b0; calc_enable_d4 <= 1'b0;
        continuous_cycle_counter_d3 <= 32'd0; continuous_cycle_counter_d4 <= 32'd0;
    end else if (!locked) begin
        delta_sum_t1 <= 0; delta_sum_t2 <= 0; delta_sum_t3 <= 0; delta_sum_t4 <= 0;
        calc_enable_d3 <= 1'b0; calc_enable_d4 <= 1'b0;
        continuous_cycle_counter_d3 <= 32'd0; continuous_cycle_counter_d4 <= 32'd0;
    end else begin
        calc_enable_d3 <= calc_enable_d2;
        calc_enable_d4 <= calc_enable_d3;
        continuous_cycle_counter_d3 <= continuous_cycle_counter_d2;
        continuous_cycle_counter_d4 <= continuous_cycle_counter_d3;
        
        if (calc_enable_d2) begin
            if (continuous_cycle_counter_d2 == 0) begin
                // 第一个周期：直接赋值
                delta_sum_t1 <= st1_st2_on_rise_diff_d;
                delta_sum_t2 <= st1_st2_on_high_diff_d;
                delta_sum_t3 <= st1_st2_off_rise_diff_d;
                delta_sum_t4 <= st1_st2_off_high_diff_d;
            end else if (continuous_cycle_counter_d2 >= 1 && continuous_cycle_counter_d2 < CONTINUOUS_MAX_CYCLES) begin
                // 第二个周期开始：迭代累加
                delta_sum_t1 <= delta_t1 + st1_st2_on_rise_diff_d;
                delta_sum_t2 <= delta_t2 + st1_st2_on_high_diff_d;
                delta_sum_t3 <= delta_t3 + st1_st2_off_rise_diff_d;
                delta_sum_t4 <= delta_t4 + st1_st2_off_high_diff_d;
            end
        end else begin
            if (current_state == IDLE || reset_rising || !ENABLE_ADJUST) begin
                delta_sum_t1 <= 0; delta_sum_t2 <= 0; delta_sum_t3 <= 0; delta_sum_t4 <= 0;
            end
        end
    end
end

// 第四阶段：范围限制
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        delta_t1 <= 0; delta_t2 <= 0; delta_t3 <= 0; delta_t4 <= 0;
    end else if (!locked) begin
        delta_t1 <= 0; delta_t2 <= 0; delta_t3 <= 0; delta_t4 <= 0;
    end else begin
        if (calc_enable_d3) begin
            // 范围限制单独作为一个阶段，减少组合逻辑深度
            delta_t1 <= (delta_sum_t1 > 40) ? 40 : 
                       ((delta_sum_t1 < -40) ? -40 : delta_sum_t1);
            delta_t2 <= (delta_sum_t2 > 20) ? 20 : 
                       ((delta_sum_t2 < -20) ? -20 : delta_sum_t2);
            delta_t3 <= (delta_sum_t3 > 40) ? 40 : 
                       ((delta_sum_t3 < -40) ? -40 : delta_sum_t3);
            delta_t4 <= (delta_sum_t4 > 20) ? 20 : 
                       ((delta_sum_t4 < -20) ? -20 : delta_sum_t4);
        end else begin
            if (current_state == IDLE || reset_rising || !ENABLE_ADJUST) begin
                delta_t1 <= 0; delta_t2 <= 0; delta_t3 <= 0; delta_t4 <= 0;
            end
        end
    end
end

    // -------------------------
    // PWM微调逻辑
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm2_rise <= 0;
            pwm2_fall <= PWM_HIGH_CYCLES;
        end else if (!locked) begin
            pwm2_rise <= 0;
            pwm2_fall <= PWM_HIGH_CYCLES;
        end else if (ENABLE_ADJUST) begin
            if (current_state == CONTINUOUS_PWM) begin
                pwm2_rise <= delta_t1 + delta_t2 ;
                pwm2_fall <= PWM_HIGH_CYCLES + delta_t3 + delta_t4 ;
            end else begin
                pwm2_rise <= 0;
                pwm2_fall <= PWM_HIGH_CYCLES;
            end
        end else begin
            pwm2_rise <= 0;
            pwm2_fall <= PWM_HIGH_CYCLES;
        end
    end

    // -------------------------
    // 状态机时序逻辑
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= WAIT_LOCK;
        end else if (!locked) begin
            current_state <= WAIT_LOCK;
        end else begin
            current_state <= next_state;
        end
    end

    // 状态机组合逻辑
    always @(*) begin
        next_state = current_state;
        case (current_state)
            WAIT_LOCK: next_state = locked ? IDLE : WAIT_LOCK;
            IDLE: begin
                if (reset_rising) next_state = IDLE;
                else if (key1_rising) next_state = CONTINUOUS_PWM;
                else if (key2_rising) next_state = DUAL_PULSE_HIGH1;
                else next_state = IDLE;
            end
            CONTINUOUS_PWM: begin
                if (reset_rising) next_state = IDLE;
                else if (continuous_cycle_counter >= CONTINUOUS_MAX_CYCLES) next_state = IDLE;
                else next_state = CONTINUOUS_PWM;
            end
            DUAL_PULSE_HIGH1: begin
                if (reset_rising) next_state = IDLE;
                else if (dual_pulse_counter >= HIGH_PULSE1_CYCLES - 1) next_state = DUAL_PULSE_LOW;
                else next_state = DUAL_PULSE_HIGH1;
            end
            DUAL_PULSE_LOW: begin
                if (reset_rising) next_state = IDLE;
                else if (dual_pulse_counter >= (HIGH_PULSE1_CYCLES + LOW_PULSE_CYCLES) - 1) next_state = DUAL_PULSE_HIGH2;
                else next_state = DUAL_PULSE_LOW;
            end
            DUAL_PULSE_HIGH2: begin
                if (reset_rising) next_state = IDLE;
                else if (dual_pulse_counter >= (HIGH_PULSE1_CYCLES + LOW_PULSE_CYCLES + HIGH_PULSE2_CYCLES) - 1) next_state = IDLE;
                else next_state = DUAL_PULSE_HIGH2;
            end
            default: next_state = IDLE;
        endcase
    end

    // -------------------------
    // 计数器逻辑
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 32'd0;
            dual_pulse_counter <= 32'd0;
            continuous_cycle_counter <= 32'd0;
        end else if (!locked) begin
            pwm_counter <= 32'd0;
            dual_pulse_counter <= 32'd0;
            continuous_cycle_counter <= 32'd0;
        end else begin
            if (reset_rising) begin
                pwm_counter <= 32'd0;
                dual_pulse_counter <= 32'd0;
                continuous_cycle_counter <= 32'd0;
            end else begin
                // PWM计数器
                if (current_state == CONTINUOUS_PWM) begin
                    if (pwm_counter >= PWM_PERIOD_CYCLES - 1) begin
                        pwm_counter <= 32'd0;
                        if (continuous_cycle_counter < CONTINUOUS_MAX_CYCLES) begin
                            continuous_cycle_counter <= continuous_cycle_counter + 1'b1;
                        end
                    end else begin
                        pwm_counter <= pwm_counter + 1'b1;
                    end
                end else begin
                    pwm_counter <= 32'd0;
                end

                // 双脉冲计数器
                if (current_state == DUAL_PULSE_HIGH1 || current_state == DUAL_PULSE_LOW || current_state == DUAL_PULSE_HIGH2) begin
                    dual_pulse_counter <= dual_pulse_counter + 1'b1;
                end else begin
                    dual_pulse_counter <= 32'd0;
                end

                // 连续周期计数器
                if (current_state != CONTINUOUS_PWM) begin
                    continuous_cycle_counter <= 32'd0;
                end
            end
        end
    end

    // -------------------------
    // PWM输出逻辑
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PWM1 <= 1'b0;
            PWM2 <= 1'b0;
        end else if (!locked) begin
            PWM1 <= 1'b0;
            PWM2 <= 1'b0;
        end else begin
            case (current_state)
                IDLE: begin
                    PWM1 <= 1'b0;
                    PWM2 <= 1'b0;
                end
                CONTINUOUS_PWM: begin
                    if (continuous_cycle_counter >= CONTINUOUS_MAX_CYCLES) begin
                        PWM1 <= 1'b0;
                        PWM2 <= 1'b0;
                    end else begin
                        PWM1 <= (pwm_counter < PWM_HIGH_CYCLES);
                        if (ENABLE_ADJUST) begin
                            if (pwm2_rise < 0) begin
                                PWM2 <= ($signed(pwm_counter) >= PWM_PERIOD_CYCLES + pwm2_rise) || ($signed(pwm_counter) < pwm2_fall);
                            end else begin
                                PWM2 <= ($signed(pwm_counter) >= pwm2_rise) && ($signed(pwm_counter) < pwm2_fall);
                            end
                        end else begin
                            PWM2 <= (pwm_counter < PWM_HIGH_CYCLES);
                        end
                    end
                end
                DUAL_PULSE_HIGH1: begin
                    PWM1 <= 1'b1;
                    PWM2 <= 1'b1;
                end
                DUAL_PULSE_LOW: begin
                    PWM1 <= 1'b0;
                    PWM2 <= 1'b0;
                end
                DUAL_PULSE_HIGH2: begin
                    PWM1 <= 1'b1;
                    PWM2 <= 1'b1;
                end
                default: begin
                    PWM1 <= 1'b0;
                    PWM2 <= 1'b0;
                end
            endcase
        end
    end

endmodule