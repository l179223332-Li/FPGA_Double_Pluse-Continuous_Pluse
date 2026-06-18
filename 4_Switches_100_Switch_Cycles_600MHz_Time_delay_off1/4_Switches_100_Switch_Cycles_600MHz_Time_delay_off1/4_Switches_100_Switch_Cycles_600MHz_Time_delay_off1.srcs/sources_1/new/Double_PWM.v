`timescale 1ns / 1ps

module Double_PWM (
    input wire sys_clk,       // 50MHz时钟
    input wire rst_n,         // 复位信号，低电平有效
    input wire key1,          // 连续PWM控制按键
    input wire key2,          // 双脉冲控制按键
    input wire reset_key,     // 复位按键
    input wire st1_off,       // 管1关断脉冲
    input wire st1_on,        // 管1开通脉冲
    input wire st2_off,       // 管2关断脉冲
    input wire st2_on,        // 管2开通脉冲
    input wire st3_off,       // 管3关断脉冲
    input wire st3_on,        // 管3开通脉冲
    input wire st4_off,       // 管4关断脉冲
    input wire st4_on,        // 管4开通脉冲
    output reg PWM1,          // PWM1输出
    output reg PWM2,          // PWM2输出
    output reg PWM3,          // PWM3输出
    output reg PWM4,           // PWM4输出
    output wire led_out1,
    output wire led_out2,
    output wire led_out3,
    output wire led_out4
);

    // 600MHz时钟倍频
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

    // 参数定义
    parameter CLK_FREQ = 200;                    // 200MHz
    parameter PWM_PERIOD = 50;                   // 50us周期
    parameter PWM_DUTY = 50;                     // 50%占空比
    parameter HIGH_PULSE1 = 30;                  // 第一个高脉冲30us
    parameter LOW_PULSE = 15;                    // 低脉冲15us
    parameter HIGH_PULSE2 = 10;                  // 第二个高脉冲10us
    parameter DEBOUNCE_TIME = 20;               // 消抖时间20ms
    parameter ENABLE_DEBOUNCE = 1;               // 消抖使能：1=启用，0=停用
    parameter CONTINUOUS_MAX_CYCLES = 80;        // 连续PWM最大输出周期数
    parameter ENABLE_ADJUST = 0;                 // PWM微调使能：1=启用ST采样调整，0=禁用

    // Delta范围保护参数
    parameter DELTA_RISE_MAX = 40;               // 上升沿最大调整量
    parameter DELTA_RISE_MIN = -40;              // 上升沿最小调整量
    parameter DELTA_HIGH_MAX = 20;               // 高电平最大调整量
    parameter DELTA_HIGH_MIN = -20;              // 高电平最小调整量
    parameter PWM_ADJUST_MAX = 1000;             // PWM调整最大限制

    // 时间-周期数精确映射
    parameter CLK_CYCLES_PER_US = CLK_FREQ;      // 每微秒的时钟周期数（200）

    // 精确计算各阶段周期数
    parameter PWM_PERIOD_CYCLES = PWM_PERIOD * CLK_CYCLES_PER_US;           // 50us → 10000周期
    parameter PWM_HIGH_CYCLES = (PWM_PERIOD_CYCLES * PWM_DUTY) / 100;       // 50%占空比 → 5000周期
    parameter HIGH_PULSE1_CYCLES = HIGH_PULSE1 * CLK_CYCLES_PER_US;         // 30us → 6000周期
    parameter LOW_PULSE_CYCLES = LOW_PULSE * CLK_CYCLES_PER_US;             // 15us → 3000周期
    parameter HIGH_PULSE2_CYCLES = HIGH_PULSE2 * CLK_CYCLES_PER_US;         // 10us → 2000周期
    parameter DEBOUNCE_CYCLES = DEBOUNCE_TIME * 1000 * CLK_CYCLES_PER_US;   // 20ms → 4,000,000周期

    // 状态机定义（新增 WAIT_LOCK）
    localparam IDLE = 3'b000;
    localparam CONTINUOUS_PWM = 3'b001;
    localparam DUAL_PULSE_HIGH1 = 3'b010;
    localparam DUAL_PULSE_LOW = 3'b011;
    localparam DUAL_PULSE_HIGH2 = 3'b100;
    localparam WAIT_LOCK = 3'b111; // 等待 PLL 锁定

    // 寄存器定义
    reg [2:0] current_state, next_state;
    reg [31:0] pwm_counter;                       // 连续PWM周期计数器
    reg [31:0] dual_pulse_counter;                // 双脉冲计数器
    reg [31:0] continuous_cycle_counter;          // 连续脉冲周期计数
    
    assign led_out3 = ~(current_state == IDLE);
    assign led_out4 = ~(current_state != IDLE);

    // 按键消抖相关寄存器
    reg [31:0] key1_debounce_counter;
    reg [31:0] key2_debounce_counter;
    reg [31:0] reset_debounce_counter;
    reg key1_debounced;
    reg key2_debounced;
    reg reset_debounced;
    reg key1_prev, key2_prev, reset_prev;
    wire key1_rising, key2_rising, reset_rising;

    // ST采样相关寄存器
    reg [31:0] st1_off_rise, st1_off_high;
    reg [31:0] st1_on_rise, st1_on_high;
    reg [31:0] st2_off_rise, st2_off_high;
    reg [31:0] st2_on_rise, st2_on_high;
    reg [31:0] st3_off_rise, st3_off_high;
    reg [31:0] st3_on_rise, st3_on_high;
    reg [31:0] st4_off_rise, st4_off_high;
    reg [31:0] st4_on_rise, st4_on_high;
    
    reg [1:0] st1_off_sync, st1_on_sync, st2_off_sync, st2_on_sync;
    reg [1:0] st3_off_sync, st3_on_sync, st4_off_sync, st4_on_sync;
    reg st1_off_prev, st1_on_prev, st2_off_prev, st2_on_prev;
    reg st3_off_prev, st3_on_prev, st4_off_prev, st4_on_prev;

    // 时间差delta寄存器（带符号）
    reg signed [31:0] delta_t21, delta_t22, delta_t23, delta_t24;
    reg signed [31:0] delta_t31, delta_t32, delta_t33, delta_t34;
    reg signed [31:0] delta_t41, delta_t42, delta_t43, delta_t44;

    // PWM微调相关寄存器
    reg signed [31:0] pwm2_rise, pwm2_fall;
    reg signed [31:0] pwm3_rise, pwm3_fall;
    reg signed [31:0] pwm4_rise, pwm4_fall;

    // 将按键上升沿检测做成组合信号
    assign key1_rising = key1_debounced & ~key1_prev;
    assign key2_rising = key2_debounced & ~key2_prev;
    assign reset_rising = reset_debounced & ~reset_prev;

    // -------------------------
    // 按键消抖逻辑 - 分频优化版
    // -------------------------

    // 添加分频计数器，降低消抖逻辑的工作频率
    reg [7:0] debounce_clk_div;
    wire debounce_clk_en = (debounce_clk_div == 8'd199); // 200分频，1MHz

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debounce_clk_div <= 8'd0;
        end else begin
            debounce_clk_div <= debounce_clk_div + 1'b1;
        end
    end

    // 调整消抖周期数（因为时钟分频了）
    localparam DEBOUNCE_CYCLES_DIV = DEBOUNCE_TIME * 1000; // 20ms * 1000 = 20000 cycles

    // 优化后的按键消抖逻辑（使用分频时钟）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key1_debounce_counter <= 32'd0;
            key2_debounce_counter <= 32'd0;
            reset_debounce_counter <= 32'd0;
            key1_debounced <= 1'b0;
            key2_debounced <= 1'b0;
            reset_debounced <= 1'b0;
        end else if (!locked) begin
            key1_debounce_counter <= 32'd0;
            key2_debounce_counter <= 32'd0;
            reset_debounce_counter <= 32'd0;
            key1_debounced <= 1'b0;
            key2_debounced <= 1'b0;
            reset_debounced <= 1'b0;
        end else if (debounce_clk_en && ENABLE_DEBOUNCE) begin
            // 只在分频时钟使能时处理消抖，大幅减少逻辑活动
            
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
            // 直接透传，不消抖
            key1_debounced <= key1;
            key2_debounced <= key2;
            reset_debounced <= reset_key;
        end
    end

    // prev更新逻辑保持不变
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
    reg st3_off_sync1, st3_on_sync1, st4_off_sync1, st4_on_sync1;
    reg st1_off_prev_d, st1_on_prev_d, st2_off_prev_d, st2_on_prev_d;
    reg st3_off_prev_d, st3_on_prev_d, st4_off_prev_d, st4_on_prev_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st1_off_sync <= 2'b00; st1_on_sync <= 2'b00;
            st2_off_sync <= 2'b00; st2_on_sync <= 2'b00;
            st3_off_sync <= 2'b00; st3_on_sync <= 2'b00;
            st4_off_sync <= 2'b00; st4_on_sync <= 2'b00;
            st1_off_sync1 <= 1'b0; st1_on_sync1 <= 1'b0;
            st2_off_sync1 <= 1'b0; st2_on_sync1 <= 1'b0;
            st3_off_sync1 <= 1'b0; st3_on_sync1 <= 1'b0;
            st4_off_sync1 <= 1'b0; st4_on_sync1 <= 1'b0;
            st1_off_prev <= 1'b0; st1_on_prev <= 1'b0;
            st2_off_prev <= 1'b0; st2_on_prev <= 1'b0;
            st3_off_prev <= 1'b0; st3_on_prev <= 1'b0;
            st4_off_prev <= 1'b0; st4_on_prev <= 1'b0;
            st1_off_prev_d <= 1'b0; st1_on_prev_d <= 1'b0;
            st2_off_prev_d <= 1'b0; st2_on_prev_d <= 1'b0;
            st3_off_prev_d <= 1'b0; st3_on_prev_d <= 1'b0;
            st4_off_prev_d <= 1'b0; st4_on_prev_d <= 1'b0;
        end else if (!locked) begin
            st1_off_sync <= 2'b00; st1_on_sync <= 2'b00;
            st2_off_sync <= 2'b00; st2_on_sync <= 2'b00;
            st3_off_sync <= 2'b00; st3_on_sync <= 2'b00;
            st4_off_sync <= 2'b00; st4_on_sync <= 2'b00;
            st1_off_sync1 <= 1'b0; st1_on_sync1 <= 1'b0;
            st2_off_sync1 <= 1'b0; st2_on_sync1 <= 1'b0;
            st3_off_sync1 <= 1'b0; st3_on_sync1 <= 1'b0;
            st4_off_sync1 <= 1'b0; st4_on_sync1 <= 1'b0;
            st1_off_prev <= 1'b0; st1_on_prev <= 1'b0;
            st2_off_prev <= 1'b0; st2_on_prev <= 1'b0;
            st3_off_prev <= 1'b0; st3_on_prev <= 1'b0;
            st4_off_prev <= 1'b0; st4_on_prev <= 1'b0;
            st1_off_prev_d <= 1'b0; st1_on_prev_d <= 1'b0;
            st2_off_prev_d <= 1'b0; st2_on_prev_d <= 1'b0;
            st3_off_prev_d <= 1'b0; st3_on_prev_d <= 1'b0;
            st4_off_prev_d <= 1'b0; st4_on_prev_d <= 1'b0;
        end else if (ENABLE_ADJUST) begin
            // 三级同步器减少亚稳态
            st1_off_sync <= {st1_off_sync[0], st1_off};
            st1_on_sync <= {st1_on_sync[0], st1_on};
            st2_off_sync <= {st2_off_sync[0], st2_off};
            st2_on_sync <= {st2_on_sync[0], st2_on};
            st3_off_sync <= {st3_off_sync[0], st3_off};
            st3_on_sync <= {st3_on_sync[0], st3_on};
            st4_off_sync <= {st4_off_sync[0], st4_off};
            st4_on_sync <= {st4_on_sync[0], st4_on};
            
            st1_off_sync1 <= st1_off_sync[1];
            st1_on_sync1 <= st1_on_sync[1];
            st2_off_sync1 <= st2_off_sync[1];
            st2_on_sync1 <= st2_on_sync[1];
            st3_off_sync1 <= st3_off_sync[1];
            st3_on_sync1 <= st3_on_sync[1];
            st4_off_sync1 <= st4_off_sync[1];
            st4_on_sync1 <= st4_on_sync[1];
            
            // 增加流水线延迟，减少组合逻辑路径
            st1_off_prev_d <= st1_off_sync1;
            st1_on_prev_d <= st1_on_sync1;
            st2_off_prev_d <= st2_off_sync1;
            st2_on_prev_d <= st2_on_sync1;
            st3_off_prev_d <= st3_off_sync1;
            st3_on_prev_d <= st3_on_sync1;
            st4_off_prev_d <= st4_off_sync1;
            st4_on_prev_d <= st4_on_sync1;
            
            st1_off_prev <= st1_off_prev_d;
            st1_on_prev <= st1_on_prev_d;
            st2_off_prev <= st2_off_prev_d;
            st2_on_prev <= st2_on_prev_d;
            st3_off_prev <= st3_off_prev_d;
            st3_on_prev <= st3_on_prev_d;
            st4_off_prev <= st4_off_prev_d;
            st4_on_prev <= st4_on_prev_d;
        end
    end

    // 边沿检测信号 - 寄存器输出优化
    reg st1_off_rising_r, st1_off_falling_r;
    reg st1_on_rising_r, st1_on_falling_r;
    reg st2_off_rising_r, st2_off_falling_r;
    reg st2_on_rising_r, st2_on_falling_r;
    reg st3_off_rising_r, st3_off_falling_r;
    reg st3_on_rising_r, st3_on_falling_r;
    reg st4_off_rising_r, st4_off_falling_r;
    reg st4_on_rising_r, st4_on_falling_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st1_off_rising_r <= 1'b0; st1_off_falling_r <= 1'b0;
            st1_on_rising_r <= 1'b0; st1_on_falling_r <= 1'b0;
            st2_off_rising_r <= 1'b0; st2_off_falling_r <= 1'b0;
            st2_on_rising_r <= 1'b0; st2_on_falling_r <= 1'b0;
            st3_off_rising_r <= 1'b0; st3_off_falling_r <= 1'b0;
            st3_on_rising_r <= 1'b0; st3_on_falling_r <= 1'b0;
            st4_off_rising_r <= 1'b0; st4_off_falling_r <= 1'b0;
            st4_on_rising_r <= 1'b0; st4_on_falling_r <= 1'b0;
        end else if (!locked) begin
            st1_off_rising_r <= 1'b0; st1_off_falling_r <= 1'b0;
            st1_on_rising_r <= 1'b0; st1_on_falling_r <= 1'b0;
            st2_off_rising_r <= 1'b0; st2_off_falling_r <= 1'b0;
            st2_on_rising_r <= 1'b0; st2_on_falling_r <= 1'b0;
            st3_off_rising_r <= 1'b0; st3_off_falling_r <= 1'b0;
            st3_on_rising_r <= 1'b0; st3_on_falling_r <= 1'b0;
            st4_off_rising_r <= 1'b0; st4_off_falling_r <= 1'b0;
            st4_on_rising_r <= 1'b0; st4_on_falling_r <= 1'b0;
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
            st3_off_rising_r <= st3_off_sync1 && !st3_off_prev_d;
            st3_off_falling_r <= !st3_off_sync1 && st3_off_prev_d;
            st3_on_rising_r <= st3_on_sync1 && !st3_on_prev_d;
            st3_on_falling_r <= !st3_on_sync1 && st3_on_prev_d;
            st4_off_rising_r <= st4_off_sync1 && !st4_off_prev_d;
            st4_off_falling_r <= !st4_off_sync1 && st4_off_prev_d;
            st4_on_rising_r <= st4_on_sync1 && !st4_on_prev_d;
            st4_on_falling_r <= !st4_on_sync1 && st4_on_prev_d;
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
    // ST采样状态机定义
    // -------------------------
    localparam SAMPLE_IDLE = 3'b000;
    localparam SAMPLE_RISING_DETECTED = 3'b001;
    localparam SAMPLE_WAIT_FALL = 3'b010;
    localparam SAMPLE_COMPLETE = 3'b011;

    // ST采样状态寄存器
    reg [2:0] st1_off_sample_state, st1_on_sample_state;
    reg [2:0] st2_off_sample_state, st2_on_sample_state;
    reg [2:0] st3_off_sample_state, st3_on_sample_state;
    reg [2:0] st4_off_sample_state, st4_on_sample_state;

    // 增加中间寄存器来存储计算值
    reg [31:0] pwm_counter_minus_rise_st1_off, pwm_counter_minus_rise_st1_on;
    reg [31:0] pwm_counter_minus_rise_st2_off, pwm_counter_minus_rise_st2_on;
    reg [31:0] pwm_counter_minus_rise_st3_off, pwm_counter_minus_rise_st3_on;
    reg [31:0] pwm_counter_minus_rise_st4_off, pwm_counter_minus_rise_st4_on;

    // 为每个ST采样状态机实现类似的优化结构
    // 这里以ST1为例，其他ST2、ST3、ST4类似实现

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

    // 类似的优化实现ST2、ST3、ST4的采样状态机...
    // 这里省略具体实现，结构与ST1相同

    // -------------------------
    // Delta计算逻辑（深度流水线优化版 + 范围保护）
    // -------------------------

    // 增加更多流水线阶段
    reg signed [31:0] st1_off_rise_d1, st1_off_high_d1, st1_on_rise_d1, st1_on_high_d1;
    reg signed [31:0] st2_off_rise_d1, st2_off_high_d1, st2_on_rise_d1, st2_on_high_d1;
    reg signed [31:0] st3_off_rise_d1, st3_off_high_d1, st3_on_rise_d1, st3_on_high_d1;
    reg signed [31:0] st4_off_rise_d1, st4_off_high_d1, st4_on_rise_d1, st4_on_high_d1;

    // 第一阶段：输入寄存器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st1_off_rise_d1 <= 0; st1_off_high_d1 <= 0;
            st1_on_rise_d1 <= 0; st1_on_high_d1 <= 0;
            st2_off_rise_d1 <= 0; st2_off_high_d1 <= 0;
            st2_on_rise_d1 <= 0; st2_on_high_d1 <= 0;
            st3_off_rise_d1 <= 0; st3_off_high_d1 <= 0;
            st3_on_rise_d1 <= 0; st3_on_high_d1 <= 0;
            st4_off_rise_d1 <= 0; st4_off_high_d1 <= 0;
            st4_on_rise_d1 <= 0; st4_on_high_d1 <= 0;
        end else if (!locked) begin
            st1_off_rise_d1 <= 0; st1_off_high_d1 <= 0;
            st1_on_rise_d1 <= 0; st1_on_high_d1 <= 0;
            st2_off_rise_d1 <= 0; st2_off_high_d1 <= 0;
            st2_on_rise_d1 <= 0; st2_on_high_d1 <= 0;
            st3_off_rise_d1 <= 0; st3_off_high_d1 <= 0;
            st3_on_rise_d1 <= 0; st3_on_high_d1 <= 0;
            st4_off_rise_d1 <= 0; st4_off_high_d1 <= 0;
            st4_on_rise_d1 <= 0; st4_on_high_d1 <= 0;
        end else begin
            // 输入数据打拍
            st1_off_rise_d1 <= st1_off_rise; st1_off_high_d1 <= st1_off_high;
            st1_on_rise_d1 <= st1_on_rise; st1_on_high_d1 <= st1_on_high;
            st2_off_rise_d1 <= st2_off_rise; st2_off_high_d1 <= st2_off_high;
            st2_on_rise_d1 <= st2_on_rise; st2_on_high_d1 <= st2_on_high;
            st3_off_rise_d1 <= st3_off_rise; st3_off_high_d1 <= st3_off_high;
            st3_on_rise_d1 <= st3_on_rise; st3_on_high_d1 <= st3_on_high;
            st4_off_rise_d1 <= st4_off_rise; st4_off_high_d1 <= st4_off_high;
            st4_on_rise_d1 <= st4_on_rise; st4_on_high_d1 <= st4_on_high;
        end
    end

    // 第二阶段：减法计算
    reg signed [31:0] st1_st2_off_rise_diff_d, st1_st2_off_high_diff_d;
    reg signed [31:0] st1_st2_on_rise_diff_d, st1_st2_on_high_diff_d;
    reg signed [31:0] st1_st3_off_rise_diff_d, st1_st3_off_high_diff_d;
    reg signed [31:0] st1_st3_on_rise_diff_d, st1_st3_on_high_diff_d;
    reg signed [31:0] st1_st4_off_rise_diff_d, st1_st4_off_high_diff_d;
    reg signed [31:0] st1_st4_on_rise_diff_d, st1_st4_on_high_diff_d;
    
    reg calc_enable_d1, calc_enable_d2, calc_enable_d3, calc_enable_d4;
    reg [31:0] continuous_cycle_counter_d1, continuous_cycle_counter_d2;
    reg [31:0] continuous_cycle_counter_d3, continuous_cycle_counter_d4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st1_st2_off_rise_diff_d <= 0; st1_st2_off_high_diff_d <= 0;
            st1_st2_on_rise_diff_d <= 0; st1_st2_on_high_diff_d <= 0;
            st1_st3_off_rise_diff_d <= 0; st1_st3_off_high_diff_d <= 0;
            st1_st3_on_rise_diff_d <= 0; st1_st3_on_high_diff_d <= 0;
            st1_st4_off_rise_diff_d <= 0; st1_st4_off_high_diff_d <= 0;
            st1_st4_on_rise_diff_d <= 0; st1_st4_on_high_diff_d <= 0;
            calc_enable_d1 <= 1'b0;
            continuous_cycle_counter_d1 <= 32'd0;
        end else if (!locked) begin
            st1_st2_off_rise_diff_d <= 0; st1_st2_off_high_diff_d <= 0;
            st1_st2_on_rise_diff_d <= 0; st1_st2_on_high_diff_d <= 0;
            st1_st3_off_rise_diff_d <= 0; st1_st3_off_high_diff_d <= 0;
            st1_st3_on_rise_diff_d <= 0; st1_st3_on_high_diff_d <= 0;
            st1_st4_off_rise_diff_d <= 0; st1_st4_off_high_diff_d <= 0;
            st1_st4_on_rise_diff_d <= 0; st1_st4_on_high_diff_d <= 0;
            calc_enable_d1 <= 1'b0;
            continuous_cycle_counter_d1 <= 32'd0;
        end else begin
            // 计算使能信号延迟
            calc_enable_d1 <= ENABLE_ADJUST && (current_state == CONTINUOUS_PWM);
            continuous_cycle_counter_d1 <= continuous_cycle_counter;
            
            if (ENABLE_ADJUST && (current_state == CONTINUOUS_PWM)) begin
                // 第一阶段：只做减法运算，避免复杂的条件判断
                st1_st2_off_rise_diff_d <= $signed(st1_off_rise_d1) - $signed(st2_off_rise_d1);
                st1_st2_off_high_diff_d <= $signed(st1_off_high_d1) - $signed(st2_off_high_d1);
                st1_st2_on_rise_diff_d <= $signed(st1_on_rise_d1) - $signed(st2_on_rise_d1);
                st1_st2_on_high_diff_d <= $signed(st1_on_high_d1) - $signed(st2_on_high_d1);
                
                st1_st3_off_rise_diff_d <= $signed(st1_off_rise_d1) - $signed(st3_off_rise_d1);
                st1_st3_off_high_diff_d <= $signed(st1_off_high_d1) - $signed(st3_off_high_d1);
                st1_st3_on_rise_diff_d <= $signed(st1_on_rise_d1) - $signed(st3_on_rise_d1);
                st1_st3_on_high_diff_d <= $signed(st1_on_high_d1) - $signed(st3_on_high_d1);
                
                st1_st4_off_rise_diff_d <= $signed(st1_off_rise_d1) - $signed(st4_off_rise_d1);
                st1_st4_off_high_diff_d <= $signed(st1_off_high_d1) - $signed(st4_off_high_d1);
                st1_st4_on_rise_diff_d <= $signed(st1_on_rise_d1) - $signed(st4_on_rise_d1);
                st1_st4_on_high_diff_d <= $signed(st1_on_high_d1) - $signed(st4_on_high_d1);
            end else begin
                st1_st2_off_rise_diff_d <= 0; st1_st2_off_high_diff_d <= 0;
                st1_st2_on_rise_diff_d <= 0; st1_st2_on_high_diff_d <= 0;
                st1_st3_off_rise_diff_d <= 0; st1_st3_off_high_diff_d <= 0;
                st1_st3_on_rise_diff_d <= 0; st1_st3_on_high_diff_d <= 0;
                st1_st4_off_rise_diff_d <= 0; st1_st4_off_high_diff_d <= 0;
                st1_st4_on_rise_diff_d <= 0; st1_st4_on_high_diff_d <= 0;
            end
        end
    end

    // 第三阶段：累加运算和范围保护
    reg signed [31:0] delta_sum_t21, delta_sum_t22, delta_sum_t23, delta_sum_t24;
    reg signed [31:0] delta_sum_t31, delta_sum_t32, delta_sum_t33, delta_sum_t34;
    reg signed [31:0] delta_sum_t41, delta_sum_t42, delta_sum_t43, delta_sum_t44;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            calc_enable_d2 <= 1'b0; calc_enable_d3 <= 1'b0; calc_enable_d4 <= 1'b0;
            continuous_cycle_counter_d2 <= 32'd0; continuous_cycle_counter_d3 <= 32'd0; continuous_cycle_counter_d4 <= 32'd0;
            
            delta_sum_t21 <= 0; delta_sum_t22 <= 0; delta_sum_t23 <= 0; delta_sum_t24 <= 0;
            delta_sum_t31 <= 0; delta_sum_t32 <= 0; delta_sum_t33 <= 0; delta_sum_t34 <= 0;
            delta_sum_t41 <= 0; delta_sum_t42 <= 0; delta_sum_t43 <= 0; delta_sum_t44 <= 0;
            
            delta_t21 <= 0; delta_t22 <= 0; delta_t23 <= 0; delta_t24 <= 0;
            delta_t31 <= 0; delta_t32 <= 0; delta_t33 <= 0; delta_t34 <= 0;
            delta_t41 <= 0; delta_t42 <= 0; delta_t43 <= 0; delta_t44 <= 0;
        end else if (!locked) begin
            calc_enable_d2 <= 1'b0; calc_enable_d3 <= 1'b0; calc_enable_d4 <= 1'b0;
            continuous_cycle_counter_d2 <= 32'd0; continuous_cycle_counter_d3 <= 32'd0; continuous_cycle_counter_d4 <= 32'd0;
            
            delta_sum_t21 <= 0; delta_sum_t22 <= 0; delta_sum_t23 <= 0; delta_sum_t24 <= 0;
            delta_sum_t31 <= 0; delta_sum_t32 <= 0; delta_sum_t33 <= 0; delta_sum_t34 <= 0;
            delta_sum_t41 <= 0; delta_sum_t42 <= 0; delta_sum_t43 <= 0; delta_sum_t44 <= 0;
            
            delta_t21 <= 0; delta_t22 <= 0; delta_t23 <= 0; delta_t24 <= 0;
            delta_t31 <= 0; delta_t32 <= 0; delta_t33 <= 0; delta_t34 <= 0;
            delta_t41 <= 0; delta_t42 <= 0; delta_t43 <= 0; delta_t44 <= 0;
        end else begin
            // 流水线延迟
            calc_enable_d2 <= calc_enable_d1;
            calc_enable_d3 <= calc_enable_d2;
            calc_enable_d4 <= calc_enable_d3;
            continuous_cycle_counter_d2 <= continuous_cycle_counter_d1;
            continuous_cycle_counter_d3 <= continuous_cycle_counter_d2;
            continuous_cycle_counter_d4 <= continuous_cycle_counter_d3;
            
            if (calc_enable_d1) begin
                // 第二阶段：根据条件进行赋值或累加
                if (continuous_cycle_counter_d1 == 0) begin
                    // 第一个周期：直接赋值
                    delta_sum_t21 <= st1_st2_off_rise_diff_d;
                    delta_sum_t22 <= st1_st2_off_high_diff_d;
                    delta_sum_t23 <= st1_st2_on_rise_diff_d;
                    delta_sum_t24 <= st1_st2_on_high_diff_d;
                    
                    delta_sum_t31 <= st1_st3_off_rise_diff_d;
                    delta_sum_t32 <= st1_st3_off_high_diff_d;
                    delta_sum_t33 <= st1_st3_on_rise_diff_d;
                    delta_sum_t34 <= st1_st3_on_high_diff_d;
                    
                    delta_sum_t41 <= st1_st4_off_rise_diff_d;
                    delta_sum_t42 <= st1_st4_off_high_diff_d;
                    delta_sum_t43 <= st1_st4_on_rise_diff_d;
                    delta_sum_t44 <= st1_st4_on_high_diff_d;
                end else if (continuous_cycle_counter_d1 >= 1 && continuous_cycle_counter_d1 < CONTINUOUS_MAX_CYCLES) begin
                    // 第二个周期开始：迭代累加
                    delta_sum_t21 <= delta_t21 + st1_st2_off_rise_diff_d;
                    delta_sum_t22 <= delta_t22 + st1_st2_off_high_diff_d;
                    delta_sum_t23 <= delta_t23 + st1_st2_on_rise_diff_d;
                    delta_sum_t24 <= delta_t24 + st1_st2_on_high_diff_d;
                    
                    delta_sum_t31 <= delta_t31 + st1_st3_off_rise_diff_d;
                    delta_sum_t32 <= delta_t32 + st1_st3_off_high_diff_d;
                    delta_sum_t33 <= delta_t33 + st1_st3_on_rise_diff_d;
                    delta_sum_t34 <= delta_t34 + st1_st3_on_high_diff_d;
                    
                    delta_sum_t41 <= delta_t41 + st1_st4_off_rise_diff_d;
                    delta_sum_t42 <= delta_t42 + st1_st4_off_high_diff_d;
                    delta_sum_t43 <= delta_t43 + st1_st4_on_rise_diff_d;
                    delta_sum_t44 <= delta_t44 + st1_st4_on_high_diff_d;
                end
            end else begin
                // 非连续PWM模式或微调禁用时清零
                if (current_state == IDLE || reset_rising || !ENABLE_ADJUST) begin
                    delta_sum_t21 <= 0; delta_sum_t22 <= 0; delta_sum_t23 <= 0; delta_sum_t24 <= 0;
                    delta_sum_t31 <= 0; delta_sum_t32 <= 0; delta_sum_t33 <= 0; delta_sum_t34 <= 0;
                    delta_sum_t41 <= 0; delta_sum_t42 <= 0; delta_sum_t43 <= 0; delta_sum_t44 <= 0;
                end
            end
            
            // 第四阶段：范围限制
            if (calc_enable_d2) begin
                // Delta范围保护
                delta_t21 <= (delta_sum_t21 > DELTA_RISE_MAX) ? DELTA_RISE_MAX : 
                           ((delta_sum_t21 < DELTA_RISE_MIN) ? DELTA_RISE_MIN : delta_sum_t21);
                delta_t22 <= (delta_sum_t22 > DELTA_HIGH_MAX) ? DELTA_HIGH_MAX : 
                           ((delta_sum_t22 < DELTA_HIGH_MIN) ? DELTA_HIGH_MIN : delta_sum_t22);
                delta_t23 <= (delta_sum_t23 > DELTA_RISE_MAX) ? DELTA_RISE_MAX : 
                           ((delta_sum_t23 < DELTA_RISE_MIN) ? DELTA_RISE_MIN : delta_sum_t23);
                delta_t24 <= (delta_sum_t24 > DELTA_HIGH_MAX) ? DELTA_HIGH_MAX : 
                           ((delta_sum_t24 < DELTA_HIGH_MIN) ? DELTA_HIGH_MIN : delta_sum_t24);
                
                delta_t31 <= (delta_sum_t31 > DELTA_RISE_MAX) ? DELTA_RISE_MAX : 
                           ((delta_sum_t31 < DELTA_RISE_MIN) ? DELTA_RISE_MIN : delta_sum_t31);
                delta_t32 <= (delta_sum_t32 > DELTA_HIGH_MAX) ? DELTA_HIGH_MAX : 
                           ((delta_sum_t32 < DELTA_HIGH_MIN) ? DELTA_HIGH_MIN : delta_sum_t32);
                delta_t33 <= (delta_sum_t33 > DELTA_RISE_MAX) ? DELTA_RISE_MAX : 
                           ((delta_sum_t33 < DELTA_RISE_MIN) ? DELTA_RISE_MIN : delta_sum_t33);
                delta_t34 <= (delta_sum_t34 > DELTA_HIGH_MAX) ? DELTA_HIGH_MAX : 
                           ((delta_sum_t34 < DELTA_HIGH_MIN) ? DELTA_HIGH_MIN : delta_sum_t34);
                
                delta_t41 <= (delta_sum_t41 > DELTA_RISE_MAX) ? DELTA_RISE_MAX : 
                           ((delta_sum_t41 < DELTA_RISE_MIN) ? DELTA_RISE_MIN : delta_sum_t41);
                delta_t42 <= (delta_sum_t42 > DELTA_HIGH_MAX) ? DELTA_HIGH_MAX : 
                           ((delta_sum_t42 < DELTA_HIGH_MIN) ? DELTA_HIGH_MIN : delta_sum_t42);
                delta_t43 <= (delta_sum_t43 > DELTA_RISE_MAX) ? DELTA_RISE_MAX : 
                           ((delta_sum_t43 < DELTA_RISE_MIN) ? DELTA_RISE_MIN : delta_sum_t43);
                delta_t44 <= (delta_sum_t44 > DELTA_HIGH_MAX) ? DELTA_HIGH_MAX : 
                           ((delta_sum_t44 < DELTA_HIGH_MIN) ? DELTA_HIGH_MIN : delta_sum_t44);
            end else begin
                if (current_state == IDLE || reset_rising || !ENABLE_ADJUST) begin
                    delta_t21 <= 0; delta_t22 <= 0; delta_t23 <= 0; delta_t24 <= 0;
                    delta_t31 <= 0; delta_t32 <= 0; delta_t33 <= 0; delta_t34 <= 0;
                    delta_t41 <= 0; delta_t42 <= 0; delta_t43 <= 0; delta_t44 <= 0;
                end
            end
        end
    end

    // -------------------------
    // PWM微调逻辑
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm2_rise <= 0; pwm2_fall <= PWM_HIGH_CYCLES;
            pwm3_rise <= 0; pwm3_fall <= PWM_HIGH_CYCLES;
            pwm4_rise <= 0; pwm4_fall <= PWM_HIGH_CYCLES;
        end else if (!locked) begin
            pwm2_rise <= 0; pwm2_fall <= PWM_HIGH_CYCLES;
            pwm3_rise <= 0; pwm3_fall <= PWM_HIGH_CYCLES;
            pwm4_rise <= 0; pwm4_fall <= PWM_HIGH_CYCLES;
        end else if (ENABLE_ADJUST) begin  // 只在微调使能时执行PWM微调逻辑
            if (current_state == CONTINUOUS_PWM) begin
                // 使用ST采样调整的PWM输出
                pwm2_rise <= delta_t23 + delta_t24 * 2;
                pwm2_fall <= PWM_HIGH_CYCLES + delta_t21 + delta_t22 * 2;
                
                pwm3_rise <= delta_t33 + delta_t34 * 2;
                pwm3_fall <= PWM_HIGH_CYCLES + delta_t31 + delta_t32 * 2;
                
                pwm4_rise <= delta_t43 + delta_t44 * 2;
                pwm4_fall <= PWM_HIGH_CYCLES + delta_t41 + delta_t42 * 2;
            end else begin
                pwm2_rise <= 0; pwm2_fall <= PWM_HIGH_CYCLES;
                pwm3_rise <= 0; pwm3_fall <= PWM_HIGH_CYCLES;
                pwm4_rise <= 0; pwm4_fall <= PWM_HIGH_CYCLES;
            end
        end else begin
            // 当微调不使能时，保持微调相关寄存器为默认值
            pwm2_rise <= 0; pwm2_fall <= PWM_HIGH_CYCLES;
            pwm3_rise <= 0; pwm3_fall <= PWM_HIGH_CYCLES;
            pwm4_rise <= 0; pwm4_fall <= PWM_HIGH_CYCLES;
        end
    end

    // -------------------------
    // 状态机时序逻辑（增加 WAIT_LOCK 逻辑）
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= WAIT_LOCK; // 复位后先等锁定
        end else if (!locked) begin
            current_state <= WAIT_LOCK; // 未锁定期间保持 WAIT_LOCK
        end else begin
            current_state <= next_state;
        end
    end

    // 状态机组合逻辑
    always @(*) begin
        next_state = current_state;

        case (current_state)
            WAIT_LOCK: begin
                if (locked)
                    next_state = IDLE;
                else
                    next_state = WAIT_LOCK;
            end

            IDLE: begin
                if (reset_rising)
                    next_state = IDLE;
                else if (key1_rising)
                    next_state = CONTINUOUS_PWM;
                else if (key2_rising)
                    next_state = DUAL_PULSE_HIGH1;
                else
                    next_state = IDLE;
            end

            CONTINUOUS_PWM: begin
                if (reset_rising)
                    next_state = IDLE;
                else if (continuous_cycle_counter >= CONTINUOUS_MAX_CYCLES)
                    next_state = IDLE;
                else
                    next_state = CONTINUOUS_PWM;
            end

            DUAL_PULSE_HIGH1: begin
                if (reset_rising)
                    next_state = IDLE;
                else if (dual_pulse_counter >= HIGH_PULSE1_CYCLES - 1)
                    next_state = DUAL_PULSE_LOW;
                else
                    next_state = DUAL_PULSE_HIGH1;
            end

            DUAL_PULSE_LOW: begin
                if (reset_rising)
                    next_state = IDLE;
                else if (dual_pulse_counter >= (HIGH_PULSE1_CYCLES + LOW_PULSE_CYCLES) - 1)
                    next_state = DUAL_PULSE_HIGH2;
                else
                    next_state = DUAL_PULSE_LOW;
            end

            DUAL_PULSE_HIGH2: begin
                if (reset_rising)
                    next_state = IDLE;
                else if (dual_pulse_counter >= (HIGH_PULSE1_CYCLES + LOW_PULSE_CYCLES + HIGH_PULSE2_CYCLES) - 1)
                    next_state = IDLE;
                else
                    next_state = DUAL_PULSE_HIGH2;
            end

            default:
                next_state = IDLE;
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
                // PWM计数器逻辑（仅在 CONTINUOUS_PWM 下计数）
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

                // dual_pulse_counter 在双脉冲模式中递增
                if (current_state == DUAL_PULSE_HIGH1 || current_state == DUAL_PULSE_LOW || current_state == DUAL_PULSE_HIGH2) begin
                    dual_pulse_counter <= dual_pulse_counter + 1'b1;
                end else begin
                    dual_pulse_counter <= 32'd0;
                end

                // continuous_cycle_counter 仅在 CONTINUOUS_PWM 下保持计数，其他状态清零
                if (current_state != CONTINUOUS_PWM) begin
                    continuous_cycle_counter <= 32'd0;
                end
            end
        end
    end

    // -------------------------
    // PWM输出逻辑（添加微调使能控制）
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PWM1 <= 1'b0;
            PWM2 <= 1'b0;
            PWM3 <= 1'b0;
            PWM4 <= 1'b0;
        end else if (!locked) begin
            PWM1 <= 1'b0;
            PWM2 <= 1'b0;
            PWM3 <= 1'b0;
            PWM4 <= 1'b0;
        end else begin
            case (current_state)
                IDLE: begin
                    PWM1 <= 1'b0;
                    PWM2 <= 1'b0;
                    PWM3 <= 1'b0;
                    PWM4 <= 1'b0;
                end

                CONTINUOUS_PWM: begin
                    if (continuous_cycle_counter >= CONTINUOUS_MAX_CYCLES) begin
                        PWM1 <= 1'b0;
                        PWM2 <= 1'b0;
                        PWM3 <= 1'b0;
                        PWM4 <= 1'b0;
                    end else begin
                        // PWM1固定50%占空比
                        PWM1 <= (pwm_counter < PWM_HIGH_CYCLES);

                        // 根据微调使能选择PWM输出方式
                        if (ENABLE_ADJUST) begin
                            // 启用微调：使用ST采样调整的PWM输出
                            // PWM2
                            if (pwm2_rise < 0) begin
                                PWM2 <= ($signed(pwm_counter) >= PWM_PERIOD_CYCLES + pwm2_rise) || ($signed(pwm_counter) < pwm2_fall);
                            end else begin
                                PWM2 <= ($signed(pwm_counter) >= pwm2_rise) && ($signed(pwm_counter) < pwm2_fall);
                            end
                            
                            // PWM3
                            if (pwm3_rise < 0) begin
                                PWM3 <= ($signed(pwm_counter) >= PWM_PERIOD_CYCLES + pwm3_rise) || ($signed(pwm_counter) < pwm3_fall);
                            end else begin
                                PWM3 <= ($signed(pwm_counter) >= pwm3_rise) && ($signed(pwm_counter) < pwm3_fall);
                            end
                            
                            // PWM4
                            if (pwm4_rise < 0) begin
                                PWM4 <= ($signed(pwm_counter) >= PWM_PERIOD_CYCLES + pwm4_rise) || ($signed(pwm_counter) < pwm4_fall);
                            end else begin
                                PWM4 <= ($signed(pwm_counter) >= pwm4_rise) && ($signed(pwm_counter) < pwm4_fall);
                            end
                        end else begin
                            // 禁用微调：PWM输出与PWM1相同（50%占空比）
                            PWM2 <= (pwm_counter < PWM_HIGH_CYCLES);
                            PWM3 <= (pwm_counter < PWM_HIGH_CYCLES);
                            PWM4 <= (pwm_counter < PWM_HIGH_CYCLES);
                        end
                    end
                end

                DUAL_PULSE_HIGH1: begin
                    PWM1 <= 1'b1;
                    PWM2 <= 1'b1;
                    PWM3 <= 1'b1;
                    PWM4 <= 1'b1;
                end

                DUAL_PULSE_LOW: begin
                    PWM1 <= 1'b0;
                    PWM2 <= 1'b0;
                    PWM3 <= 1'b0;
                    PWM4 <= 1'b0;
                end

                DUAL_PULSE_HIGH2: begin
                    PWM1 <= 1'b1;
                    PWM2 <= 1'b1;
                    PWM3 <= 1'b1;
                    PWM4 <= 1'b1;
                end

                default: begin
                    PWM1 <= 1'b0;
                    PWM2 <= 1'b0;
                    PWM3 <= 1'b0;
                    PWM4 <= 1'b0;
                end
            endcase
        end
    end

endmodule