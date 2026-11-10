#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};

// GM6020 current-mode group command for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);
const MAX_VOLTAGE_RAW: f32 = 20000.0; //【调试】电压最高输出

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// Feedback and current command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
}

impl Motor {
    /// Clear the current command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    pub fn read(&mut self, data: &[u8; 8]) {
        //angle
        let angle_raw =((data[0] as u16) << 8) | (data[1] as u16);
        self.angle_rad = (angle_raw as f32) * (2.0 * core::f32::consts::PI / 8192.0); //单圈rad
        //rotation speed
        let speed_raw = (((data[2] as u16) << 8) | (data[3] as u16)) as i16;
        self.speed_rpm = speed_raw as f32;
        //current
        self.current_raw = (((data[4] as u16) << 8) | (data[5] as u16)) as i16; //协议原始值
    }
}

/// PID gains, plus the integral and history kept between ticks.
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral: f32,
    pub previous_error: f32,
    pub integral_limit: f32,
    pub max_output: f32,
    pub first_run: bool,
}

impl Default for Pid {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            integral: 0.0,
            previous_error: 0.0,
            integral_limit: 0.0,
            max_output: 0.0,
            first_run: true,
        }
    }
}

impl Pid {
    pub fn reset(&mut self) { 
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.first_run = true;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns A. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        self.calculate_from_error(target - measured, dt_s)
    }

    pub fn calculate_from_error(&mut self, error: f32, dt_s: f32) -> f32 {
        //limit integral
        let i = if self.ki != 0.0 {
            self.integral += error * dt_s;
            self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
            self.ki * self.integral
        } else {
            self.integral = 0.0;
            0.0
        };

        //PID
        let p = self.kp * error;
        let d = if self.first_run {
            self.first_run = false;
            self.previous_error = error;
            0.0
        } else {
            let d_val = self.kd * (error - self.previous_error) / dt_s;
            self.previous_error = error;
            d_val
        };

        (p + i + d).clamp(-self.max_output, self.max_output)
    }  
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, raw voltage out.
    pub speed_pid: Pid,

    //电机物理零位
    pub reference_angle_rad: f32, 
    pub max_position_rpm: f32, // rad/s
    pub max_target_rate_rad_s: f32,
    pub rc_deadband: f32,

    //电机转动方向
    pub direction: f32, 
    
    // 故障检测：疑似堵转
    pub stall_ticks: u16,
    // 故障检测：高频振荡
    pub oscillation_ticks: u16,
    pub last_oscillation_sign: i8, // 记录上一次误差符号，用于计算振荡周期
    pub oscillation_periods: u16,  // 累计完整周期的次数

    //摇杆中位
    pub remote_center: f32,
}

/// State shared by every control tick.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    pub last_right_switch: u8,
    pub last_left_switch: u8, // 用于故障恢复动作判断
    pub yaw_feedback_age: u16,   
    pub pitch_feedback_age: u16, 
     /// 回正超时计时
    pub prepare_timeout_ticks: u16,
    /// 回正稳定计时
    pub prepare_settle_ticks: u16,

}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {

        // yaw and pitch angle
        let mut yaw = Axis::default();
        yaw.reference_angle_rad = 2.653; // 【调试】yaw零位 正前方
        yaw.max_position_rpm = 100.0;  // 默认位置环输出限幅，速度环目标最大 100 rpm
        yaw.max_target_rate_rad_s = 2.0;
        yaw.rc_deadband = 0.1;
        yaw.remote_center = 0.0; // 【调试】yaw中位

        yaw.position_pid.kp = 300.0;
        yaw.position_pid.ki = 0.0;
        yaw.position_pid.max_output = 100.0; // 对应 max_position_rpm
        yaw.position_pid.integral_limit = 10.0; // 积分抗饱和，具体数值待调

        yaw.speed_pid.kp = 150.0;
        yaw.speed_pid.ki = 0.0;
        yaw.speed_pid.kd = 0.0;
        yaw.speed_pid.max_output = 10000.0; // 电压原始值限幅 (范围是 -25000~25000)
        yaw.speed_pid.integral_limit = 100.0;

        // 配置 pitch 轴 PID 与参考角度
        let mut pitch = Axis::default();
        pitch.reference_angle_rad = 3.12; // 【调试】pitch零位 水平
        pitch.max_position_rpm = 100.0;
        pitch.max_target_rate_rad_s = 2.0;
        pitch.rc_deadband = 0.1;
        pitch.remote_center = 0.0; // 【调试】pitch中位

        pitch.position_pid.kp = 400.0;
        pitch.position_pid.ki = 80.0;
        pitch.position_pid.kd = 2.0;
        pitch.position_pid.max_output = 300.0;
        pitch.position_pid.integral_limit = 80.0;

        pitch.speed_pid.kp = 150.0;
        pitch.speed_pid.ki = 50.0;
        pitch.speed_pid.kd = 0.0;
        pitch.speed_pid.max_output = 20000.0;
        pitch.speed_pid.integral_limit = 400.0;


        Self {
            state: State::Waiting,
            yaw,
            pitch,
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            last_right_switch: 0,
            last_left_switch: 0,
            yaw_feedback_age: 9999,
            pitch_feedback_age: 9999,
            prepare_timeout_ticks: 0,
            prepare_settle_ticks: 0,
        }
    }
}

impl Control {
    /// Clear both motors' current commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input forces `Waiting`.
    pub fn tick(&mut self, can: &mut Can1) {
        self.yaw_feedback_age = self.yaw_feedback_age.saturating_add(1);
        self.pitch_feedback_age = self.pitch_feedback_age.saturating_add(1);
        
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();

        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }

        self.update();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the motor commands and return to `Waiting`.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            if self.state != State::Error {
                self.state = State::Waiting;
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    // TODO: check the ID and length, pass the data to the matching axis's
                    // `Motor::read`, and record its feedback age.
                    let frame = envelope.frame;
                    
                    let id = match frame.id() {
                        embedded_can::Id::Standard(std_id) => std_id.as_raw(),
                        _ => continue,
                    };
                    
                    let data = frame.data();
                    if data.len() < 8 {
                        continue;
                    }

                    let mut buf = [0u8; 8];
                    buf.copy_from_slice(&data[..8]);

                    match id {
                    //【调试】yaw id
                        0x205 => {
                            self.yaw.motor.read(&buf);
                            self.yaw_feedback_age = 0;
                        }
                    //【调试】pitch id
                        0x206 => {
                            self.pitch.motor.read(&buf);
                            self.pitch_feedback_age = 0;
                        }
                        _ => {}
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    pub fn update_mode(&mut self) {
        //读拨杆
        let right_sw = self.remote.as_ref().map_or(0, |r| r.right_switch);
        let left_sw = self.remote.as_ref().map_or(0, |r| r.left_switch);
        
        //如果状态新鲜
        let feedback_ok = self.yaw_feedback_age < 50 && self.pitch_feedback_age < 50;

        match self.state {
            State::Waiting => {
                // enable后进waiting，启动动作（先下拨2，再中拨3），回正以及重置
                if feedback_ok && self.last_right_switch == 2 && right_sw == 3 {
                    //目标角度设置为物理零位
                    self.yaw.target_angle_rad = self.yaw.reference_angle_rad;
                    self.pitch.target_angle_rad = self.pitch.reference_angle_rad;                
                    //全部重置
                    self.yaw.position_pid.reset();
                    self.yaw.speed_pid.reset();
                    self.pitch.position_pid.reset();
                    self.pitch.speed_pid.reset();
                    self.prepare_timeout_ticks = 0;
                    self.prepare_settle_ticks = 0;
                    //进prepare
                    self.state = State::Prepare;
                    rtt_target::rprintln!("State -> Prepare (Recentering)");
                }
            }

            State::Prepare => {
                //如果右拨杆下拨，回到waiting
                if right_sw == 2 {
                    self.state = State::Waiting;
                    rtt_target::rprintln!("State -> Waiting (Disabled in Prepare)");
                } else if self.check_faults() { //堵转或者高频振荡进error
                    self.state = State::Error;
                } else { //右拨杆在3，没有故障，开始计时
                    self.prepare_timeout_ticks += 1;
                    if self.prepare_timeout_ticks > 5000 { // 3秒超时进error
                        self.state = State::Error;
                        rtt_target::rprintln!("Error: Prepare timeout!");
                    } else {
                        // 【调试】判断两轴是否稳定 (角度误差 < 0.05 rad，速度 < 10 rpm)
                        let yaw_err = Self::wrap_angle(self.yaw.target_angle_rad - self.yaw.motor.angle_rad).abs();
                        let pitch_err = Self::wrap_angle(self.pitch.target_angle_rad - self.pitch.motor.angle_rad).abs();
                        let yaw_stable = yaw_err < 0.1 && self.yaw.motor.speed_rpm.abs() < 10.0;
                        let pitch_stable = pitch_err < 0.1 && self.pitch.motor.speed_rpm.abs() < 10.0;
                            
                        if yaw_stable && pitch_stable { //两个都稳定
                            self.prepare_settle_ticks += 1;
                            if self.prepare_settle_ticks > 200 { // 稳定 200ms
                                self.state = State::Running; //进running
                                rtt_target::rprintln!("State -> Running");
                            }
                        } else {
                            self.prepare_settle_ticks = 0; // 不稳定就重新计时
                        }
                    }
                }
            }

            State::Running => {
                if right_sw == 2 { // 右拨杆下拨，进waiting
                    self.state = State::Waiting;
                    rtt_target::rprintln!("State -> Waiting (Disabled in Running)");
                } else if self.check_faults() { //堵转或者高频振荡进error
                    self.state = State::Error;
                } else if !feedback_ok { //反馈丢失，进waiting
                    self.state = State::Waiting;
                    rtt_target::rprintln!("State -> Waiting (Feedback lost)");
                }
            }

            State::Error => {
                // 恢复动作：左拨杆先下拨(2)再上拨(1)
                if self.last_left_switch == 2 && left_sw == 3 {
                    // 恢复动作触发后，重置目标角度和速度环目标
                    self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
                    self.pitch.target_angle_rad = self.pitch.motor.angle_rad;
                    self.yaw.target_rpm = 0.0;
                    self.pitch.target_rpm = 0.0;
                    
                    self.yaw.position_pid.reset();
                    self.yaw.speed_pid.reset();
                    self.pitch.position_pid.reset();
                    self.pitch.speed_pid.reset();
                    self.yaw.stall_ticks = 0;
                    self.pitch.stall_ticks = 0;
                    self.yaw.oscillation_periods = 0;
                    self.pitch.oscillation_periods = 0;
                    
                    self.state = State::Waiting;
                    rtt_target::rprintln!("Manual Recovery -> Waiting (Please re-enable)");
                }
            }
        }

        // 记录上一拍状态，用于边沿检测
        self.last_right_switch = right_sw;
        self.last_left_switch = left_sw;
    }

    /// Set targets for the current state, run each axis's PIDs and write the current commands.
    pub fn read_remote_input(&mut self) {
        // 放在 Control 的方法中；先读取输入，不在这里直接写电压输出。
        if let Some(remote) = self.remote {
            // 【修复】一定要减去中值 1024！否则摇杆回中也会满速旋转
            let mut yaw_input = -(remote.channels[2] as f32 - - self.yaw.remote_center) / 660.0; //【调试】如果方向反了乘-1
            let mut pitch_input = -(remote.channels[3] as f32 - - self.pitch.remote_center) / 660.0; //【调试】如果方向反了乘-1
            
            yaw_input = yaw_input.clamp(-1.0, 1.0);
            pitch_input = pitch_input.clamp(-1.0, 1.0);

            //死区
            if yaw_input.abs() < self.yaw.rc_deadband { yaw_input = 0.0; }
            if pitch_input.abs() < self.pitch.rc_deadband { pitch_input = 0.0; }

            let dt = 0.001; // 1 ms tick
            
            self.yaw.target_angle_rad += yaw_input * self.yaw.max_target_rate_rad_s * dt; 
            self.pitch.target_angle_rad += pitch_input * self.pitch.max_target_rate_rad_s * dt; 
            
            // 限制pitch角度范围 rad
            let half = 0.25;  // ±0.25 rad ≈ ±14°，按实际行程调
            self.pitch.target_angle_rad = self.pitch.target_angle_rad.clamp(
            self.pitch.reference_angle_rad - half,
            self.pitch.reference_angle_rad + half,);
            
            //yaw 0~2π回绕
            let two_pi = 2.0 * core::f32::consts::PI;
            
            if self.yaw.target_angle_rad >= two_pi {
                self.yaw.target_angle_rad -= two_pi;
            } else if self.yaw.target_angle_rad < 0.0 {
                self.yaw.target_angle_rad += two_pi;
            }
        }



    }
    
    pub fn update(&mut self) {                
        if self.state == State::Waiting || self.state == State::Error {
            self.disable_all();
            return;
        }

        if self.state == State::Running {
            self.read_remote_input(); 
        } 

        
        // --- yaw 双环控制 ---
        // 1. 计算最短路径误差并进行回绕处理
        let yaw_err = Self::wrap_angle(self.yaw.target_angle_rad - self.yaw.motor.angle_rad);
        
        // 2. 位置环：输入 rad 误差，输出 rpm
        let yaw_rpm = self.yaw.position_pid.calculate_from_error(yaw_err, 0.001);
        self.yaw.target_rpm = yaw_rpm.clamp(-self.yaw.max_position_rpm, self.yaw.max_position_rpm);
        
        // 3. 速度环：输入 rpm，输出 电压原始值
        let yaw_voltage = self.yaw.speed_pid.calculate(self.yaw.target_rpm, self.yaw.motor.speed_rpm, 0.001);
        self.yaw.motor.command_voltage_raw = yaw_voltage; //【调试】电机方向

        // --- pitch 双环控制 ---

        let pitch_err = Self::wrap_angle(self.pitch.target_angle_rad - self.pitch.motor.angle_rad);
        
        let pitch_rpm = self.pitch.position_pid.calculate_from_error(pitch_err, 0.001);
        self.pitch.target_rpm = pitch_rpm.clamp(-self.pitch.max_position_rpm, self.pitch.max_position_rpm);
        
        let pitch_voltage = self.pitch.speed_pid.calculate(self.pitch.target_rpm, self.pitch.motor.speed_rpm, 0.001);
        self.pitch.motor.command_voltage_raw = pitch_voltage;//【调试】电机方向
    }

    /// 辅助函数：将角度误差限制在 -π 到 π 之间，解决过零点突变
    fn wrap_angle(mut angle: f32) -> f32 {
        let pi = core::f32::consts::PI;
        let two_pi = 2.0 * pi;
        angle = (angle + pi) % two_pi;
        if angle < 0.0 {
            angle += two_pi;
        }
        angle - pi
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];

        // TODO: clamp each voltage count output within the bench limit (protocol +/-25000),
        // encode signed i16 big-endian and write it into the
        // bytes for its motor ID as the manual describes. Other motor IDs stay zero.

        // yaw
        let yaw_raw = self.yaw.motor.command_voltage_raw.clamp(-MAX_VOLTAGE_RAW, MAX_VOLTAGE_RAW) as i16;
        let yaw_bytes = yaw_raw.to_be_bytes();
        data[0] = yaw_bytes[0];
        data[1] = yaw_bytes[1];

        // pitch
        let pitch_raw = self.pitch.motor.command_voltage_raw.clamp(-MAX_VOLTAGE_RAW, MAX_VOLTAGE_RAW) as i16;
        let pitch_bytes = pitch_raw.to_be_bytes();
        data[2] = pitch_bytes[0];
        data[3] = pitch_bytes[1];

        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// 检测堵转与高频振荡的入口
    fn check_faults(&mut self) -> bool {
        // 分别独立检查两轴，避免互相干扰
        if let Some(reason) = Self::check_axis_faults(&mut self.yaw) {
            rtt_target::rprintln!("Error: Yaw {} detected!", reason);
            return true;
        }
        if let Some(reason) = Self::check_axis_faults(&mut self.pitch) {
            rtt_target::rprintln!("Error: Pitch {} detected!", reason);
            return true;
        }
        false
    }

    /// 检查单个轴的状态，返回触发的原因（如果发生故障）
    fn check_axis_faults(axis: &mut Axis) -> Option<&'static str> {
        const STALL_TARGET_RPM: f32 = 50.0;
        const STALL_OUTPUT: f32 = 2000.0;
        const STALL_ACTUAL_RPM: f32 = 10.0;
        const STALL_TIMEOUT_TICKS: u16 = 500;

        const OSC_WINDOW_TICKS: u16 = 200;
        const OSC_MIN_AMPLITUDE_RAD: f32 = 0.1;
        const OSC_MAX_FREQ_HZ: f32 = 30.0;

        // 1. 疑似堵转检测 (500ms)
        let is_stalling = axis.target_rpm.abs() > STALL_TARGET_RPM 
            && axis.motor.command_voltage_raw.abs() > STALL_OUTPUT 
            && axis.motor.speed_rpm.abs() < STALL_ACTUAL_RPM;

        if is_stalling {
            axis.stall_ticks = axis.stall_ticks.saturating_add(1);
        } else {
            axis.stall_ticks = 0; // 只要条件不满足，立刻清零重新计时
        }

        if axis.stall_ticks > STALL_TIMEOUT_TICKS {
            return Some("Stall");
        }

        // 2. 持续高频振荡检测 (200ms 窗口)
        axis.oscillation_ticks += 1;
        
        let err = Control::wrap_angle(axis.target_angle_rad - axis.motor.angle_rad);
        
        // 振幅需大于 0.1 rad 的噪声容限才认为是“振荡”而非“稳定在目标附近”
        let current_sign = if err.abs() > OSC_MIN_AMPLITUDE_RAD {
            if err > 0.0 { 1 } else { -1 }
        } else {
            0
        };

        if current_sign != 0 {
            if axis.last_oscillation_sign != 0 && current_sign != axis.last_oscillation_sign {
                axis.oscillation_periods += 1;
            }
            axis.last_oscillation_sign = current_sign;
        } else {
            axis.last_oscillation_sign = 0;
        }

        if axis.oscillation_ticks >= OSC_WINDOW_TICKS {
            let full_periods = axis.oscillation_periods as f32 / 2.0;
            let window_sec = OSC_WINDOW_TICKS as f32 / 1000.0;
            let freq_hz = full_periods / window_sec;
            
            // 无论是否触发，结算后都要清零窗口
            axis.oscillation_periods = 0;
            axis.oscillation_ticks = 0;
            
            if freq_hz > OSC_MAX_FREQ_HZ {
                return Some("Oscillation");
            }
        }

        None // 没有故障
    }
}

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    
    // 【1】 循环外声明限频计数器
    let mut log_ticks = 0u16; 

    loop {
        ticker.next().await;
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        
        control.tick(&mut can);
        watchdog.pet();

        // 【2】 限频 RTT 日志
        log_ticks += 1;
        if log_ticks >= 200 { // 200ms 打印一次
            log_ticks = 0;
           
            let ch2 = control.remote.map_or(0, |r| r.channels[2]);
            let ch3 = control.remote.map_or(0, |r| r.channels[3]);
           
            // 打印
            rtt_target::rprintln!(
                "state={:?} ch2={} ch3={} |yaw: angle={:.3} target={:.3} rpm={:.1} target_rpm={:.1} out={:.1} | int_p={:.1} int_s={:.1} stall={} osc_period={}",
                control.state,
                ch2,
                ch3,
                control.yaw.motor.angle_rad,
                control.yaw.target_angle_rad,
                control.yaw.motor.speed_rpm,
                control.yaw.target_rpm,
                control.yaw.motor.command_voltage_raw,
                
                // 【3】 任务书要求加的：PID 内部量、堵转计时、振荡周期
                control.yaw.position_pid.integral,
                control.yaw.speed_pid.integral,
                control.yaw.stall_ticks,
                control.yaw.oscillation_periods
            );
            rtt_target::rprintln!(
                "state={:?} ch2={} ch3={} |pitch: angle={:.3} target={:.3} rpm={:.1} target_rpm={:.1} out={:.1} | int_p={:.1} int_s={:.1} stall={} osc_period={}",
                control.state,
                ch2,
                ch3,
                control.pitch.motor.angle_rad,
                control.pitch.target_angle_rad,
                control.pitch.motor.speed_rpm,
                control.pitch.target_rpm,
                control.pitch.motor.command_voltage_raw,
                control.pitch.position_pid.integral,
                control.pitch.speed_pid.integral,
                control.pitch.stall_ticks,
                control.pitch.oscillation_periods
            );
        }
    }
}