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
const MOTOR_COMMAND_ID: u16 = 0x1FE;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);

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
    /// Current command in A, encoded by `Control::transmit`.
    pub command_current_a: f32,
}

impl Motor {
    /// Clear the current command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_current_a = 0.0;
    }

    pub fn read(&mut self, _data: &[u8; 8]) {
        // TODO: decode angle, speed and current as the manual describes.
        //angle
        let angle_raw =((_data[0] as u16) << 8) | (_data[1] as u16);
        self.angle_rad = (angle_raw as f32) * (2.0 * core::f32::consts::PI / 8192.0); //单圈rad
        //rotation speed
        let speed_raw = (((_data[2] as u16) << 8) | (_data[3] as u16)) as i16;
        self.speed_rpm = speed_raw as f32;
        //current
        self.current_raw = (((_data[4] as u16) << 8) | (_data[5] as u16)) as i16; //协议原始值
    }
}

/// PID gains, plus the integral and history kept between ticks.
// TODO: add gains, history, an integral limit and a symmetric output limit `maxout`.
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral: f32,
    pub previous_error: f32,
    pub integral_limit: f32,
    pub max_output: f32,
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
        }
    }
}

impl Pid {
    // TODO: clear the integral and history.
    pub fn reset(&mut self) { 
        self.integral = 0.0;
        self.previous_error = 0.0;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns A. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        // TODO: compute the PID with dt_s, and limit the integral and output.
        let error = target - measured;
        //limit integral
        self.integral += error * dt_s;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);

        //PID
        let p = self.kp * error;
        let i = self.ki * self.integral;
        let d = self.kd * (error - self.previous_error) / dt_s;
        self.previous_error = error;
        let pid_output = (p + i + d).clamp(-self.max_output, self.max_output);
        pid_output
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
    /// Speed loop: rpm in, A out.
    pub speed_pid: Pid,
}

/// State shared by every control tick.
// TODO: add feedback ages and a fault record.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            yaw: Axis::default(),
            pitch: Axis::default(),
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
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
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() {
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
            self.state = State::Waiting;
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
                        0x205 => self.yaw.motor.read(&buf),
                        0x206 => self.pitch.motor.read(&buf),
                        _ => {}
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    // TODO: the transitions, and initialising targets and PIDs on entering a state.
    pub fn update_mode(&mut self) {
        match self.state {
            State::Waiting => {
                // TODO: enter Prepare after valid feedback and a new enable request.
            }
            State::Prepare => {
                // TODO: enter Running once recentering settles; stop on a timeout or fault.
            }
            State::Running => {
                // TODO: handle disable, feedback timeouts and control faults.
            }
            State::Error => {
                // TODO: keep zero output until your manual recovery condition.
            }
        }
    }

    /// Set targets for the current state, run each axis's PIDs and write the current commands.
    // TODO: limited recentering in Prepare, remote cascade control in Running, and
    // `disable_all` in Waiting and Error. Each tick is 0.001 s.
    
    pub fn read_remote_input(&mut self) {
        // 放在 Control 的方法中；先读取输入，不在这里直接写电压输出。
        if let Some(remote) = self.remote {
            let yaw_input = remote.channels[2] as f32 / 660.0;
            let pitch_input = remote.channels[3] as f32 / 660.0;
            let left_switch = remote.left_switch;
            let right_switch = remote.right_switch;
            // 接下来由你实现死区、方向、目标更新和状态判断。
            
            ket y_in = yaw_input;
            

            
        }
    }
    
    pub fn update(&mut self) {                

        // yaw position loop and speed loop
        let yaw_rpm = self.yaw.position_pid.calculate(self.yaw.target_angle_rad, self.yaw.motor.angle_rad, 0.001);
        self.yaw.target_rpm = yaw_rpm;
        let yaw_current = self.yaw.speed_pid.calculate(self.yaw.target_rpm, self.yaw.motor.speed_rpm, 0.001);
        self.yaw.motor.command_current_a = yaw_current;

        // pitch position loop and speed loop
        let pitch_rpm = self.pitch.position_pid.calculate(self.pitch.target_angle_rad, self.pitch.motor.angle_rad, 0.001);
        self.pitch.target_rpm = pitch_rpm;
        let pitch_current = self.pitch.speed_pid.calculate(self.pitch.target_rpm, self.pitch.motor.speed_rpm, 0.001);
        self.pitch.motor.command_current_a = pitch_current;
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];
        // TODO: limit and convert each axis's `command_current_a`, and write it into the
        // bytes for its motor ID as the manual describes. Other motor IDs stay zero.

        const MAX_CURRENT_A: f32 = 3.0;

        //yaw
        let yaw_current = self.yaw.motor.command_current_a.clamp(-MAX_CURRENT_A, MAX_CURRENT_A);
        let yaw_current_raw = (yaw_current / MAX_CURRENT_A * 16384.0) as i16;
        let yaw_bytes = yaw_current_raw.to_be_bytes();
        data[0] = yaw_bytes[0];
        data[1] = yaw_bytes[1];

        //pitch
        let pitch_current = self.pitch.motor.command_current_a.clamp(-MAX_CURRENT_A, MAX_CURRENT_A);
        let pitch_current_raw = (pitch_current / MAX_CURRENT_A * 16384.0) as i16;
        let pitch_bytes = pitch_current_raw.to_be_bytes(); 
        data[2] = pitch_bytes[0];
        data[3] = pitch_bytes[1];

        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
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
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();
    }
}
