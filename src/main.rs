use std::io::{Write, Seek, SeekFrom, Result as IoResult};
use signal_hook::{consts::SIGINT, iterator::Signals};
use std::{error::Error, thread, time::Duration, process, env, fs, cmp, fs::File};

fn main() {
    let args: Vec<String> = env::args().collect();

    let M: f32 = 0.2;
    let BASE_SPEED: f32 = 30.0;

    let P: f32 = 2.0;
    let I: f32 = 0.028;
    let D: f32 = 0.98;

    let A_MAX: f32 = 1000.0;
    let A_MIN: f32 = -500.0;

    let TEMP_SETPOINT: f32 = 78.0;
    let LOW_TEMP: f32 = 45.0;

    static MANUAL_CONTROL_ADDR: u16 = 21;
    static SPEED_CONTROL_ADDR: u16 = 25;
    static SPEED_MAX: u8 = 59;
    static SPEED_MIN: u8 = 0;
    static SPEED_DIFF: u8 = SPEED_MAX - SPEED_MIN;
    let LOW_SPEED: u8 = SPEED_DIFF / 2 + SPEED_MIN;

    let LOOP_INTERVAL: u64 = 200;
    let LOOP_DELTA: f32 = LOOP_INTERVAL as f32 / 1000.0;
    let LOOP_INTERVAL_DIVIDED: f32 = LOOP_INTERVAL as f32 / 1000.0;

    let SLOW_RAMP_RATE: u8 = 6;
    let RAMP_RATE: u8 = 1;

    static EC_FILE: &str = "/dev/ec";
    static CPU_TEMP_FILE: &str = "/sys/class/hwmon/hwmon3/temp1_input";

    fn pct_to_speed(val: u8) -> u8 {
        (val as u32 * SPEED_DIFF as u32 / 255 - SPEED_MIN as u32) as u8
    }

    fn enable_manual_control() {
        write_ec(MANUAL_CONTROL_ADDR, 1);
    }

    fn disable_manual_control() {
        write_ec(MANUAL_CONTROL_ADDR, 0);
    }

    fn read_temp() -> f32 {
        let raw: i32 = fs::read_to_string(CPU_TEMP_FILE).unwrap().trim().parse().unwrap();
        raw as f32 / 1000.0
    }

    fn write_speed(speed: u8) {
        write_ec(SPEED_CONTROL_ADDR, speed);
    }

    fn write_ec(addr: u16, val: u8) {
        let mut ec_file = File::create(EC_FILE).unwrap();

        ec_file.seek(SeekFrom::Start(addr as u64)).unwrap();
        ec_file.write(&[val]).unwrap();
    }

    let mut signals = Signals::new(&[SIGINT]).unwrap();

    thread::spawn(move || {
        for sig in signals.forever() {
            // println!("Received signal {:?}", sig);
            disable_manual_control();
            process::exit(0);
        }
    });

    let interval = Duration::from_millis(LOOP_INTERVAL);

    let mut speed = SPEED_MIN;

    let mut accumulation: f32 = 0.0;
    let mut last_pid_output: f32 = 0.0;

    let mut curr_tick: u8 = 0;

    enable_manual_control();

    speed = pct_to_speed(255);

    loop {

        // Read the current CPU core temperature
        let temp = read_temp();

        let error = temp - TEMP_SETPOINT;

        accumulation = accumulation + error;
        if accumulation > A_MAX {
            accumulation = A_MAX;
        } else if accumulation < A_MIN {
            accumulation = A_MIN;
        }


        let pi_output = {
            if temp > TEMP_SETPOINT {
                error * P + accumulation * I
            } else {
                0.0
            }
        };

        let derivative = last_pid_output - pi_output;

        let speed_offset = {
            if temp <= LOW_TEMP {
                0.00
            } else if temp <= TEMP_SETPOINT {
                BASE_SPEED + (error * M)
            } else {
                BASE_SPEED
            }
        };

        let pid_output = pi_output + derivative * D;
        last_pid_output = pid_output;
        let final_output = pid_output + speed_offset;

        speed = {
            if final_output >= 100.0 {
                255
            } else if final_output <= 0.0 {
                0
            } else {
                pct_to_speed((final_output * 255.0 / 100.0) as u8)
            }
        };

        // Output for tuning the PID gains
        // println!("T:{temp} E:{error} A:{accumulation} D:{derivative} OF:{speed_offset} O1:{pi_output} O2:{pid_output}");

        write_speed(speed);

        curr_tick += 1;

        if curr_tick > 20 {
            enable_manual_control();
            curr_tick = 0;
        }

        thread::sleep(interval);
    }
}
