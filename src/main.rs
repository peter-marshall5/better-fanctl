use std::io::{Write, Seek, SeekFrom};
use signal_hook::{consts::SIGINT, iterator::Signals};
use std::{thread, time::Duration, process, env, fs, fs::File};

fn main() {
    let _args: Vec<String> = env::args().collect();

    static BASE_SPEED: f32 = 36.0;
    static BASE_SPEED_FALLOFF_AC: f32 = 0.2;
    static BASE_SPEED_FALLOFF_BAT: f32 = 0.4;

    static ERROR_GAIN: f32 = 2.0;
    static ACCUM_GAIN: f32 = 0.028;
    static DERIVATIVE_GAIN: f32 = 0.98;

    static ACCUM_MAX: f32 = 1000.0;
    static ACCUM_MIN: f32 = -500.0;

    static TEMP_SETPOINT: f32 = 78.0;
    static LOW_TEMP: f32 = 45.0;

    static MANUAL_ENABLE_ADDRESS: u16 = 21;
    static SPEED_CONTROL_ADDRESS: u16 = 25;
    static MAX_SPEED: u8 = 59;
    static MIN_SPEED: u8 = 0;
    static SPEED_DIFFERENCE: u8 = MAX_SPEED - MIN_SPEED;

    static LOOP_INTERVAL_MS: u64 = 200;

    static EC_PATH: &str = "/dev/ec";
    static CPU_TEMP_PATH: &str = "/sys/class/hwmon/hwmon3/temp1_input";
    static AC_PLUGGED_PATH: &str = "/sys/class/power_supply/ACAD/online";

    let interval = Duration::from_millis(LOOP_INTERVAL_MS);

    let mut speed;

    let mut accumulation: f32 = 0.0;
    let mut longterm_accumulation: f32 = 0.0;
    let mut last_pid_output: f32 = 0.0;

    let mut curr_tick: u8 = 0;

    let mut battery_mode: bool = false;


    fn pct_to_speed(val: u8) -> u8 {
        (val as u32 * SPEED_DIFFERENCE as u32 / 255 - MIN_SPEED as u32) as u8
    }

    fn enable_manual_control() {
        write_ec(MANUAL_ENABLE_ADDRESS, 1);
    }

    fn disable_manual_control() {
        write_ec(MANUAL_ENABLE_ADDRESS, 0);
    }

    fn read_temp() -> f32 {
        let raw: i32 = fs::read_to_string(CPU_TEMP_PATH).unwrap().trim().parse().unwrap();
        raw as f32 / 1000.0
    }

    fn write_speed(speed: u8) {
        write_ec(SPEED_CONTROL_ADDRESS, speed);
    }

    fn write_ec(addr: u16, val: u8) {
        let mut ec_file = File::create(EC_PATH).unwrap();

        ec_file.seek(SeekFrom::Start(addr as u64)).unwrap();
        ec_file.write(&[val]).unwrap();
    }

    let mut signals = Signals::new(&[SIGINT]).unwrap();

    thread::spawn(move || {
        for _sig in signals.forever() {
            // println!("Received signal {:?}", sig);
            disable_manual_control();
            process::exit(0);
        }
    });

    enable_manual_control();

    loop {

        // Read the current CPU core temperature
        let temp = read_temp();

        let error = temp - TEMP_SETPOINT;

        accumulation = accumulation + error;
        if accumulation > ACCUM_MAX {
            accumulation = ACCUM_MAX;
        } else if accumulation < ACCUM_MIN {
            accumulation = ACCUM_MIN;
        }

        // Moving average of temperature
        // Tries to factor in the effects of the saturation of the cooling system
        longterm_accumulation = {
            if battery_mode {
                error
            } else {
                if error > longterm_accumulation {
                    error
                } else {
                    longterm_accumulation * 0.99 + error * 0.01
                }
            }
        };

        let pi_output = {
            if temp > TEMP_SETPOINT {
                error * ERROR_GAIN + accumulation * ACCUM_GAIN
            } else {
                0.0
            }
        };

        let derivative = last_pid_output - pi_output;

        let speed_offset = {
            if temp <= LOW_TEMP && battery_mode {
                0.00
            } else if temp <= TEMP_SETPOINT {
                if battery_mode {
                    BASE_SPEED + (longterm_accumulation * BASE_SPEED_FALLOFF_BAT)
                } else {
                    BASE_SPEED + (longterm_accumulation * BASE_SPEED_FALLOFF_AC)
                }
            } else {
                BASE_SPEED
            }
        };

        let pid_output = pi_output + derivative * DERIVATIVE_GAIN;
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
        // println!("S:{speed} T:{temp} E:{error} A:{accumulation} D:{derivative} OF:{speed_offset} O1:{pi_output} O2:{pid_output} L:{longterm_accumulation}");

        write_speed(speed);

        curr_tick += 1;

        if curr_tick > 20 {
            enable_manual_control();
            battery_mode = fs::read_to_string(AC_PLUGGED_PATH).unwrap().trim() == "0";
            curr_tick = 0;
        }

        thread::sleep(interval);
    }
}
