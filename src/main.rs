use std::io::{Write, Seek, SeekFrom};
use signal_hook::{consts::SIGINT, iterator::Signals};
use std::{thread, time::Duration, env, fs, sync::Arc, sync::Mutex};


struct FanSpeedController {
    ec_path: String,
    manual_enable_address: u16,
    speed_control_address: u16,
    min_speed: u8,
    max_speed: u8,
}

impl FanSpeedController {
    fn enable_manual_control(&self) {
        self.write_ec(self.manual_enable_address, 1);
    }

    fn disable_manual_control(&self) {
        self.write_ec(self.manual_enable_address, 0);
    }

    fn write_speed(&self, speed: f32) {
        let raw_speed = ((speed / 100.0 * (self.max_speed - self.min_speed) as f32) as u32 - self.min_speed as u32) as u8;
        self.write_ec(self.speed_control_address, raw_speed);
    }

    fn write_ec(&self, addr: u16, val: u8) {
        let mut ec_file = fs::OpenOptions::new()
            .write(true)
            .create(false)
            .open(&self.ec_path)
            .unwrap();

        ec_file.seek(SeekFrom::Start(addr as u64)).unwrap();
        ec_file.write(&[val]).unwrap();
    }
}

impl Drop for FanSpeedController {
    fn drop(&mut self) {
        self.disable_manual_control()
    }
}


struct TemperatureSensor {
    hwmon_path: String,
}

impl TemperatureSensor {
    fn read_temp(&self) -> f32 {
        let raw: i32 = fs::read_to_string(&self.hwmon_path).unwrap().trim().parse().unwrap();
        raw as f32 / 1000.0
    }
}


struct PidController {
    error_gain: f32,
    accum_gain: f32,
    deriv_gain: f32,
    temp_setpoint: f32,
    low_temp: f32,
    accum_min: f32,
    accum_max: f32,
    base_speed: f32,
    base_speed_falloff_ac: f32,
    base_speed_falloff_bat: f32,
    accumulation: f32,
    longterm_accumulation: f32,
    last_pid_output: f32,
    battery_mode: bool,
}

impl PidController {
    fn tick(&mut self, temp: f32) -> f32 {
        let error = temp - self.temp_setpoint;

        self.accumulation = self.accumulation + error;
        if self.accumulation > self.accum_max {
            self.accumulation = self.accum_max;
        } else if self.accumulation < self.accum_min {
            self.accumulation = self.accum_min;
        }

        // Moving average of temperature
        // Tries to factor in the effects of the saturation of the cooling system
        self.longterm_accumulation = {
            if self.battery_mode {
                error
            } else {
                if error > self.longterm_accumulation {
                    error
                } else {
                    self.longterm_accumulation * 0.99 + error * 0.01
                }
            }
        };

        let pi_output = {
            if temp > self.temp_setpoint {
                error * self.error_gain + self.accumulation * self.accum_gain
            } else {
                0.0
            }
        };

        let derivative = self.last_pid_output - pi_output;

        let speed_offset = {
            if temp <= self.low_temp && self.battery_mode {
                0.00
            } else if temp <= self.temp_setpoint {
                if self.battery_mode {
                    self.base_speed + (self.longterm_accumulation * self.base_speed_falloff_bat)
                } else {
                    self.base_speed + (self.longterm_accumulation * self.base_speed_falloff_ac)
                }
            } else {
                self.base_speed
            }
        };

        let pid_output = pi_output + derivative * self.deriv_gain;
        self.last_pid_output = pid_output;

        pid_output + speed_offset
    }
}


fn main() {
    let args: Vec<String> = env::args().collect();

    static LOOP_INTERVAL_MS: u64 = 200;

    let interval = Duration::from_millis(LOOP_INTERVAL_MS);

    let mut curr_tick: u8 = 0;

    let main_fan = FanSpeedController {
        ec_path: args[2].clone(),
        manual_enable_address: 21,
        speed_control_address: 25,
        min_speed: 0,
        max_speed: 59,
    };

    let cpu_thermal = TemperatureSensor {
        hwmon_path: args[1].clone(),
    };

    let mut pid_controller = PidController {
        error_gain: 4.2,
        accum_gain: 0.018,
        deriv_gain: 0.995,
        temp_setpoint: 72.0,
        low_temp: 45.0,
        base_speed: 12.0,
        base_speed_falloff_ac: 0.2,
        base_speed_falloff_bat: 0.4,
        accum_min: 1000.0,
        accum_max: -500.0,
        accumulation: 0.0,
        longterm_accumulation: 0.0,
        last_pid_output: 0.0,
        battery_mode: false,
    };

    let ac_plugged_path: &str = &args[3].clone();

    let stop_signal = Arc::new(Mutex::new(false));
    let cloned_signal = Arc::clone(&stop_signal);

    let mut signals = Signals::new(&[SIGINT]).unwrap();

    thread::spawn(move || {
        for _sig in signals.forever() {
            let mut state = cloned_signal.lock().unwrap();
            *state = true;
        }
    });

    let _ = cpu_thermal.read_temp();
    main_fan.enable_manual_control();

    loop {
        if *stop_signal.lock().unwrap() {
            main_fan.disable_manual_control();
            break;
        }

        // Read the current CPU core temperature
        let temp = cpu_thermal.read_temp();

        let final_output = pid_controller.tick(temp);

        //println!("{}", final_output);

        let final_speed = {
            if final_output >= 100.0 {
                100.
            } else if final_output <= 0.0 {
                0.0
            } else {
                final_output
            }
        };

        main_fan.write_speed(final_speed); 

        curr_tick += 1;

        if curr_tick > 20 {
            main_fan.enable_manual_control();
            pid_controller.battery_mode = fs::read_to_string(ac_plugged_path).unwrap().trim() == "0";
            curr_tick = 0;
        }

        thread::sleep(interval);
    }
}
