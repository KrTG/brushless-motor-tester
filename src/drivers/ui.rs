use core::fmt::Write;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10, ascii::FONT_8X13, ascii::FONT_10X20},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle, Styled},
    text::Text,
};
use heapless::String;
use rtt_target::rprintln;
use ssd1306::{Ssd1306, mode::BufferedGraphicsMode, prelude::*};
use stm32f7xx_hal::pac::DWT;

#[derive(PartialEq, Clone, Copy)]
pub enum DisplayedUi {
    None,
    Loading,
    Options,
    SensorReadings,
    Offline,
    Throttle,
    Settings,
}

#[derive(PartialEq, Clone, Copy)]
pub enum Setpoint {
    Throttle,
    Thrust,
    Current,
    EngineRPM,
    NoiseDB,
}

#[derive(PartialEq, Clone, Copy)]
pub enum ForceUnit {
    Gram,
    Newton,
}

struct OpDrawText {
    text: String<32>,
    fontsize: u8,
    position: Point,
}

enum Op {
    DrawText(OpDrawText),
    Flush,
    None,
}
const EMPTY_OP: Op = Op::None;

impl core::fmt::Display for Setpoint {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Setpoint::Throttle => write!(f, "Thrtl"),
            Setpoint::Thrust => write!(f, "Force"),
            Setpoint::Current => write!(f, "Crrnt"),
            Setpoint::EngineRPM => write!(f, "RPM"),
            Setpoint::NoiseDB => write!(f, "Noise"),
        }
    }
}

pub struct Ui<DI, SIZE>
where
    DI: WriteOnlyDataCommand,
    SIZE: DisplaySize,
{
    display: Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
    box_pos: Point,
    box_vel: Point,
    selected_option_menu: u8,
    selected_option_test: u8,
    pub setpoint: Setpoint,
    pub throttle_setpoint: f32,
    pub thrust_setpoint: f32,
    pub current_setpoint: f32,
    pub timer_sec: f32,
    pub throttle_limit: f32,
    pub min_voltage: f32,
    pub force_unit: ForceUnit,
    pub displayed_ui: DisplayedUi,
    last_fingerprint: u64,
    dirty: bool,
    buffer: [Op; 16],
}

impl<DI, SIZE> Ui<DI, SIZE>
where
    DI: WriteOnlyDataCommand,
    SIZE: DisplaySize,
{
    pub fn new(display: Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>) -> Self {
        Self {
            display,
            box_pos: Point::new(10, 10),
            box_vel: Point::new(8, 4),
            selected_option_menu: 0,
            selected_option_test: 0,
            setpoint: Setpoint::Throttle,
            throttle_setpoint: 5.0,
            thrust_setpoint: 0.1,
            current_setpoint: 0.4,
            timer_sec: 0.0,
            throttle_limit: 50.0,
            min_voltage: 3.6,
            force_unit: ForceUnit::Newton,
            displayed_ui: DisplayedUi::None,
            last_fingerprint: 0,
            dirty: true,
            buffer: [EMPTY_OP; 16],
        }
    }

    pub fn force_unit_factor(&self) -> f32 {
        match self.force_unit {
            ForceUnit::Gram => 1.0,
            ForceUnit::Newton => 0.00980665,
        }
    }

    pub fn init(&mut self) -> Result<(), ()> {
        self.display.init().map_err(|_| ())?;
        self.clear();
        self.flush()
    }

    pub fn render(
        &mut self,
        weight: f32,
        current: f32,
        voltage: f32,
        voltage_per_cell: f32,
        time_left: Option<f32>,
        throttle: f32,
        dwt: Option<&DWT>,
    ) -> bool {
        let v_comp = (voltage * 40.0) as u64;
        let i_comp = (current * 4.0) as u64;
        let thr_comp = throttle as u64;
        let weight_hash = (weight * 100.0) as u64;

        let starttime = dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0);
        let fingerprint = v_comp
            ^ i_comp.wrapping_shl(12)
            ^ thr_comp.wrapping_shl(24)
            ^ weight_hash.wrapping_shl(36);
        rprintln!(
            "Fingerprint time: {}",
            dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0) - starttime
        );

        if !self.dirty && fingerprint == self.last_fingerprint {
            return false;
        }
        self.dirty = false;
        self.last_fingerprint = fingerprint;

        let starttime = dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0);
        match self.displayed_ui {
            DisplayedUi::None => {}
            DisplayedUi::Loading => self.display_loading(),
            DisplayedUi::Options => self.display_options(voltage, voltage_per_cell),
            DisplayedUi::SensorReadings => self.display_sensor_readings(
                weight,
                current,
                voltage,
                voltage_per_cell,
                time_left,
                throttle,
                dwt,
            ),
            DisplayedUi::Offline => self.display_offline(),
            DisplayedUi::Throttle => self.display_throttle(throttle, voltage, voltage_per_cell),
            DisplayedUi::Settings => self.display_settings(),
        }
        rprintln!(
            "Time 3: {}",
            dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0) - starttime
        );

        true
    }

    pub fn down(&mut self) {
        match self.displayed_ui {
            DisplayedUi::Options => self.selected_option_menu = (self.selected_option_menu + 1) % 4,
            DisplayedUi::SensorReadings => {
                self.selected_option_test = (self.selected_option_test + 1) % 3
            }
            DisplayedUi::Settings => {
                self.selected_option_menu = (self.selected_option_menu + 1) % 3
            }
            _ => {}
        }
        self.dirty = true;
    }

    pub fn up(&mut self) {
        match self.displayed_ui {
            DisplayedUi::Options => self.selected_option_menu = (self.selected_option_menu + 1) % 4,
            DisplayedUi::SensorReadings => {
                self.selected_option_test = (self.selected_option_test + 1) % 3;
            }
            DisplayedUi::Settings => {
                self.selected_option_menu = (self.selected_option_menu + 1) % 3
            }
            _ => {}
        }
        self.dirty = true;
    }

    fn handle_settings_left(&mut self) {
        if self.selected_option_menu == 0 {
            self.throttle_limit = (self.throttle_limit - 5.0).max(10.0);
        } else if self.selected_option_menu == 1 {
            self.min_voltage = (self.min_voltage - 0.1).max(3.3);
        }
    }

    pub fn left(&mut self) {
        match self.displayed_ui {
            DisplayedUi::Options => self.handle_options_left(),
            DisplayedUi::Settings => self.handle_settings_left(),
            _ => {}
        }
        self.dirty = true;
    }

    fn handle_options_left(&mut self) {
        if self.selected_option_menu == 0 {
            self.setpoint = match self.setpoint {
                Setpoint::Throttle => Setpoint::Current,
                Setpoint::Thrust => Setpoint::Throttle,
                Setpoint::Current => Setpoint::Thrust,
                _ => Setpoint::Throttle,
            };
        } else if self.selected_option_menu == 1 {
            if self.setpoint == Setpoint::Throttle {
                self.throttle_setpoint = (self.throttle_setpoint - 1.0).max(5.0);
            } else if self.setpoint == Setpoint::Thrust {
                if self.thrust_setpoint < 10.0 {
                    self.thrust_setpoint = (self.thrust_setpoint - 0.1).max(0.1);
                } else {
                    self.thrust_setpoint = (self.thrust_setpoint - 1.0).max(9.9);
                }
            } else if self.setpoint == Setpoint::Current {
                if self.current_setpoint < 5.0 {
                    self.current_setpoint = (self.current_setpoint - 0.2).max(0.4);
                } else {
                    self.current_setpoint = (self.current_setpoint - 1.0).max(4.8);
                }
            }
        } else {
            self.timer_sec = (self.timer_sec - 1.0).max(0.0);
        }
    }

    fn handle_settings_right(&mut self) {
        if self.selected_option_menu == 0 {
            self.throttle_limit = (self.throttle_limit + 5.0).min(100.0);
        } else if self.selected_option_menu == 1 {
            self.min_voltage = (self.min_voltage + 0.1).min(4.2);
        } else {
            // Exit back to main options
            self.displayed_ui = DisplayedUi::Options;
            self.selected_option_menu = 3; // Highlight Settings entry again
        }
    }

    pub fn right(&mut self) {
        match self.displayed_ui {
            DisplayedUi::Options => self.handle_options_right(),
            DisplayedUi::Settings => self.handle_settings_right(),
            _ => {}
        }
        self.dirty = true;
    }

    fn handle_options_right(&mut self) {
        if self.selected_option_menu == 0 {
            self.setpoint = match self.setpoint {
                Setpoint::Throttle => Setpoint::Thrust,
                Setpoint::Thrust => Setpoint::Current,
                Setpoint::Current => Setpoint::Throttle,
                _ => Setpoint::Throttle,
            };
        } else if self.selected_option_menu == 1 {
            if self.setpoint == Setpoint::Throttle {
                self.throttle_setpoint = (self.throttle_setpoint + 1.0).min(self.throttle_limit);
            } else if self.setpoint == Setpoint::Thrust {
                if self.thrust_setpoint < 10.0 {
                    self.thrust_setpoint = (self.thrust_setpoint + 0.1).min(10.0);
                } else {
                    self.thrust_setpoint = (self.thrust_setpoint + 1.0).min(180.0);
                }
            } else if self.setpoint == Setpoint::Current {
                if self.current_setpoint < 5.0 {
                    self.current_setpoint = (self.current_setpoint + 0.2).min(5.0);
                } else {
                    self.current_setpoint = (self.current_setpoint + 1.0).min(20.0);
                }
            }
        } else if self.selected_option_menu == 2 {
            self.timer_sec = (self.timer_sec + 1.0).min(60.0);
        } else {
            self.displayed_ui = DisplayedUi::Settings;
            self.selected_option_menu = 0;
        }
    }

    pub fn set_loading(&mut self) {
        self.displayed_ui = DisplayedUi::Loading;
        self.dirty = true;
    }

    pub fn set_offline(&mut self) {
        self.displayed_ui = DisplayedUi::Offline;
        self.dirty = true;
    }

    pub fn engine_off(&mut self) {
        if self.displayed_ui == DisplayedUi::Throttle
            || self.displayed_ui == DisplayedUi::SensorReadings
            || self.displayed_ui == DisplayedUi::Loading
            || self.displayed_ui == DisplayedUi::None
        {
            self.displayed_ui = DisplayedUi::Options;
            self.dirty = true;
        }
    }

    pub fn engine_transition(&mut self) {
        if self.displayed_ui != DisplayedUi::Throttle {
            self.displayed_ui = DisplayedUi::Throttle;
            self.dirty = true;
        }
    }

    pub fn engine_on(&mut self) {
        if self.displayed_ui != DisplayedUi::SensorReadings {
            self.displayed_ui = DisplayedUi::SensorReadings;
            self.dirty = true;
        }
    }

    fn clear(&mut self) {
        let _ = self.display.clear(BinaryColor::Off);
    }

    pub fn flush(&mut self) -> Result<(), ()> {
        self.display.flush().map_err(|_| ())
    }

    fn draw_border(&mut self) {
        let border = Rectangle::new(Point::new(0, 0), Size::new(127, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1));
        let _ = border.draw(&mut self.display);
    }

    fn draw_voltage(&mut self, voltage: f32, voltage_per_cell: f32) {
        let text_small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        let postfix = if voltage_per_cell < self.min_voltage {
            "!!"
        } else if voltage_per_cell < 3.7 {
            "!"
        } else {
            ""
        };

        let mut subtext_str = String::<32>::new();
        let _ = write!(subtext_str, "V: {:.2}{}", voltage, postfix);

        let _ = Text::new(&subtext_str, Point::new(4, 63 - 4), text_small).draw(&mut self.display);
    }

    fn get_setpoint(&self) -> f32 {
        match self.setpoint {
            Setpoint::Throttle => self.throttle_setpoint,
            Setpoint::Thrust => self.thrust_setpoint,
            Setpoint::Current => self.current_setpoint,
            Setpoint::EngineRPM => self.timer_sec,
            Setpoint::NoiseDB => self.timer_sec,
        }
    }

    fn unit(&self) -> &str {
        match self.setpoint {
            Setpoint::Throttle => "%",
            Setpoint::Thrust => "N",
            Setpoint::Current => "A",
            Setpoint::EngineRPM => "RPM",
            Setpoint::NoiseDB => "dB",
        }
    }

    fn precision(&self) -> u32 {
        match self.setpoint {
            Setpoint::Throttle => 0,
            Setpoint::Thrust => 1,
            Setpoint::Current => 1,
            Setpoint::EngineRPM => 0,
            Setpoint::NoiseDB => 0,
        }
    }

    fn display_sensor_readings(
        &mut self,
        weight: f32,
        current: f32,
        voltage: f32,
        voltage_per_cell: f32,
        time_left: Option<f32>,
        throttle: f32,
        dwt: Option<&DWT>,
    ) {
        let starttime = dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0);
        self.clear();
        rprintln!(
            "Clear time: {}",
            dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0) - starttime
        );
        let starttime = dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0);
        rprintln!(
            "Border time: {}",
            dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0) - starttime
        );

        let text_big = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let text_medium = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);
        let text_small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        let starttime = dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0);

        let mut display_str_force = String::<32>::new();
        let force_newtons = weight * self.force_unit_factor();
        let _ = write!(display_str_force, "F: {:.2}N", force_newtons);

        let mut display_str_current = String::<32>::new();
        let _ = display_str_current.push_str("I: ");
        let _ = write!(display_str_current, "{:.2}{}", current, "A");

        let mut display_str_throttle = String::<32>::new();
        let _ = display_str_throttle.push_str("Thr: ");
        let _ = write!(display_str_throttle, "{:.0}{}", throttle, "%");

        rprintln!(
            "String time: {}",
            dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0) - starttime
        );
        let starttime = dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0);

        if self.selected_option_test == 0 {
            let _ =
                Text::new(&display_str_force, Point::new(4, 15), text_big).draw(&mut self.display);
            let _ = Text::new(&display_str_current, Point::new(4, 15 + 12), text_medium)
                .draw(&mut self.display);
            let _ = Text::new(&display_str_throttle, Point::new(4, 15 + 24), text_medium)
                .draw(&mut self.display);
        } else if self.selected_option_test == 1 {
            let _ = Text::new(&display_str_current, Point::new(4, 15), text_big)
                .draw(&mut self.display);
            let _ = Text::new(&display_str_throttle, Point::new(4, 15 + 12), text_medium)
                .draw(&mut self.display);
            let _ = Text::new(&display_str_force, Point::new(4, 15 + 24), text_medium)
                .draw(&mut self.display);
        } else if self.selected_option_test == 2 {
            let _ = Text::new(&display_str_throttle, Point::new(4, 15), text_big)
                .draw(&mut self.display);
            let _ = Text::new(&display_str_force, Point::new(4, 15 + 12), text_medium)
                .draw(&mut self.display);
            let _ = Text::new(&display_str_current, Point::new(4, 15 + 24), text_medium)
                .draw(&mut self.display);
        }

        rprintln!(
            "Text time: {}",
            dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0) - starttime
        );
        let starttime = dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0);

        self.draw_voltage(voltage, voltage_per_cell);

        rprintln!(
            "Voltage time: {}",
            dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0) - starttime
        );
        let starttime = dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0);

        if let Some(time_left) = time_left {
            let mut display_str = String::<32>::new();
            let _ = write!(display_str, "Time: {:.0}s", time_left);
            let _ =
                Text::new(&display_str, Point::new(65, 63 - 4), text_small).draw(&mut self.display);
        }

        rprintln!(
            "Time time: {}",
            dwt.map(|dwt| dwt.cyccnt.read()).unwrap_or(0) - starttime
        );
    }

    fn display_options(&mut self, voltage: f32, voltage_per_cell: f32) {
        self.clear();

        let text_big = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);

        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 0 {
            let _ = write!(display_str, "< Set:{} >", self.setpoint);
        } else {
            let _ = write!(display_str, "  Set:{}  ", self.setpoint);
        }
        let _ = Text::new(&display_str, Point::new(4, 15), text_big).draw(&mut self.display);

        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 1 {
            if self.precision() == 0 {
                let _ = write!(
                    display_str,
                    "< {}:{:.0}{} >",
                    self.setpoint,
                    self.get_setpoint(),
                    self.unit()
                );
            } else {
                let _ = write!(
                    display_str,
                    "< {}:{:.1}{} >",
                    self.setpoint,
                    self.get_setpoint(),
                    self.unit()
                );
            }
        } else {
            if self.precision() == 0 {
                let _ = write!(
                    display_str,
                    "  {}:{:.0}{}  ",
                    self.setpoint,
                    self.get_setpoint(),
                    self.unit()
                );
            } else {
                let _ = write!(
                    display_str,
                    "  {}:{:.1}{}  ",
                    self.setpoint,
                    self.get_setpoint(),
                    self.unit()
                );
            }
        }
        let _ = Text::new(&display_str, Point::new(4, 15 + 12), text_big).draw(&mut self.display);

        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 2 {
            let _ = write!(display_str, "< Timer:{}s >", self.timer_sec);
        } else {
            let _ = write!(display_str, "  Timer:{}s  ", self.timer_sec);
        }
        let _ = Text::new(&display_str, Point::new(4, 15 + 24), text_big).draw(&mut self.display);

        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 3 {
            let _ = write!(display_str, "  Settings >");
        } else {
            let _ = write!(display_str, "  Settings  ");
        }
        let _ = Text::new(&display_str, Point::new(4, 15 + 36), text_big).draw(&mut self.display);

        self.draw_voltage(voltage, voltage_per_cell);
    }

    fn display_settings(&mut self) {
        self.clear();

        let text_big = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);

        // 1. Throttle Limit
        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 0 {
            let _ = write!(display_str, "< Thr lim:{}% >", self.throttle_limit);
        } else {
            let _ = write!(display_str, "  Thr lim:{}%  ", self.throttle_limit);
        }
        let _ = Text::new(&display_str, Point::new(4, 15), text_big).draw(&mut self.display);

        // 2. Minimum Voltage
        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 1 {
            let _ = write!(display_str, "< Min V:{:.1}V >", self.min_voltage);
        } else {
            let _ = write!(display_str, "  Min V:{:.1}V  ", self.min_voltage);
        }
        let _ = Text::new(&display_str, Point::new(4, 15 + 13), text_big).draw(&mut self.display);

        // 3. Exit
        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 2 {
            let _ = write!(display_str, "  Exit >");
        } else {
            let _ = write!(display_str, "  Exit  ");
        }
        let _ = Text::new(&display_str, Point::new(4, 15 + 26), text_big).draw(&mut self.display);
    }

    fn display_loading(&mut self) {
        self.clear();

        let box_size = 10;
        let bounds = self.display.bounding_box();
        let width = bounds.size.width as i32;
        let height = bounds.size.height as i32;

        // Update position
        self.box_pos += self.box_vel;

        // Bounce logic (keeping space for the border)
        if self.box_pos.x <= 1 || self.box_pos.x >= width - box_size - 1 {
            self.box_vel.x = -self.box_vel.x;
            self.box_pos.x += self.box_vel.x; // Prevent getting stuck
        }
        if self.box_pos.y <= 1 || self.box_pos.y >= height - box_size - 1 {
            self.box_vel.y = -self.box_vel.y;
            self.box_pos.y += self.box_vel.y;
        }

        // Draw the bouncing box
        let _ = Rectangle::new(self.box_pos, Size::new(box_size as u32, box_size as u32))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut self.display);

        self.dirty = true;
    }

    fn display_offline(&mut self) {
        self.clear();

        let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let _ = Text::new("Sensor Offline", Point::new(10, 30), text_style).draw(&mut self.display);
    }

    fn display_throttle(&mut self, throttle: f32, voltage: f32, voltage_per_cell: f32) {
        self.clear();

        let text_big = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

        let mut display_str = String::<32>::new();
        let _ = write!(display_str, "Thr:{:.0}%", throttle);

        let _ = Text::new(&display_str, Point::new(4, 32), text_big).draw(&mut self.display);
        self.draw_voltage(voltage, voltage_per_cell);
    }
}
