use core::fmt::Write;
use embedded_graphics::{
    mono_font::{
        MonoTextStyle, ascii::FONT_6X10, ascii::FONT_8X13, ascii::FONT_9X18, ascii::FONT_10X20,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::String;
use ssd1306::{Ssd1306, mode::BufferedGraphicsMode, prelude::*};

#[derive(PartialEq, Clone, Copy)]
pub enum DisplayedUi {
    None,
    Loading,
    Options,
    SensorReadings,
    Offline,
    Throttle,
}

#[derive(PartialEq, Clone, Copy)]
pub enum Setpoint {
    Throttle,
    Thrust,
    Current,
    EngineRPM,
    NoiseDB,
}

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
    pub throttle_setpoint: f32,
    pub thrust_setpoint: f32,
    pub current_setpoint: f32,
    pub timer_sec: f32,
    pub setpoint: Setpoint,
    displayed_ui: DisplayedUi,
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
            throttle_setpoint: 10.0,
            thrust_setpoint: 5.0,
            current_setpoint: 0.3,
            setpoint: Setpoint::Throttle,
            timer_sec: 0.0,
            displayed_ui: DisplayedUi::None,
        }
    }

    pub fn init(&mut self) -> Result<(), ()> {
        self.display.init().map_err(|_| ())?;
        self.clear();
        self.flush()
    }

    fn clear(&mut self) {
        let _ = self.display.clear(BinaryColor::Off);
    }

    fn flush(&mut self) -> Result<(), ()> {
        self.display.flush().map_err(|_| ())
    }

    fn draw_border(&mut self) {
        let _ = Rectangle::new(Point::new(0, 0), Size::new(127, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut self.display);
    }

    fn draw_voltage(&mut self, voltage: f32, voltage_per_cell: f32) {
        let text_small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        let postfix = if voltage_per_cell < 3.5 {
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
            Setpoint::Thrust => 0,
            Setpoint::Current => 1,
            Setpoint::EngineRPM => 0,
            Setpoint::NoiseDB => 0,
        }
    }

    pub fn display_sensor_readings(
        &mut self,
        weight_str: &str,
        current: f32,
        voltage: f32,
        voltage_per_cell: f32,
        time_left: Option<f32>,
    ) {
        self.displayed_ui = DisplayedUi::SensorReadings;
        self.clear();
        self.draw_border();

        let text_big = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let text_medium = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);
        let text_small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        let mut display_str_force = String::<32>::new();
        let _ = display_str_force.push_str("F: ");
        let _ = display_str_force.push_str(weight_str);
        let _ = display_str_force.push_str("N");

        let mut display_str_current = String::<32>::new();
        let _ = display_str_current.push_str("I: ");
        let _ = write!(display_str_current, "{:.2}{}", current, "A");

        if self.selected_option_test == 0 {
            let _ =
                Text::new(&display_str_force, Point::new(4, 16), text_big).draw(&mut self.display);
            let _ = Text::new(&display_str_current, Point::new(4, 16 + 12), text_medium)
                .draw(&mut self.display);
        } else {
            let _ = Text::new(&display_str_current, Point::new(4, 16), text_big)
                .draw(&mut self.display);
            let _ = Text::new(&display_str_force, Point::new(4, 16 + 12), text_medium)
                .draw(&mut self.display);
        }

        self.draw_voltage(voltage, voltage_per_cell);

        if let Some(time_left) = time_left {
            let mut display_str = String::<32>::new();
            let _ = write!(display_str, "Time: {:.0}s", time_left);
            let _ =
                Text::new(&display_str, Point::new(65, 63 - 4), text_small).draw(&mut self.display);
        }

        let _ = self.flush();
    }

    pub fn display_options(&mut self, voltage: f32, voltage_per_cell: f32) {
        self.displayed_ui = DisplayedUi::Options;
        self.clear();
        self.draw_border();

        let text_big = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);

        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 0 {
            let _ = write!(display_str, "< Set:{} >", self.setpoint);
        } else {
            let _ = write!(display_str, "  Set:{}  ", self.setpoint);
        }
        let _ = Text::new(&display_str, Point::new(4, 16), text_big).draw(&mut self.display);

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
        let _ = Text::new(&display_str, Point::new(4, 16 + 12), text_big).draw(&mut self.display);

        let mut display_str = String::<32>::new();
        if self.selected_option_menu == 2 {
            let _ = write!(display_str, "< Timer:{}s >", self.timer_sec);
        } else {
            let _ = write!(display_str, "  Timer:{}s  ", self.timer_sec);
        }
        let _ = Text::new(&display_str, Point::new(4, 16 + 24), text_big).draw(&mut self.display);

        self.draw_voltage(voltage, voltage_per_cell);

        let _ = self.flush();
    }

    pub fn display_loading(&mut self) {
        self.displayed_ui = DisplayedUi::Loading;
        self.clear();
        self.draw_border();

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

        let _ = self.flush();
    }

    pub fn display_offline(&mut self) {
        self.displayed_ui = DisplayedUi::Offline;
        self.clear();
        self.draw_border();

        let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let _ = Text::new("Sensor Offline", Point::new(10, 30), text_style).draw(&mut self.display);

        let _ = self.flush();
    }

    pub fn display_throttle(&mut self, throttle: f32, voltage: f32, voltage_per_cell: f32) {
        self.displayed_ui = DisplayedUi::Throttle;
        self.clear();
        self.draw_border();

        let text_big = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

        let mut display_str = String::<32>::new();
        let _ = write!(display_str, "Thr:{:.0}%", throttle);

        let _ = Text::new(&display_str, Point::new(4, 32), text_big).draw(&mut self.display);
        self.draw_voltage(voltage, voltage_per_cell);

        let _ = self.flush();
    }

    pub fn down(&mut self) {
        match self.displayed_ui {
            DisplayedUi::Options => self.selected_option_menu = (self.selected_option_menu + 1) % 3,
            DisplayedUi::SensorReadings => {
                self.selected_option_test = (self.selected_option_test + 1) % 2
            }
            _ => {}
        }
    }

    pub fn up(&mut self) {
        match self.displayed_ui {
            DisplayedUi::Options => self.selected_option_menu = (self.selected_option_menu + 2) % 3,
            DisplayedUi::SensorReadings => {
                self.selected_option_test = (self.selected_option_test + 1) % 2
            }
            _ => {}
        }
    }

    pub fn left(&mut self) {
        if self.displayed_ui != DisplayedUi::Options {
            return;
        }

        if self.selected_option_menu == 0 {
            self.setpoint = match self.setpoint {
                Setpoint::Throttle => Setpoint::Current,
                Setpoint::Thrust => Setpoint::Throttle,
                Setpoint::Current => Setpoint::Thrust,
                _ => Setpoint::Throttle,
            };
        } else if self.selected_option_menu == 1 {
            if self.setpoint == Setpoint::Throttle {
                self.throttle_setpoint = (self.throttle_setpoint - 1.0).max(10.0);
            } else if self.setpoint == Setpoint::Thrust {
                if self.thrust_setpoint < 100.0 {
                    self.thrust_setpoint = (self.thrust_setpoint - 1.0).max(5.0);
                } else if self.thrust_setpoint < 1000.0 {
                    self.thrust_setpoint = (self.thrust_setpoint - 10.0).max(99.0);
                } else {
                    self.thrust_setpoint = (self.thrust_setpoint - 100.0).max(990.0);
                }
            } else if self.setpoint == Setpoint::Current {
                if self.current_setpoint < 5.0 {
                    self.current_setpoint = (self.current_setpoint - 0.1).max(0.3);
                } else {
                    self.current_setpoint = (self.current_setpoint - 1.0).max(4.9);
                }
            }
        } else {
            self.timer_sec = (self.timer_sec - 1.0).max(0.0);
        }
    }

    pub fn right(&mut self) {
        if self.displayed_ui != DisplayedUi::Options {
            return;
        }

        if self.selected_option_menu == 0 {
            self.setpoint = match self.setpoint {
                Setpoint::Throttle => Setpoint::Thrust,
                Setpoint::Thrust => Setpoint::Current,
                Setpoint::Current => Setpoint::Throttle,
                _ => Setpoint::Throttle,
            };
        } else if self.selected_option_menu == 1 {
            if self.setpoint == Setpoint::Throttle {
                self.throttle_setpoint = (self.throttle_setpoint + 1.0).min(100.0);
            } else if self.setpoint == Setpoint::Thrust {
                if self.thrust_setpoint < 100.0 {
                    self.thrust_setpoint = (self.thrust_setpoint + 1.0).min(100.0);
                } else if self.thrust_setpoint < 1000.0 {
                    self.thrust_setpoint = (self.thrust_setpoint + 10.0).min(1000.0);
                } else {
                    self.thrust_setpoint = (self.thrust_setpoint + 100.0).min(18000.0);
                }
            } else if self.setpoint == Setpoint::Current {
                if self.current_setpoint < 5.0 {
                    self.current_setpoint = (self.current_setpoint + 0.1).min(5.0);
                } else {
                    self.current_setpoint = (self.current_setpoint + 1.0).min(20.0);
                }
            }
        } else {
            self.timer_sec = (self.timer_sec + 1.0).min(60.0);
        }
    }
}
