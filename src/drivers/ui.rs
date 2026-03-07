use core::fmt::Write;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10, ascii::FONT_8X13, ascii::FONT_10X20},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::{Deque, String};
use ssd1306::{Ssd1306, mode::BufferedGraphicsMode, prelude::*};
use stm32f7xx_hal::pac::DWT;

const MAIN_MENU_COUNT: u8 = 4;
const SETTINGS_MENU_COUNT: u8 = 4;
const SENSOR_MODES_COUNT: u8 = 3;

#[derive(PartialEq, Clone, Copy)]
pub enum DisplayedUi {
    None,
    Loading,
    MainMenu,
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

impl ForceUnit {
    pub fn as_str(&self) -> &'static str {
        match self {
            ForceUnit::Gram => "g",
            ForceUnit::Newton => "N",
        }
    }
}

impl core::fmt::Display for ForceUnit {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ForceUnit::Gram => write!(f, "g"),
            ForceUnit::Newton => write!(f, "N"),
        }
    }
}

struct OpDrawText {
    text: String<32>,
    fontsize: u8,
    position: Point,
}

struct OpDrawRect {
    position: Point,
    size: Size,
    style: PrimitiveStyle<BinaryColor>,
}
enum Op {
    Clear,
    DrawText(OpDrawText),
    DrawRect(OpDrawRect),
    Flush,
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
    pub setpoint: Setpoint,
    pub throttle_setpoint: f32,
    pub thrust_setpoint: f32,
    pub current_setpoint: f32,
    pub timer_sec: f32,
    pub throttle_limit: f32,
    pub min_voltage: f32,
    pub force_unit: ForceUnit,
    pub displayed_ui: DisplayedUi,
    last_voltage: f32,
    dirty: bool,
    buffer: Deque<Op, 64>,
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
            thrust_setpoint: 0.05,
            current_setpoint: 0.4,
            timer_sec: 0.0,
            throttle_limit: 50.0,
            min_voltage: 3.6,
            force_unit: ForceUnit::Newton,
            displayed_ui: DisplayedUi::None,
            last_voltage: 0.0,
            dirty: true,
            buffer: Deque::new(),
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

    pub fn render_partial(&mut self) -> bool {
        if self.buffer.is_empty() {
            return false;
        }
        self.draw_call();
        true
    }

    pub fn render_full(&mut self) {
        while self.render_partial() {}
    }

    pub fn update(
        &mut self,
        weight: f32,
        current: f32,
        voltage: f32,
        voltage_per_cell: f32,
        time_left: Option<f32>,
        throttle: f32,
    ) -> bool {
        let voltage_changed = (voltage - self.last_voltage).abs() > 0.03;

        if !self.dirty && !voltage_changed {
            return false;
        }
        self.dirty = false;
        self.last_voltage = voltage;

        let _ = self.buffer.push_back(Op::Clear);
        match self.displayed_ui {
            DisplayedUi::None => {}
            DisplayedUi::Loading => self.display_loading(),
            DisplayedUi::MainMenu => self.display_main_menu(voltage, voltage_per_cell),
            DisplayedUi::SensorReadings => self.display_sensor_readings(
                weight,
                current,
                voltage,
                voltage_per_cell,
                time_left,
                throttle,
                None,
            ),
            DisplayedUi::Offline => self.display_offline(),
            DisplayedUi::Throttle => self.display_throttle(throttle, voltage, voltage_per_cell),
            DisplayedUi::Settings => self.display_settings(),
        }
        let _ = self.buffer.push_back(Op::Flush);

        true
    }

    pub fn down(&mut self) {
        match self.displayed_ui {
            DisplayedUi::MainMenu => {
                self.selected_option_menu = (self.selected_option_menu + 1) % MAIN_MENU_COUNT
            }
            DisplayedUi::SensorReadings => {
                self.selected_option_test = (self.selected_option_test + 1) % SENSOR_MODES_COUNT
            }
            DisplayedUi::Settings => {
                self.selected_option_menu = (self.selected_option_menu + 1) % SETTINGS_MENU_COUNT
            }
            _ => {}
        }
        self.dirty = true;
    }

    pub fn up(&mut self) {
        match self.displayed_ui {
            DisplayedUi::MainMenu => {
                self.selected_option_menu =
                    (self.selected_option_menu + MAIN_MENU_COUNT - 1) % MAIN_MENU_COUNT
            }
            DisplayedUi::SensorReadings => {
                self.selected_option_test =
                    (self.selected_option_test + SENSOR_MODES_COUNT - 1) % SENSOR_MODES_COUNT;
            }
            DisplayedUi::Settings => {
                self.selected_option_menu =
                    (self.selected_option_menu + SETTINGS_MENU_COUNT - 1) % SETTINGS_MENU_COUNT
            }
            _ => {}
        }
        self.dirty = true;
    }

    pub fn left(&mut self) {
        match self.displayed_ui {
            DisplayedUi::MainMenu => self.handle_main_menu_left(),
            DisplayedUi::Settings => self.handle_settings_left(),
            _ => {}
        }
        self.dirty = true;
    }

    pub fn right(&mut self) {
        match self.displayed_ui {
            DisplayedUi::MainMenu => self.handle_main_menu_right(),
            DisplayedUi::Settings => self.handle_settings_right(),
            _ => {}
        }
        self.dirty = true;
    }

    fn draw_call(&mut self) {
        if let Some(op) = self.buffer.pop_front() {
            match op {
                Op::DrawText(op) => {
                    let style = match op.fontsize {
                        6 => MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        8 => MonoTextStyle::new(&FONT_8X13, BinaryColor::On),
                        _ => MonoTextStyle::new(&FONT_10X20, BinaryColor::On),
                    };
                    let _ = Text::new(&op.text, op.position, style).draw(&mut self.display);
                }
                Op::DrawRect(op) => {
                    let _ = Rectangle::new(op.position, op.size)
                        .into_styled(op.style)
                        .draw(&mut self.display);
                }
                Op::Clear => self.clear(),
                Op::Flush => {
                    let _ = self.flush();
                }
            }
        }
    }

    fn handle_main_menu_left(&mut self) {
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
                match self.force_unit {
                    ForceUnit::Newton => {
                        if self.thrust_setpoint < 2.0 {
                            self.thrust_setpoint = (self.thrust_setpoint - 0.05).max(0.05);
                        } else {
                            self.thrust_setpoint = (self.thrust_setpoint - 0.5).max(1.95);
                        }
                    }
                    ForceUnit::Gram => {
                        if self.thrust_setpoint <= 200.0 {
                            self.thrust_setpoint = (self.thrust_setpoint - 5.0).max(5.0);
                        } else {
                            self.thrust_setpoint = (self.thrust_setpoint - 50.0).max(200.0);
                        }
                    }
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

    fn handle_main_menu_right(&mut self) {
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
                match self.force_unit {
                    ForceUnit::Newton => {
                        if self.thrust_setpoint < 2.0 {
                            self.thrust_setpoint = (self.thrust_setpoint + 0.05).min(2.0);
                        } else {
                            self.thrust_setpoint = (self.thrust_setpoint + 0.5).min(20.0);
                        }
                    }
                    ForceUnit::Gram => {
                        if self.thrust_setpoint < 200.0 {
                            self.thrust_setpoint = (self.thrust_setpoint + 5.0).min(200.0);
                        } else {
                            self.thrust_setpoint = (self.thrust_setpoint + 50.0).min(2000.0);
                        }
                    }
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

    fn handle_settings_left(&mut self) {
        if self.selected_option_menu == 0 {
            self.throttle_limit = (self.throttle_limit - 5.0).max(10.0);
        } else if self.selected_option_menu == 1 {
            self.min_voltage = (self.min_voltage - 0.1).max(3.3);
        } else if self.selected_option_menu == 2 {
            match self.force_unit {
                ForceUnit::Gram => {
                    self.force_unit = ForceUnit::Newton;
                    self.thrust_setpoint = 0.05;
                }
                ForceUnit::Newton => {
                    self.force_unit = ForceUnit::Gram;
                    self.thrust_setpoint = 5.0;
                }
            };
        }
    }

    fn handle_settings_right(&mut self) {
        if self.selected_option_menu == 0 {
            self.throttle_limit = (self.throttle_limit + 5.0).min(100.0);
        } else if self.selected_option_menu == 1 {
            self.min_voltage = (self.min_voltage + 0.1).min(4.2);
        } else if self.selected_option_menu == 2 {
            match self.force_unit {
                ForceUnit::Gram => {
                    self.force_unit = ForceUnit::Newton;
                    self.thrust_setpoint = 0.05;
                }
                ForceUnit::Newton => {
                    self.force_unit = ForceUnit::Gram;
                    self.thrust_setpoint = 5.0;
                }
            };
        } else {
            // Exit back to main menu
            self.displayed_ui = DisplayedUi::MainMenu;
            self.selected_option_menu = 3; // Highlight Settings entry again
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
            self.displayed_ui = DisplayedUi::MainMenu;
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
        let postfix = if voltage_per_cell < self.min_voltage {
            "!!"
        } else if voltage_per_cell < 3.7 {
            "!"
        } else {
            ""
        };

        let mut subtext_str = String::<32>::new();
        let _ = write!(subtext_str, "V: {:.2}{}", voltage, postfix);

        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: subtext_str,
            fontsize: 6,
            position: Point::new(4, 63 - 4),
        }));
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
            Setpoint::Thrust => self.force_unit.as_str(),
            Setpoint::Current => "A",
            Setpoint::EngineRPM => "RPM",
            Setpoint::NoiseDB => "dB",
        }
    }

    fn precision(&self) -> u32 {
        match self.setpoint {
            Setpoint::Throttle => 0,
            Setpoint::Thrust => match self.force_unit {
                ForceUnit::Gram => 0,
                ForceUnit::Newton => 2,
            },
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
        _dwt: Option<&DWT>,
    ) {
        let mut display_str_force = String::<32>::new();
        let force = weight * self.force_unit_factor();
        match self.force_unit {
            ForceUnit::Gram => {
                let _ = write!(display_str_force, "F: {:.0}{}", force, self.force_unit);
            }
            ForceUnit::Newton => {
                let _ = write!(display_str_force, "F: {:.2}{}", force, self.force_unit);
            }
        }

        let mut display_str_current = String::<32>::new();
        let _ = display_str_current.push_str("I: ");
        let _ = write!(display_str_current, "{:.2}{}", current, "A");

        let mut display_str_throttle = String::<32>::new();
        let _ = display_str_throttle.push_str("Thr: ");
        let _ = write!(display_str_throttle, "{:.0}{}", throttle, "%");

        if self.selected_option_test == 0 {
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_force,
                fontsize: 10,
                position: Point::new(4, 15),
            }));
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_current,
                fontsize: 8,
                position: Point::new(4, 15 + 12),
            }));
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_throttle,
                fontsize: 8,
                position: Point::new(4, 15 + 24),
            }));
        } else if self.selected_option_test == 1 {
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_current,
                fontsize: 10,
                position: Point::new(4, 15),
            }));
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_throttle,
                fontsize: 8,
                position: Point::new(4, 15 + 12),
            }));
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_force,
                fontsize: 8,
                position: Point::new(4, 15 + 24),
            }));
        } else if self.selected_option_test == 2 {
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_throttle,
                fontsize: 10,
                position: Point::new(4, 15),
            }));
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_force,
                fontsize: 8,
                position: Point::new(4, 15 + 12),
            }));
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str_current,
                fontsize: 8,
                position: Point::new(4, 15 + 24),
            }));
        }

        if let Some(time_left) = time_left {
            let mut display_str = String::<32>::new();
            let _ = write!(display_str, "Time: {:.0}s", time_left);
            let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
                text: display_str,
                fontsize: 8,
                position: Point::new(65, 63 - 4),
            }));
        }

        self.draw_voltage(voltage, voltage_per_cell);
        self.dirty = true;
    }

    fn display_main_menu(&mut self, voltage: f32, voltage_per_cell: f32) {
        let mut display_str_menu = String::<32>::new();
        if self.selected_option_menu == 0 {
            let _ = write!(display_str_menu, "< Set:{} >", self.setpoint);
        } else {
            let _ = write!(display_str_menu, "  Set:{}  ", self.setpoint);
        }

        let mut display_str_setpoint = String::<32>::new();
        if self.selected_option_menu == 1 {
            match self.precision() {
                0 => {
                    let _ = write!(
                        display_str_setpoint,
                        "< {}:{:.0}{} >",
                        self.setpoint,
                        self.get_setpoint(),
                        self.unit()
                    );
                }
                1 => {
                    let _ = write!(
                        display_str_setpoint,
                        "< {}:{:.1}{} >",
                        self.setpoint,
                        self.get_setpoint(),
                        self.unit()
                    );
                }
                _ => {
                    let _ = write!(
                        display_str_setpoint,
                        "< {}:{:.2}{} >",
                        self.setpoint,
                        self.get_setpoint(),
                        self.unit()
                    );
                }
            }
        } else {
            match self.precision() {
                0 => {
                    let _ = write!(
                        display_str_setpoint,
                        "  {}:{:.0}{}  ",
                        self.setpoint,
                        self.get_setpoint(),
                        self.unit()
                    );
                }
                1 => {
                    let _ = write!(
                        display_str_setpoint,
                        "  {}:{:.1}{}  ",
                        self.setpoint,
                        self.get_setpoint(),
                        self.unit()
                    );
                }
                _ => {
                    let _ = write!(
                        display_str_setpoint,
                        "  {}:{:.2}{}  ",
                        self.setpoint,
                        self.get_setpoint(),
                        self.unit()
                    );
                }
            }
        }

        let mut display_str_timer = String::<32>::new();
        if self.selected_option_menu == 2 {
            let _ = write!(display_str_timer, "< Timer:{}s >", self.timer_sec);
        } else {
            let _ = write!(display_str_timer, "  Timer:{}s  ", self.timer_sec);
        }

        let mut display_str_settings = String::<32>::new();
        if self.selected_option_menu == 3 {
            let _ = write!(display_str_settings, "  Settings >");
        } else {
            let _ = write!(display_str_settings, "  Settings  ");
        }

        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str_menu,
            fontsize: 8,
            position: Point::new(4, 15),
        }));

        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str_setpoint,
            fontsize: 8,
            position: Point::new(4, 15 + 12),
        }));

        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str_timer,
            fontsize: 8,
            position: Point::new(4, 15 + 24),
        }));

        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str_settings,
            fontsize: 8,
            position: Point::new(4, 15 + 36),
        }));

        self.draw_voltage(voltage, voltage_per_cell);
    }

    fn display_settings(&mut self) {
        let mut display_str_throttle_limit = String::<32>::new();
        if self.selected_option_menu == 0 {
            let _ = write!(
                display_str_throttle_limit,
                "< Thr lim:{}% >",
                self.throttle_limit
            );
        } else {
            let _ = write!(
                display_str_throttle_limit,
                "  Thr lim:{}%  ",
                self.throttle_limit
            );
        }

        // 2. Minimum Voltage
        let mut display_str_min_voltage = String::<32>::new();
        if self.selected_option_menu == 1 {
            let _ = write!(
                display_str_min_voltage,
                "< Min V:{:.1}V >",
                self.min_voltage
            );
        } else {
            let _ = write!(
                display_str_min_voltage,
                "  Min V:{:.1}V  ",
                self.min_voltage
            );
        }

        // 3. Force unit
        let mut display_str_force_unit = String::<32>::new();
        if self.selected_option_menu == 2 {
            let _ = write!(display_str_force_unit, "< F unit: {} >", self.force_unit);
        } else {
            let _ = write!(display_str_force_unit, "  F unit: {}  ", self.force_unit);
        }

        let mut display_str_exit = String::<32>::new();
        if self.selected_option_menu == 3 {
            let _ = write!(display_str_exit, "  Exit >");
        } else {
            let _ = write!(display_str_exit, "  Exit  ");
        }
        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str_throttle_limit,
            fontsize: 8,
            position: Point::new(4, 15),
        }));
        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str_min_voltage,
            fontsize: 8,
            position: Point::new(4, 15 + 12),
        }));
        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str_force_unit,
            fontsize: 8,
            position: Point::new(4, 15 + 24),
        }));
        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str_exit,
            fontsize: 8,
            position: Point::new(4, 15 + 36),
        }));
    }

    fn display_loading(&mut self) {
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
        let _ = self.buffer.push_back(Op::DrawRect(OpDrawRect {
            position: self.box_pos,
            size: Size::new(box_size as u32, box_size as u32),
            style: PrimitiveStyle::with_stroke(BinaryColor::On, 1),
        }));

        self.dirty = true;
    }

    fn display_offline(&mut self) {
        let mut display_str = String::<32>::new();
        let _ = write!(display_str, "Sensor Offline");

        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str,
            fontsize: 10,
            position: Point::new(4, 32),
        }));
    }

    fn display_throttle(&mut self, throttle: f32, voltage: f32, voltage_per_cell: f32) {
        let mut display_str = String::<32>::new();
        let _ = write!(display_str, "Thr:{:.0}%", throttle);

        let _ = self.buffer.push_back(Op::DrawText(OpDrawText {
            text: display_str,
            fontsize: 10,
            position: Point::new(4, 32),
        }));
        self.draw_voltage(voltage, voltage_per_cell);

        self.dirty = true;
    }
}
