use core::fmt::Write;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10, ascii::FONT_10X20},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::String;
use ssd1306::{Ssd1306, mode::BufferedGraphicsMode, prelude::*};

pub struct Ui<DI, SIZE>
where
    DI: WriteOnlyDataCommand,
    SIZE: DisplaySize,
{
    display: Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
    box_pos: Point,
    box_vel: Point,
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

    pub fn display_force(&mut self, weight_str: &str, voltage: f32, voltage_per_cell: f32) {
        self.clear();
        self.draw_border();

        let text_big = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let text_small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        let mut display_str = String::<32>::new();
        let _ = display_str.push_str("F: ");
        let _ = display_str.push_str(weight_str);
        let _ = display_str.push_str("N");

        let postfix = if voltage_per_cell < 3.5 {
            "!!"
        } else if voltage_per_cell < 3.7 {
            "!"
        } else {
            ""
        };

        let mut subtext_str = String::<32>::new();
        let _ = write!(subtext_str, "V: {:.2}{}", voltage, postfix);

        let _ = Text::new(&display_str, Point::new(4, 32), text_big).draw(&mut self.display);
        let _ = Text::new(&subtext_str, Point::new(4, 63 - 4), text_small).draw(&mut self.display);

        let _ = self.flush();
    }

    pub fn display_thrust(&mut self, thrust: f32, voltage: f32, voltage_per_cell: f32) {
        self.clear();
        self.draw_border();

        let text_big = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let text_small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        let mut display_str = String::<32>::new();
        let _ = write!(display_str, "Thrust: {:.2}%", thrust);

        let postfix = if voltage_per_cell < 3.5 {
            "!!"
        } else if voltage_per_cell < 3.7 {
            "!"
        } else {
            ""
        };

        let mut subtext_str = String::<32>::new();
        let _ = write!(subtext_str, "V: {:.2}{}", voltage, postfix);

        let _ = Text::new(&display_str, Point::new(4, 32), text_big).draw(&mut self.display);
        let _ = Text::new(&subtext_str, Point::new(4, 63 - 4), text_small).draw(&mut self.display);

        let _ = self.flush();
    }

    pub fn display_loading(&mut self) {
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
        self.clear();
        self.draw_border();

        let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let _ = Text::new("Sensor Offline", Point::new(10, 30), text_style).draw(&mut self.display);

        let _ = self.flush();
    }
}
