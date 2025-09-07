use alloc::{rc::Rc, string::String};
use core::{
    cell::{RefCell, RefMut},
    fmt::Write,
};
use ds323x::{
    DateTimeAccess, Datelike, Ds323x, NaiveDateTime, Timelike, ic::DS3231, interface::I2cInterface,
};
use embedded_sdmmc::{TimeSource, Timestamp};

pub struct RtcTimeSource<I2C> {
    // QUESTION: Do I have to pack it in a RefCell to be able to borrow it in the get_timestamp implementation?
    // Is there a simpler way to do it. To my understanding the ownership isn't the issue, but the TimeSource trait
    // constraint is the limitation.
    rtc: RefCell<Ds323x<I2cInterface<I2C>, DS3231>>,
}

impl<I2C> RtcTimeSource<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c_bus: I2C) -> Self {
        Self {
            rtc: RefCell::new(Ds323x::new_ds3231(i2c_bus)),
        }
    }

    pub fn with_rtc<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut RefMut<Ds323x<I2cInterface<I2C>, DS3231>>) -> R,
    {
        let mut rtc = self.rtc.borrow_mut();
        f(&mut rtc)
    }

    pub fn get_datetime(
        &self,
    ) -> Result<NaiveDateTime, ds323x::Error<<I2C as embedded_hal::i2c::ErrorType>::Error>> {
        self.with_rtc(|rtc| rtc.datetime())
    }
}

// This Newtype wrapper is required impl TimeSource on an Rc instance. See below when instantiating rtc_timesource.
pub struct SharedRtcTimeSource<I2C>(pub Rc<RtcTimeSource<I2C>>);

impl<I2C> Clone for SharedRtcTimeSource<I2C> {
    fn clone(&self) -> Self {
        SharedRtcTimeSource(self.0.clone())
    }
}

impl<I2C> TimeSource for SharedRtcTimeSource<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    fn get_timestamp(&self) -> Timestamp {
        if let Ok(dt) = self.0.with_rtc(|rtc| rtc.datetime()) {
            Timestamp {
                year_since_1970: (dt.year() as i32 - 1970) as u8,
                zero_indexed_month: (dt.month() as u8) - 1,
                zero_indexed_day: (dt.day() as u8) - 1,
                hours: dt.hour() as u8,
                minutes: dt.minute() as u8,
                seconds: dt.second() as u8,
            }
        } else {
            // Provide a sensible fallback in case of error.
            Timestamp {
                year_since_1970: 0,
                zero_indexed_month: 0,
                zero_indexed_day: 0,
                hours: 0,
                minutes: 0,
                seconds: 0,
            }
        }
    }
}

pub fn pretty_format_datetime(dt: NaiveDateTime) -> String {
    let mut line: String = String::new();
    core::write!(
        &mut line,
        "{}/{:02}/{:02} {:02}:{:02}:{:02}",
        dt.year(),
        dt.month(),
        dt.day(),
        dt.hour(),
        dt.minute(),
        dt.second(),
    )
    .unwrap();
    line
}
