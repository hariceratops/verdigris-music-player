use core::{
    cell::{RefCell, RefMut},
    fmt::Write,
};
use ds323x::{DateTimeAccess, Datelike, NaiveDateTime, Timelike};
use embedded_sdmmc::{TimeSource, Timestamp};
use heapless::String;

use crate::bus::RtcType;

pub struct RtcTimeSource {
    rtc: RefCell<RtcType>,
}

impl RtcTimeSource {
    pub fn new(rtc: RtcType) -> Self {
        Self {
            rtc: RefCell::new(rtc),
        }
    }

    pub fn with_rtc<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut RefMut<RtcType>) -> R,
    {
        let mut rtc = self.rtc.borrow_mut();
        f(&mut rtc)
    }

    pub fn get_datetime(
        &self,
    ) -> Result<
        NaiveDateTime,
        ds323x::Error<embassy_embedded_hal::shared_bus::I2cDeviceError<embassy_rp::i2c::Error>>,
    > {
        self.with_rtc(|rtc| rtc.datetime())
    }
}

// impl<I2C> Clone for RtcTimeSource<I2C> {
//     fn clone(&self) -> Self {
//         SharedRtcTimeSource(self.0.clone())
//     }
// }

impl TimeSource for RtcTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        if let Ok(dt) = self.with_rtc(|rtc| rtc.datetime()) {
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

pub fn pretty_format_datetime(dt: NaiveDateTime) -> String<20> {
    let mut line: String<20> = String::new();
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
