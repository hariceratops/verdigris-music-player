use core::fmt::Write;
use ds323x::{DateTimeAccess, Datelike, Ds323x, NaiveDateTime, Timelike, interface::I2cInterface};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_rp::i2c::I2c;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex,
};
use embedded_sdmmc::{TimeSource, Timestamp};
use heapless::String;
use static_cell::StaticCell;

pub(crate) type RtcAsyncMutex<'a> = mutex::Mutex<CriticalSectionRawMutex, Rtc>;
pub(crate) type RtcAsyncMutexGuard<'a> = mutex::MutexGuard<'a, CriticalSectionRawMutex, Rtc>;
pub(crate) type Rtc = Ds323x<
    I2cInterface<
        I2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Blocking>>,
    >,
    ds323x::ic::DS3231,
>;

pub struct RtcTimeSource<'a> {
    rtc: &'a mut RtcAsyncMutex<'a>,
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    GetTimeError,
    SetTimeError,
}

#[allow(dead_code)]
impl<'a> RtcTimeSource<'a> {
    pub fn new(rtc: Rtc) -> Self {
        static RTC: StaticCell<RtcAsyncMutex> = StaticCell::new();
        let rtc = RTC.init(mutex::Mutex::new(rtc));
        Self { rtc: rtc }
    }

    pub fn with_rtc_sync<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut RtcAsyncMutexGuard) -> R,
    {
        let mut rtc = embassy_futures::block_on(self.rtc.lock());
        f(&mut rtc)
    }

    pub async fn with_rtc<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut RtcAsyncMutexGuard) -> R,
    {
        let mut rtc = self.rtc.lock().await;
        f(&mut rtc)
    }

    pub async fn get_datetime(&self) -> Result<NaiveDateTime, Error> {
        // Original error: ds323x::Error<embassy_embedded_hal::shared_bus::I2cDeviceError<embassy_rp::i2c::Error>>
        self.with_rtc(|rtc| rtc.datetime().map_err(|_| Error::GetTimeError))
            .await
    }

    pub async fn pretty_print_now(&self) -> Result<String<20>, Error> {
        if let Ok(dt) = self.with_rtc(|rtc| rtc.datetime()).await {
            Ok(pretty_format_datetime(dt))
        } else {
            Err(Error::GetTimeError)
        }
    }

    pub async fn set_datetime_from_iso8601(&self, dt_str: &str) -> Result<(), Error> {
        // self.with_rtc(|rtc| rtc.set_datetime(&dt).map_err(|_| Error::SetTimeError)).await
        if let Ok(dt) = NaiveDateTime::parse_from_str(dt_str, "%Y-%m-%d %H:%M:%S") {
            self.with_rtc(|rtc| rtc.set_datetime(&dt).map_err(|_| Error::SetTimeError)).await
        } else {
            Err(Error::SetTimeError)
        }
    }
}

impl<'a> TimeSource for RtcTimeSource<'a> {
    fn get_timestamp(&self) -> Timestamp {
        if let Ok(dt) = self.with_rtc_sync(|rtc| rtc.datetime()) {
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
