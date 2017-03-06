use std::ffi::OsStr;

extern crate dmx_serial as serial;

#[macro_use]
extern crate lazy_static;

use std::{cmp, thread, time};

// The ideal baudrate for sending a break is 45,455 baud.
// At this rate, sendin an 8-bit 0x00 will take the recommended 176 us
// The following stop bit would take a reasonable 22 us.
//
// However, this non-standard baud rate is not supported. The closest common
// baud rates are 57,600 bit/s and 38,400 bit/s. The former is chose here,
// resulting in the following timings:
//
// BREAK:                 138 us    (spec minimum is 92 uS)
// actual BREAK
// MARK-AFTER-BREAK:      17 us    (spec minimum is  8 uS)

const BREAK_SETTINGS: serial::PortSettings = serial::PortSettings {
    baud_rate: serial::Baud57600,
    char_size: serial::Bits7,
    parity: serial::ParityNone,
    stop_bits: serial::Stop1,
    flow_control: serial::FlowNone,
};

const DMX_SETTINGS: serial::PortSettings = serial::PortSettings {
    // DMX calls for 250_000 baud
    baud_rate: serial::BaudOther(250_000),
    char_size: serial::Bits8,
    parity: serial::ParityNone,
    stop_bits: serial::Stop2,
    flow_control: serial::FlowNone,
};

lazy_static! {
    // break will take at least 138 uS, followed by MAB of at least 8 uS
    // we use a sleep + discard instead of using the kernel's builtin flushing
    // functions, as they are much too slow
    static ref SERIAL_TOTAL_BREAK: time::Duration = time::Duration::new(0, 136_000);
}

pub trait DmxTransmitter {
    fn send_break(&mut self) -> serial::Result<()>;

    fn send_raw_data(&mut self, data: &[u8]) -> serial::Result<()>;

    #[inline(always)]
    fn send_dmx_packet(&mut self, data: &[u8]) -> serial::Result<()> {
        self.send_dmx_alt_packet(data, 0x00)
    }

    #[inline]
    fn send_dmx_alt_packet(&mut self, data: &[u8], start: u8) -> serial::Result<()> {
        let mut prefixed = [0; 513];
        let dlen = cmp::min(data.len(), 512);

        // prepare prefixed packet
        prefixed[0] = start;
        prefixed[1..(dlen + 1)].clone_from_slice(data);

        self.send_raw_dmx_packet(&prefixed)
    }

    fn send_raw_dmx_packet(&mut self, data: &[u8]) -> serial::Result<()>;
}


impl<T: serial::SerialPort> DmxTransmitter for T {
    #[inline(always)]
    fn send_break(&mut self) -> serial::Result<()> {
        self.configure(&BREAK_SETTINGS)?;
        self.write(&[0x00])?;
        Ok(())
    }

    #[inline(always)]
    fn send_raw_data(&mut self, data: &[u8]) -> serial::Result<()> {
        self.configure(&DMX_SETTINGS)?;
        self.write(data)?;
        Ok(())
    }

    #[inline]
    fn send_raw_dmx_packet(&mut self, data: &[u8]) -> serial::Result<()> {
        // make a copy to ensure no pointer dereferencing overhead later
        let min_brk = *SERIAL_TOTAL_BREAK;

        self.send_break()?;
        thread::sleep(min_brk);
        self.send_raw_data(data)?;

        Ok(())
    }
}

pub fn open_serial<T: AsRef<OsStr> + ?Sized>(port: &T) -> serial::Result<serial::SystemPort> {
    serial::open(port)
}
