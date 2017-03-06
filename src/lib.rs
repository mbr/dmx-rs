//! DMX512
//!
//! The "Digital Multiplex" (DMX) protocol is used to control stage lighting
//! and effects in large and small setups. It is electrically based on [RS485]
//! (https://en.wikipedia.org/wiki/RS-485) and can (almost) easily be
//! implemented on microcontrollers as well as soft real-time capable operating
//! systems.
//!
//! ## The protocol
//!
//! The protocol itself assumes a single-master/multiple-slave system. The
//! master periodically sends updates for up to 512 *channels*. Each one of
//! these updates is called a DMX *packet*, which consists of a *start code*
//! and any number of channels, in-order starting from channel 1. Packets can
//! contain less than 512 channels, but must always start at 1 and progress
//! in order.
//!
//! Channels are byte values ranging from 0 to 255. For most channels these
//! are intensity values, with 0 being "off" and 255 "full brightness". However
//! other functions may be connected to a channel, such as selecting a blink
//! sequence or setting a servo position.
//!
//! ## Technical details
//!
//! DMX is transmitted using serial protocol at the unusual bitrate of
//! 250,000 baud, with no parity and two stop bits.
//!
//! To begin a transmission, a sender must first pull the line low to send a
//! so called *break*, followed by pulling it high to send a *mark*. The
//! duration of this break is fairly long, as is the mark, both being far
//! longer than the time it usually takes to submit a single byte.
//!
//! After the break/mark-after-break sequence, regular transmission begins at
//! 250,000 baud by sending a single-byte start code. This start code is almost
//! always `0xFF`, unless special functions are used, which are
//! vendor-specific.
//!
//! Right after the start code, any number of channels may be transmitted.
//!
//! ## Refresh rate
//!
//! The refresh rate depends on the number of channels transmitted. For the
//! full 512 channels, the maximum achievable standard-compliant refresh rate
//! is about 44 frames per second. If less than 512 channels are sent inside
//! a packet, higher refresh rate are possible.
//!
//! It should be noted that there is a minimum time between breaks (and
//! therefore DMX packets) of 1204 microseconds, theoretically capping the
//! refresh rate at about 830 updates per second.
//!
//! DMX is usually meant to be sent continuously, with at least one update
//! per second. A lot of devices will switch off if intervals become too large.
//!
//! ## More information
//!
//! The [DMX512-A standard]
//! (http://tsp.esta.org/tsp/documents/docs/E1-11_2008R2013.pdf) contains the
//! detailed specification.
//!
//! # Implementations
//!
//! Currently, only an implementation using Linux serial devices is available.
//! Connecting a UART to an RS485 transceiver is enough to get this working.
//! The implementation is not 100% optimal for DMX: As most Linux kernels
//! are not real-time capable, perfectly stable frame rates are not always
//! achievable. However, the DMX protocol is fairly tolerant of loose timing.
//!
//! The UARTs must support non-standard baudrates and reasonably fast baud-rate
//! switching. Sending a break is done by switch to a slow baud-rate, sending
//! a single `0x00` byte, then waiting a bit and switching back to 250,000
//! baud.
//!
//! ## Example
//!
//! The interface is fairly simple to use:
//!
//! ```no_run
//!    use dmx::{self, DmxTransmitter};
//!    use std::{thread, time};
//!
//!    let mut dmx_port = dmx::open_serial("/dev/ttyS1").unwrap();
//!
//!    // a typical 4-channel device, expecting RGBV values. this will set a
//!    // fairly bright yellow.
//!    let data = &[0xe4, 0xe4, 0x00, 0xca];
//!
//!    loop {
//!        dmx_port.send_dmx_packet(data).unwrap();
//!
//!        // repeat about every 51 ms. for more accurate frame timings,
//!        // consider using the ticktock crate.
//!        thread::sleep(time::Duration::new(0, 50_000_000));
//!    }
//! ```

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

/// A DMX transmitter.
///
/// Usually there is one transmitter on a bus, the master. Transmitters send
/// DMX data.
pub trait DmxTransmitter {
    /// Send a single break.
    ///
    /// Sends a break and returns as soon as possible afterwards. A caller is
    /// itself responsible for waiting an appropriate amount of time before
    /// sending data.
    fn send_break(&mut self) -> serial::Result<()>;

    /// Send raw data.
    ///
    /// Sends out bytes at the appropriate bitrate for DMX. Does **not** send
    /// a break first. Returns after the data is buffered, which might be
    /// before transmitting is complete.
    fn send_raw_data(&mut self, data: &[u8]) -> serial::Result<()>;

    /// Blocking send a full DMX packet.
    ///
    /// Preprends a default start code of `0x00` before sending a
    /// `break` and channel data.
    /// Will block until the full send is buffered to be sent out, but may
    /// return before the transmission is complete.
    ///
    /// This will create an additional stack copy of `channels`; see
    /// `send_dmx_alt_packet` for details.
    #[inline(always)]
    fn send_dmx_packet(&mut self, channels: &[u8]) -> serial::Result<()> {
        self.send_dmx_alt_packet(channels, 0x00)
    }

    /// Blocking send a full DMX packet with a non-standard start code.
    ///
    /// Prepends an arbitrary start code `start` to a packet using a buffer
    /// on the stack (no allocations are made, but the cost of an extra copy
    /// is incurred). If required, this can be avoided by using
    /// `send_raw_dmx_packet`.
    ///
    /// Like `send_dmx_packet` will send a break first and returns after
    /// buffering.
    #[inline]
    fn send_dmx_alt_packet(&mut self, channels: &[u8], start: u8) -> serial::Result<()> {
        let mut prefixed = [0; 513];
        let dlen = cmp::min(channels.len(), 512);

        // prepare prefixed packet
        prefixed[0] = start;
        prefixed[1..(dlen + 1)].clone_from_slice(channels);

        self.send_raw_dmx_packet(&prefixed)
    }

    /// Blocking send a DMX packet including start code.
    ///
    /// Sends a break, followed by the specified data. Returns after buffering.
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

/// Opens a serial device with DMX support.
pub fn open_serial<T: AsRef<OsStr> + ?Sized>(port: &T) -> serial::Result<serial::SystemPort> {
    serial::open(port)
}
