use paste::paste;

use super::gpio::{sealed, AnyPin};
use crate::gpio::sealed::Pin as SealedPin;
use crate::gpio::Pin;
use crate::pac::PWM;
use crate::{pac, peripherals};

const PWM_CHANNEL_A: u8 = 0;
const PWM_CHANNEL_B: u8 = 1;

pub trait PwmPin<const SLICE: u8, const CHANNEL: u8> {
    fn pwm_slice_num(&self) -> u8 {
        SLICE
    }
}

macro_rules! impl_pin {
    ($name:ident, $slice:expr, $channel:ident) => {
        impl PwmPin<$slice, $channel> for peripherals::$name {}
    };
}

impl_pin!(PIN_0, 0, PWM_CHANNEL_A);
impl_pin!(PIN_1, 0, PWM_CHANNEL_B);
impl_pin!(PIN_2, 1, PWM_CHANNEL_A);
impl_pin!(PIN_3, 1, PWM_CHANNEL_B);
impl_pin!(PIN_4, 2, PWM_CHANNEL_A);
impl_pin!(PIN_5, 2, PWM_CHANNEL_B);
impl_pin!(PIN_6, 3, PWM_CHANNEL_A);
impl_pin!(PIN_7, 3, PWM_CHANNEL_B);
impl_pin!(PIN_8, 4, PWM_CHANNEL_A);
impl_pin!(PIN_9, 4, PWM_CHANNEL_B);
impl_pin!(PIN_10, 5, PWM_CHANNEL_A);
impl_pin!(PIN_11, 5, PWM_CHANNEL_B);
impl_pin!(PIN_12, 6, PWM_CHANNEL_A);
impl_pin!(PIN_13, 6, PWM_CHANNEL_B);
impl_pin!(PIN_14, 7, PWM_CHANNEL_A);
impl_pin!(PIN_15, 7, PWM_CHANNEL_B);
impl_pin!(PIN_16, 0, PWM_CHANNEL_A);
impl_pin!(PIN_17, 0, PWM_CHANNEL_B);
impl_pin!(PIN_18, 1, PWM_CHANNEL_A);
impl_pin!(PIN_19, 1, PWM_CHANNEL_B);
impl_pin!(PIN_20, 2, PWM_CHANNEL_A);
impl_pin!(PIN_21, 2, PWM_CHANNEL_B);
impl_pin!(PIN_22, 3, PWM_CHANNEL_A);
impl_pin!(PIN_23, 3, PWM_CHANNEL_B);
impl_pin!(PIN_24, 4, PWM_CHANNEL_A);
impl_pin!(PIN_25, 4, PWM_CHANNEL_B);
impl_pin!(PIN_26, 5, PWM_CHANNEL_A);
impl_pin!(PIN_27, 5, PWM_CHANNEL_B);
impl_pin!(PIN_28, 6, PWM_CHANNEL_A);
impl_pin!(PIN_29, 6, PWM_CHANNEL_B);

// PWM output is function 4 for all pads
const PWM_FUNCTION: u8 = 4;

macro_rules! enable_slice {
    ($index:expr) => {
        paste! {
        unsafe {
           PWM.[<ch $index _csr>]().write(|w| {
           w.set_divmode(pac::pwm::vals::Divmode::DIV);
           w.set_en(true);
           })
        }
        }
    };
}

fn enable_slice(slice: u8) {
    match slice {
        0 => enable_slice!(0),
        1 => enable_slice!(1),
        2 => enable_slice!(2),
        3 => enable_slice!(3),
        4 => enable_slice!(4),
        5 => enable_slice!(5),
        6 => enable_slice!(6),
        7 => enable_slice!(7),
        _ => {}
    }
}

macro_rules! set_top {
    ($index:expr, $value: expr) => {
        paste! {
            unsafe {
            PWM.[<ch $index _top>]().write(|w| {
                w.[<set_ch $index _top>]($value)
            })
            }
        }
    };
}

fn set_top(slice: u8, value: u16) {
    match slice {
        0 => set_top!(0, value),
        1 => set_top!(1, value),
        2 => set_top!(2, value),
        3 => set_top!(3, value),
        4 => set_top!(4, value),
        5 => set_top!(5, value),
        6 => set_top!(6, value),
        7 => set_top!(7, value),
        _ => {}
    }
}
macro_rules! set_div {
    ($index:expr, $int: expr, $frac:expr) => {
        paste! {
            unsafe {
            PWM.[<ch $index _div>]().write(|w| {
                w.set_int($int);
                w.set_frac($frac);
            })
            }
        }
    };
}
fn set_div(slice: u8, int: u8, frac: u8) {
    match slice {
        0 => set_div!(0, int, frac),
        1 => set_div!(1, int, frac),
        2 => set_div!(2, int, frac),
        3 => set_div!(3, int, frac),
        4 => set_div!(4, int, frac),
        5 => set_div!(5, int, frac),
        6 => set_div!(6, int, frac),
        7 => set_div!(7, int, frac),
        _ => {}
    }
}

macro_rules! set_cc {
    ($index:expr, $a: expr, $b:expr) => {
        paste! {
            unsafe {
            PWM.[<ch $index _cc>]().modify(|w| {
                if let Some(a) = $a {
                w .set_a(a);
                }
                if let Some(b) = $b {
                w.set_b(b);
                }
            })
            }
        }
    };
}
fn set_cc(slice: u8, a: Option<u16>, b: Option<u16>) {
    match slice {
        0 => set_cc!(0, a, b),
        1 => set_cc!(1, a, b),
        2 => set_cc!(2, a, b),
        3 => set_cc!(3, a, b),
        4 => set_cc!(4, a, b),
        5 => set_cc!(5, a, b),
        6 => set_cc!(6, a, b),
        7 => set_cc!(7, a, b),
        _ => {}
    }
}

pub struct ConfigSlice<const SLICE: u8>;

impl<const SLICE: u8> ConfigSlice<SLICE> {
    pub fn top(self, top: u16) -> Self {
        set_top(SLICE, top);
        self
    }

    pub fn div(self, int: u8, frac: u8) -> Self {
        set_div(SLICE, int, frac);
        self
    }

    pub fn split(
        self,
    ) -> (
        ConfigChannel<SLICE, PWM_CHANNEL_A, false>,
        ConfigChannel<SLICE, PWM_CHANNEL_B, false>,
    ) {
        (ConfigChannel {}, ConfigChannel {})
    }

    pub fn degrade(self) -> AnyConfigSlice {
        AnyConfigSlice { slice: SLICE }
    }
}

pub struct AnyConfigSlice {
    slice: u8,
}

impl AnyConfigSlice {
    pub fn top(self, top: u16) -> Self {
        set_top(self.slice, top);
        self
    }

    pub fn div(self, int: u8, frac: u8) -> Self {
        set_div(self.slice, int, frac);
        self
    }

    pub fn split(self) -> (AnyConfigChannel<false>, AnyConfigChannel<false>) {
        (
            AnyConfigChannel {
                slice: self.slice,
                channel: PWM_CHANNEL_A,
            },
            AnyConfigChannel {
                slice: self.slice,
                channel: PWM_CHANNEL_B,
            },
        )
    }
}

pub struct ConfigChannel<const SLICE: u8, const CHANNEL: u8, const PIN_SET: bool>;

impl<const SLICE: u8, const CHANNEL: u8> ConfigChannel<SLICE, CHANNEL, true> {
    pub fn enable(self) -> EnabledChannel<SLICE, CHANNEL> {
        enable_slice(SLICE);
        EnabledChannel {}
    }
}

impl<const SLICE: u8, const CHANNEL: u8, const PIN_SET: bool> ConfigChannel<SLICE, CHANNEL, PIN_SET> {
    pub fn compare(self, level: u16) -> Self {
        match CHANNEL {
            PWM_CHANNEL_A => set_cc(SLICE, Some(level), None),
            PWM_CHANNEL_B => set_cc(SLICE, None, Some(level)),
            _ => {}
        }
        self
    }

    pub fn degrade(self) -> AnyConfigChannel<PIN_SET> {
        AnyConfigChannel {
            slice: SLICE,
            channel: CHANNEL,
        }
    }
}

impl<const SLICE: u8, const CHANNEL: u8> ConfigChannel<SLICE, CHANNEL, false> {
    pub fn pin(self, pin: impl PwmPin<SLICE, CHANNEL> + sealed::Pin) -> ConfigChannel<SLICE, CHANNEL, true> {
        unsafe {
            pin.io().ctrl().write(|w| {
                w.set_funcsel(PWM_FUNCTION);
            });
        }
        ConfigChannel {}
    }

    pub fn any_pin(self, pin: AnyPin) -> ConfigChannel<SLICE, CHANNEL, true> {
        assert_eq!((pin.pin() / 2) % 16, SLICE as u8, "Pin doesn't map to PWM slice");
        assert_eq!(pin.pin() & 1, CHANNEL as u8, "Pin map to wrong PWM channel");
        unsafe {
            pin.io().ctrl().write(|w| {
                w.set_funcsel(PWM_FUNCTION);
            });
        }
        ConfigChannel {}
    }
}

pub struct AnyConfigChannel<const PIN_SET: bool> {
    slice: u8,
    channel: u8,
}

impl AnyConfigChannel<true> {
    pub fn enable(self) -> AnyEnabledChannel {
        enable_slice(self.slice);
        AnyEnabledChannel {
            slice: self.slice,
            channel: self.channel,
        }
    }
}
impl<const PIN_SET: bool> AnyConfigChannel<PIN_SET> {
    pub fn compare(self, level: u16) -> Self {
        match self.channel {
            PWM_CHANNEL_A => set_cc(self.slice, Some(level), None),
            PWM_CHANNEL_B => set_cc(self.slice, None, Some(level)),
            _ => {}
        }
        self
    }
}

impl AnyConfigChannel<false> {
    pub fn pin<const SLICE: u8, const CHANNEL: u8>(
        self,
        pin: impl PwmPin<SLICE, CHANNEL> + sealed::Pin,
    ) -> AnyConfigChannel<true> {
        unsafe {
            pin.io().ctrl().write(|w| {
                w.set_funcsel(PWM_FUNCTION);
            });
        }
        AnyConfigChannel {
            slice: self.slice,
            channel: self.channel,
        }
    }

    pub fn any_pin(self, pin: AnyPin) -> AnyConfigChannel<true> {
        assert_eq!((pin.pin() / 2) % 16, self.slice, "Pin doesn't map to PWM slice");
        assert_eq!(pin.pin() & 1, self.slice, "Pin map to wrong PWM channel");
        unsafe {
            pin.io().ctrl().write(|w| {
                w.set_funcsel(PWM_FUNCTION);
            });
        }
        AnyConfigChannel {
            slice: self.slice,
            channel: self.channel,
        }
    }
}
pub struct EnabledChannel<const SLICE: u8, const CHANNEL: u8>;

impl<const SLICE: u8, const CHANNEL: u8> EnabledChannel<SLICE, CHANNEL> {
    pub fn compare(&self, level: u16) {
        match CHANNEL {
            PWM_CHANNEL_A => set_cc(SLICE, Some(level), None),
            PWM_CHANNEL_B => set_cc(SLICE, None, Some(level)),
            _ => {}
        }
    }
    pub fn degrade(self) -> AnyEnabledChannel {
        AnyEnabledChannel {
            slice: SLICE,
            channel: CHANNEL,
        }
    }
}
pub struct AnyEnabledChannel {
    slice: u8,
    channel: u8,
}

impl AnyEnabledChannel {
    pub fn compare(&self, level: u16) {
        match self.channel {
            PWM_CHANNEL_A => set_cc(self.slice, Some(level), None),
            PWM_CHANNEL_B => set_cc(self.slice, None, Some(level)),
            _ => {}
        }
    }
}

pub trait Slice<const SLICE: u8> {
    fn config(self) -> ConfigSlice<SLICE>
    where
        Self: Sized,
    {
        ConfigSlice {}
    }
}

macro_rules! impl_slice {
    ($name:ident, $slice:expr) => {
        impl Slice<$slice> for peripherals::$name {}
    };
}

impl_slice!(PWM_SLICE0, 0);
impl_slice!(PWM_SLICE1, 1);
impl_slice!(PWM_SLICE2, 2);
impl_slice!(PWM_SLICE3, 3);
impl_slice!(PWM_SLICE4, 4);
impl_slice!(PWM_SLICE5, 5);
impl_slice!(PWM_SLICE6, 6);
impl_slice!(PWM_SLICE7, 7);
