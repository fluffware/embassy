use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin as FuturePin;
use core::task::{Context, Poll};

use embassy_cortex_m::interrupt::{Interrupt, InterruptExt};
use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{Drive, SlewRate,sealed::Pin, Pull};

use crate::{interrupt, pac, peripherals};

const PIOS: [&pac::pio::Pio; 2] = [&pac::PIO0, &pac::PIO1];
const NEW_AW: AtomicWaker = AtomicWaker::new();
const PIO_WAKERS_INIT: [AtomicWaker; 4] = [NEW_AW; 4];
static FIFO_OUT_WAKERS: [[AtomicWaker; 4]; 2] = [PIO_WAKERS_INIT; 2];
static FIFO_IN_WAKERS: [[AtomicWaker; 4]; 2] = [PIO_WAKERS_INIT; 2];

pub enum FifoJoin {
    /// Both TX and RX fifo is enabled
    Duplex,
    /// Rx fifo twice as deep. TX fifo disabled
    RxOnly,
    /// Tx fifo twice as deep. RX fifo disabled
    TxOnly,
}

#[derive(PartialEq)]
pub enum ShiftDirection {
    Right = 1,
    Left = 0,
}

#[interrupt]
unsafe fn PIO0_IRQ_1() {
    use crate::pac;
    let ints = pac::PIO0.irqs(1).ints().read().0;
    let inte = pac::PIO0.irqs(1).inte();
    // Check all RXNFULL
    for i in 0..4 {
        if ints & (1 << i) != 0 {
            inte.modify(|m| {
                m.0 &= !(1 << i);
            });
            FIFO_IN_WAKERS[0][i].wake();
        }
    }
}

#[interrupt]
unsafe fn PIO1_IRQ_1() {
    use crate::pac;
    let ints = pac::PIO1.irqs(1).ints().read().0;
    let inte = pac::PIO1.irqs(1).inte();
    // Check all RXNFULL
    for i in 0..4 {
        if ints & (1 << i) != 0 {
            inte.modify(|m| {
                m.0 &= !(1 << i);
            });
            FIFO_IN_WAKERS[1][i].wake();
        }
    }
}

#[interrupt]
unsafe fn PIO0_IRQ_0() {
    use crate::pac;
    let ints = pac::PIO0.irqs(0).ints().read().0 >> 4;
    let inte = pac::PIO0.irqs(0).inte();
    //debug!("!{:04x}",ints);
    // Check all TXNFULL
    for i in 0..4 {
        if ints & (1 << i) != 0 {
            inte.modify(|m| {
                m.0 &= !(1 << (i + 4));
            });
            FIFO_OUT_WAKERS[0][i].wake();
        }
    }
}

#[interrupt]
unsafe fn PIO1_IRQ_0() {
    use crate::pac;
    let ints = pac::PIO1.irqs(0).ints().read().0 >> 4;
    let inte = pac::PIO1.irqs(0).inte();
    // Check all TXNFULL
    for i in 0..4 {
        if ints & (1 << i) != 0 {
            inte.modify(|m| {
                m.0 &= !(1 << (i + 4));
            });
            //debug!("!{:04x}",ints);
            FIFO_OUT_WAKERS[1][i].wake();
        }
    }
}

/// Future that waits for TX-FIFO to become writable
pub struct FifoOutFuture<'a, PIO: PioInstanceTrait, const SM: u8> {
    sm: &'a PioStateMachine<PIO, SM>,
    pio: PhantomData<PIO>,
    value: u32,
}

impl<'a, PIO: PioInstanceTrait, const SM: u8> FifoOutFuture<'a, PIO, SM> {
    pub fn new(sm: &'a PioStateMachine<PIO, SM>, value: u32) -> Self {
        unsafe {
            let irq = PIO::IrqOut::steal();
            irq.disable();
            irq.set_priority(interrupt::Priority::P3);

            irq.enable();
        }
        FifoOutFuture {
            sm,
            pio: PhantomData::default(),
            value,
        }
    }
}

impl<'d, PIO: PioInstanceTrait, const SM: u8> Future for FifoOutFuture<'d, PIO, SM> {
    type Output = ();
    fn poll(self: FuturePin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        //debug!("Poll {},{}", PIO::PIO_NO, SM);
        if self.sm.try_push_tx(self.value) {
            Poll::Ready(())
        } else {
            FIFO_OUT_WAKERS[PIO::PIO_NO as usize][SM as usize].register(cx.waker());
            unsafe {
                let irq = PIO::IrqOut::steal();
                irq.disable();
                critical_section::with(|_| {
                    PIOS[PIO::PIO_NO as usize].irqs(0).inte().modify(|m| {
                        m.0 |= 1 << (SM + 4);
                    });
                });
                irq.enable();
            }
            // debug!("Pending");
            Poll::Pending
        }
    }
}

/// Future that waits for RX-FIFO to become readable
pub struct FifoInFuture<'a, PIO: PioInstanceTrait, const SM: u8> {
    sm: &'a PioStateMachine<PIO, SM>,
    pio: PhantomData<PIO>,
}

impl<'a, PIO: PioInstanceTrait, const SM: u8> FifoInFuture<'a, PIO, SM> {
    pub fn new(sm: &'a PioStateMachine<PIO, SM>) -> Self {
        unsafe {
            let irq = PIO::IrqIn::steal();
            irq.disable();
            irq.set_priority(interrupt::Priority::P3);

            irq.enable();
        }
        FifoInFuture {
            sm,
            pio: PhantomData::default(),
        }
    }
}

impl<'d, PIO: PioInstanceTrait, const SM: u8> Future for FifoInFuture<'d, PIO, SM> {
    type Output = u32;
    fn poll(self: FuturePin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        //debug!("Poll {},{}", PIO::PIO_NO, SM);
        if let Some(v) = self.sm.try_pull_rx() {
            Poll::Ready(v)
        } else {
            FIFO_IN_WAKERS[PIO::PIO_NO as usize][SM as usize].register(cx.waker());
            unsafe {
                let irq = PIO::IrqIn::steal();
                irq.disable();
                critical_section::with(|_| {
                    PIOS[PIO::PIO_NO as usize].irqs(1).inte().modify(|m| {
                        m.0 |= 1 << SM;
                    });
                });
                irq.enable();
            }
            //debug!("Pending");
            Poll::Pending
        }
    }
}
pub struct PioPin<PIO: PioInstanceTrait> {
    pin_bank: u8,
    pio: PhantomData<PIO>,
}

impl<PIO: PioInstanceTrait> PioPin<PIO> {
    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: Drive) {
        unsafe {
            self.pad_ctrl().modify(|w| {
                w.set_drive(match strength {
                    Drive::_2mA => pac::pads::vals::Drive::_2MA,
                    Drive::_4mA => pac::pads::vals::Drive::_4MA,
                    Drive::_8mA => pac::pads::vals::Drive::_8MA,
                    Drive::_12mA => pac::pads::vals::Drive::_12MA,
                });
            });
        }
    }

    // Set the pin's slew rate.
    #[inline]
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        unsafe {
            self.pad_ctrl().modify(|w| {
                w.set_slewfast(slew_rate == SlewRate::Fast);
            });
        }
    }

   /// Set the pin's pull.
    #[inline]
    pub fn set_pull(&mut self, pull: Pull) {
        unsafe {
            self.pad_ctrl().modify(|w| {
                match pull {
                    Pull::Up => w.set_pue(true),
                    Pull::Down => w.set_pde(true),
                    Pull::None => {}
                }
            });
        }
    }

    /// Set the pin's pull.
    #[inline]
    pub fn set_schmitt(&mut self, enable: bool) {
        unsafe {
            self.pad_ctrl().modify(|w| {
		w.set_schmitt(enable);
            });
        }
    }

    
    
}

impl<PIO: PioInstanceTrait> Pin for PioPin<PIO> {
    fn pin_bank(&self) -> u8 {
        self.pin_bank
    }
}

pub struct PioStateMachine<PIO: PioInstanceTrait, const SM: u8> {
    pio: PhantomData<PIO>,
}

impl<PIO: PioInstanceTrait, const SM: u8> PioStateMachine<PIO, SM> {
    pub fn pio(&self) -> u8 {
        let _ = self;
        PIO::PIO_NO
    }

    pub fn sm(&self) -> u8 {
        let _ = self;
        SM
    }

    pub fn restart(&self) {
        let _ = self;
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .ctrl()
                .modify(|w| w.set_sm_restart(1u8 << SM));
        }
    }
    pub fn set_enable(&self, enable: bool) {
        let _ = self;
        let mask = 1u8 << SM;
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .ctrl()
                .modify(|w| w.set_sm_enable(w.sm_enable() | (if enable { mask } else { 0 })));
        }
    }

    pub fn is_enabled(&self) -> bool {
        let _ = self;
        unsafe { PIOS[PIO::PIO_NO as usize].ctrl().read().sm_enable() & (1u8 << SM) != 0 }
    }
   
    pub fn is_tx_empty(&self) -> bool {
        let _ = self;
        unsafe { PIOS[PIO::PIO_NO as usize].fstat().read().txempty() & (1u8 << SM) != 0 }
    }
    pub fn is_tx_full(&self) -> bool {
        let _ = self;
        unsafe { PIOS[PIO::PIO_NO as usize].fstat().read().txfull() & (1u8 << SM) != 0 }
    }

    pub fn is_rx_empty(&self) -> bool {
        let _ = self;
        unsafe { PIOS[PIO::PIO_NO as usize].fstat().read().rxempty() & (1u8 << SM) != 0 }
    }
    pub fn is_rx_full(&self) -> bool {
        let _ = self;
        unsafe { PIOS[PIO::PIO_NO as usize].fstat().read().rxfull() & (1u8 << SM) != 0 }
    }

    pub fn push_tx(&self, v: u32) {
        unsafe {
            PIOS[PIO::PIO_NO as usize].txf(SM as usize).write_value(v);
        }
    }

    pub fn try_push_tx(&self, v: u32) -> bool {
        if self.is_tx_full() {
            return false;
        }
        self.push_tx(v);
        true
    }

    pub fn pull_rx(&self) -> u32 {
        unsafe { PIOS[PIO::PIO_NO as usize].rxf(SM as usize).read() }
    }

    pub fn try_pull_rx(&self) -> Option<u32> {
        if self.is_rx_empty() {
            return None;
        }
        Some(self.pull_rx())
    }

    pub fn set_clkdiv(&self, div_x_256: u32) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .clkdiv()
                .write(|w| w.0 = div_x_256 << 8);
        }
    }

    pub fn get_clkdiv(&self) -> u32 {
        unsafe { PIOS[PIO::PIO_NO as usize].sm(SM as usize).clkdiv().read().0 >> 8 }
    }

    pub fn clkdiv_restart(&self) {
        let _ = self;
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .ctrl()
                .modify(|w| w.set_clkdiv_restart(1u8 << SM));
        }
    }
    
    pub fn set_side_enable(&self, enable: bool) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .execctrl()
                .modify(|w| w.set_side_en(enable));
        }
    }

    pub fn is_side_enabled(&self) -> bool {
        unsafe { PIOS[PIO::PIO_NO as usize].sm(SM as usize).execctrl().read().side_en() }
    }

    pub fn set_side_pindir(&self, pindir: bool) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .execctrl()
                .modify(|w| w.set_side_pindir(pindir));
        }
    }

    pub fn is_side_pindir(&self) -> bool {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .execctrl()
                .read()
                .side_pindir()
        }
    }

    pub fn set_wrap(&self, source: u8, target: u8) {
        unsafe {
            PIOS[PIO::PIO_NO as usize].sm(SM as usize).execctrl().modify(|w| {
                w.set_wrap_top(source);
                w.set_wrap_bottom(target)
            });
        }
    }

    /// Get wrapping addresses. Returns (source, target).
    pub fn get_wrap(&self) -> (u8, u8) {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).execctrl().read();
            (r.wrap_top(), r.wrap_bottom())
        }
    }

    pub fn set_fifo_join(&self, join: FifoJoin) {
        let (rx, tx) = match join {
            FifoJoin::Duplex => (false, false),
            FifoJoin::RxOnly => (true, false),
            FifoJoin::TxOnly => (false, true),
        };
        unsafe {
            PIOS[PIO::PIO_NO as usize].sm(SM as usize).shiftctrl().modify(|w| {
                w.set_fjoin_rx(rx);
                w.set_fjoin_tx(tx)
            });
        }
    }
    pub fn get_fifo_join(&self) -> FifoJoin {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).shiftctrl().read();
            // Ignores the invalid state when both bits are set
            if r.fjoin_rx() {
                FifoJoin::RxOnly
            } else if r.fjoin_tx() {
                FifoJoin::TxOnly
            } else {
                FifoJoin::Duplex
            }
        }
    }

    pub fn clear_fifos(&self)
    {
	// Toggle FJOIN_RX to flush FIFOs
	unsafe {
	    let shiftctrl = PIOS[PIO::PIO_NO as usize].sm(SM as usize).shiftctrl();
	    shiftctrl.modify(|w| {
                w.set_fjoin_rx(!w.fjoin_rx());
            });
	    shiftctrl.modify(|w| {
                w.set_fjoin_rx(!w.fjoin_rx());
            });
	    
	}
    }
    
    pub fn set_pull_threshold(&self, threshold: u8) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .shiftctrl()
                .modify(|w| w.set_pull_thresh(threshold));
        }
    }

    pub fn get_pull_threshold(&self) -> u8 {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).shiftctrl().read();
            r.pull_thresh()
        }
    }
    pub fn set_push_threshold(&self, threshold: u8) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .shiftctrl()
                .modify(|w| w.set_push_thresh(threshold));
        }
    }

    pub fn get_push_threshold(&self) -> u8 {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).shiftctrl().read();
            r.push_thresh()
        }
    }

    pub fn set_out_shift_dir(&self, dir: ShiftDirection) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .shiftctrl()
                .modify(|w| w.set_out_shiftdir(dir == ShiftDirection::Right));
        }
    }
    pub fn get_out_shiftdir(&self) -> ShiftDirection {
        unsafe {
            if PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .shiftctrl()
                .read()
                .out_shiftdir()
            {
                ShiftDirection::Right
            } else {
                ShiftDirection::Left
            }
        }
    }

    pub fn set_in_shift_dir(&self, dir: ShiftDirection) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .shiftctrl()
                .modify(|w| w.set_in_shiftdir(dir == ShiftDirection::Right));
        }
    }
    pub fn get_in_shiftdir(&self) -> ShiftDirection {
        unsafe {
            if PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .shiftctrl()
                .read()
                .in_shiftdir()
            {
                ShiftDirection::Right
            } else {
                ShiftDirection::Left
            }
        }
    }

    pub fn set_autopull(&self, auto: bool) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .shiftctrl()
                .modify(|w| w.set_autopull(auto));
        }
    }

    pub fn is_autopull(&self) -> bool {
        unsafe { PIOS[PIO::PIO_NO as usize].sm(SM as usize).shiftctrl().read().autopull() }
    }

    pub fn set_autopush(&self, auto: bool) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .shiftctrl()
                .modify(|w| w.set_autopush(auto));
        }
    }

    pub fn is_autopush(&self) -> bool {
        unsafe { PIOS[PIO::PIO_NO as usize].sm(SM as usize).shiftctrl().read().autopush() }
    }

    pub fn get_addr(&self) -> u8 {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).addr().read();
            r.addr()
        }
    }
    pub fn set_sideset_count(&self, count: u8) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .pinctrl()
                .modify(|w| w.set_sideset_count(count));
        }
    }

    pub fn get_sideset_count(&self) -> u8 {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).pinctrl().read();
            r.sideset_count()
        }
    }

    pub fn make_pio_pin(&self, pin: impl Pin) -> PioPin<PIO> {
        unsafe {
            pin.io().ctrl().write(|w| {
                w.set_funcsel(
                    if PIO::PIO_NO == 1 {
                        pac::io::vals::Gpio0CtrlFuncsel::PIO1_0
                    } else {
                        // PIO == 0
                        pac::io::vals::Gpio0CtrlFuncsel::PIO0_0
                    }
                    .0,
                );
            });
        }
        PioPin {
            pin_bank: pin.pin_bank(),
            pio: PhantomData::default(),
        }
    }

    pub fn set_sideset_base_pin(&self, base_pin: &PioPin<PIO>) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .pinctrl()
                .modify(|w| w.set_sideset_base(base_pin.pin()));
        }
    }

    pub fn get_sideset_base(&self) -> u8 {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).pinctrl().read();
            r.sideset_base()
        }
    }

    /// Set the range of out pins affected by a set instruction.
    pub fn set_set_range(&self, base: u8, count: u8) {
        assert!(base + count < 32);
        unsafe {
            PIOS[PIO::PIO_NO as usize].sm(SM as usize).pinctrl().modify(|w| {
                w.set_set_base(base);
                w.set_set_count(count)
            });
        }
    }

    /// Get the range of out pins affected by a set instruction. Returns (base, count).
    pub fn get_set_range(&self) -> (u8, u8) {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).pinctrl().read();
            (r.set_base(), r.set_count())
        }
    }

    pub fn set_in_base_pin(&self, base: &PioPin<PIO>) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .pinctrl()
                .modify(|w| w.set_in_base(base.pin()));
        }
    }

    pub fn get_in_base(&self) -> u8 {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).pinctrl().read();
            r.in_base()
        }
    }

    pub fn set_out_range(&self, base: u8, count: u8) {
        assert!(base + count < 32);
        unsafe {
            PIOS[PIO::PIO_NO as usize].sm(SM as usize).pinctrl().modify(|w| {
                w.set_out_base(base);
                w.set_out_count(count)
            });
        }
    }

    /// Get the range of out pins affected by a set instruction. Returns (base, count).
    pub fn get_out_range(&self) -> (u8, u8) {
        unsafe {
            let r = PIOS[PIO::PIO_NO as usize].sm(SM as usize).pinctrl().read();
            (r.out_base(), r.out_count())
        }
    }

    pub fn set_out_pins<'a, 'b: 'a>(&'a self, pins: &'b [&PioPin<PIO>]) {
        let count = pins.len();
        assert!(count >= 1);
        let start = pins[0].pin() as usize;
        assert!(start + pins.len() <= 32);
        for i in 0..count {
            assert!(pins[i].pin() as usize == start + i, "Pins must be sequential");
        }
        self.set_out_range(start as u8, count as u8);
    }
    
    pub fn set_set_pins<'a, 'b: 'a>(&'a self, pins: &'b [&PioPin<PIO>]) {
        let count = pins.len();
        assert!(count >= 1);
        let start = pins[0].pin() as usize;
        assert!(start + pins.len() <= 32);
        for i in 0..count {
            assert!(pins[i].pin() as usize == start + i, "Pins must be sequential");
        }
        self.set_set_range(start as u8, count as u8);
    }

    pub fn get_current_instr() -> u32 {
        unsafe { PIOS[PIO::PIO_NO as usize].sm(SM as usize).instr().read().0 }
    }

    pub fn exec_instr(&self, instr: u16) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .sm(SM as usize)
                .instr()
                .write(|w| w.set_instr(instr));
        }
    }

  
    pub fn wait_push<'a>(&'a self, value: u32) -> FifoOutFuture<'a, PIO, SM> {
        FifoOutFuture::new(&self, value)
    }

    pub fn wait_pull<'a>(&'a self) -> FifoInFuture<'a, PIO, SM> {
        FifoInFuture::new(&self)
    }
}

pub struct PioCommon<PIO: PioInstanceTrait> {
    pio: PhantomData<PIO>,
}

impl<PIO: PioInstanceTrait> PioCommon<PIO> {
    pub fn write_instr(&self, start: usize, instrs: &[u16]) {
        let _ = self;
        for (i, instr) in instrs.iter().enumerate() {
            unsafe {
                PIOS[PIO::PIO_NO as usize].instr_mem(i + start).write(|w| {
                    w.set_instr_mem(*instr);
                });
            }
        }
    }

    pub fn clear_irq(&self, irq_no: usize) {
        assert!(irq_no < 8);
        unsafe { PIOS[PIO::PIO_NO as usize].irq().write(|w| w.set_irq(1 << irq_no)) }
    }

    pub fn clear_irqs(&self, mask: u8) {
        unsafe { PIOS[PIO::PIO_NO as usize].irq().write(|w| w.set_irq(mask)) }
    }

    pub fn force_irq(&self, irq_no: usize) {
        assert!(irq_no < 8);
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .irq_force()
                .write(|w| w.set_irq_force(1 << irq_no))
        }
    }

    pub fn set_input_sync_bypass<'a>(&'a self, bypass: u32, mask: u32) {
        unsafe {
            PIOS[PIO::PIO_NO as usize]
                .input_sync_bypass()
                .modify(|w| *w = (*w & !mask) | (bypass & mask));
        }
    }

    pub fn set_input_sync_bypass_pins<'a>(&self, pins: &[&PioPin<PIO>]) {
        let mut bypass = 0;
        for pin in pins {
            bypass |= 1 << pin.pin();
        }
        self.set_input_sync_bypass(bypass, bypass);
    }

    pub fn get_input_sync_bypass(&self) -> u32 {
        unsafe { PIOS[PIO::PIO_NO as usize].input_sync_bypass().read() }
    }
}

pub struct PioInstance<const PIO_NO: u8> {}

pub trait Pio<PIO: PioInstanceTrait>: Sized {
    fn pio(&self) -> u8 {
        let _ = self;
        PIO::PIO_NO
    }

    fn split(
        self,
    ) -> (
        PioCommon<PIO>,
        PioStateMachine<PIO, 0>,
        PioStateMachine<PIO, 1>,
        PioStateMachine<PIO, 2>,
        PioStateMachine<PIO, 3>,
    ) {
        let _ = self;
        (
            PioCommon {
                pio: PhantomData::default(),
            },
            PioStateMachine {
                pio: PhantomData::default(),
            },
            PioStateMachine {
                pio: PhantomData::default(),
            },
            PioStateMachine {
                pio: PhantomData::default(),
            },
            PioStateMachine {
                pio: PhantomData::default(),
            },
        )
    }
}

pub trait PioInstanceTrait {
    const PIO_NO: u8;
    type IrqOut: Interrupt;
    type IrqIn: Interrupt;
}

impl PioInstanceTrait for PioInstance<0> {
    const PIO_NO: u8 = 0;
    type IrqOut = interrupt::PIO0_IRQ_0;
    type IrqIn = interrupt::PIO0_IRQ_1;
}

impl PioInstanceTrait for PioInstance<1> {
    const PIO_NO: u8 = 1;
    type IrqOut = interrupt::PIO1_IRQ_0;
    type IrqIn = interrupt::PIO1_IRQ_1;
}

macro_rules! impl_pio_sm {
    ($name:ident, $pio:expr) => {
        impl Pio<PioInstance<$pio>> for peripherals::$name {}
    };
}

impl_pio_sm!(PIO0, 0);
impl_pio_sm!(PIO1, 1);
