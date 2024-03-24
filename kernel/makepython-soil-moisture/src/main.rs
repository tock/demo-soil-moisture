// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2024.

//! Tock kernel for the MakePython nRF52840 - Soil Moisture Edition.

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]

use kernel::capabilities;
use kernel::component::Component;
use kernel::hil::time::Counter;
use kernel::hil::usb::Client;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::process::ProcessLoadingAsync;
use kernel::scheduler::round_robin::RoundRobinSched;
#[allow(unused_imports)]
use kernel::{create_capability, debug, debug_gpio, debug_verbose, static_init};

use nrf52840::gpio::Pin;
use nrf52840::interrupt_service::Nrf52840DefaultPeripherals;

mod policy;

// The datasheet and website and everything say this is connected to P1.10, but
// actually looking at the hardware files (and what actually works) is that the
// LED is connected to P1.11 (as of a board I received in September 2023).
//
// https://github.com/Makerfabs/NRF52840/issues/1
const LED_PIN: Pin = Pin::P1_11;

const BUTTON_RST_PIN: Pin = Pin::P0_18;
const BUTTON_PIN: Pin = Pin::P1_15;

const GPIO_SOIL_SENSOR_POWER: Pin = Pin::P1_10;

const _UART_TX_PIN: Pin = Pin::P0_06;
const _UART_RX_PIN: Pin = Pin::P0_08;

const I2C_SDA_PIN: Pin = Pin::P0_26;
const I2C_SCL_PIN: Pin = Pin::P0_27;

// Constants related to the configuration of the 15.4 network stack
/// Personal Area Network ID for the IEEE 802.15.4 radio
const PAN_ID: u16 = 0xABCD;
/// Gateway (or next hop) MAC Address
const DST_MAC_ADDR: capsules_extra::net::ieee802154::MacAddress =
    capsules_extra::net::ieee802154::MacAddress::Short(49138);
const DEFAULT_CTX_PREFIX_LEN: u8 = 8; //Length of context for 6LoWPAN compression
const DEFAULT_CTX_PREFIX: [u8; 16] = [0x0_u8; 16]; //Context for 6LoWPAN Compression

/// UART Writer for panic!()s.
pub mod io;

// How should the kernel respond when a process faults. For this board we choose
// to stop the app and print a notice, but not immediately panic. This allows
// users to debug their apps, but avoids issues with using the USB/CDC stack
// synchronously for panic! too early after the board boots.
const FAULT_RESPONSE: kernel::process::StopWithDebugFaultPolicy =
    kernel::process::StopWithDebugFaultPolicy {};

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 8;

// State for loading and holding applications.
static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

static mut CHIP: Option<&'static nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>> = None;
static mut PROCESS_PRINTER: Option<&'static kernel::process::ProcessPrinterText> = None;
static mut CDC_REF_FOR_PANIC: Option<
    &'static capsules_extra::usb::cdc::CdcAcm<
        'static,
        nrf52::usbd::Usbd,
        capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, nrf52::rtc::Rtc>,
    >,
> = None;
static mut NRF52_POWER: Option<&'static nrf52840::power::Power> = None;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

// Function for the CDC/USB stack to use to enter the bootloader.
fn baud_rate_reset_bootloader_enter() {
    unsafe {
        // 0x90 is the magic value the bootloader expects
        NRF52_POWER.unwrap().set_gpregret(0x90);
        cortexm4::scb::reset();
    }
}

fn crc(s: &'static str) -> u32 {
    kernel::utilities::helpers::crc32_posix(s.as_bytes())
}

//------------------------------------------------------------------------------
// SYSCALL DRIVER TYPE DEFINITIONS
//------------------------------------------------------------------------------

type AlarmDriver = components::alarm::AlarmDriverComponentType<nrf52840::rtc::Rtc<'static>>;
type AdcDriver = components::adc::AdcDedicatedComponentType<nrf52840::adc::Adc<'static>>;
type ButtonDriver = components::button::ButtonComponentType<nrf52840::gpio::GPIOPin<'static>>;
type GpioDriver = components::gpio::GpioComponentType<nrf52840::gpio::GPIOPin<'static>>;
type Led = kernel::hil::led::LedLow<'static, nrf52::gpio::GPIOPin<'static>>;
type LedDriver = components::led::LedsComponentType<Led, 1>;
type RngDriver = components::rng::RngComponentType<nrf52840::trng::Trng<'static>>;

type Screen = components::ssd1306::Ssd1306ComponentType<nrf52840::i2c::TWI<'static>>;
type ScreenDriver = components::screen::ScreenSharedComponentType<Screen>;

type Ieee802154MacDevice = components::ieee802154::Ieee802154ComponentMacDeviceType<
    nrf52840::ieee802154_radio::Radio<'static>,
    nrf52840::aes::AesECB<'static>,
>;
type Ieee802154Driver = components::ieee802154::Ieee802154ComponentType<
    nrf52840::ieee802154_radio::Radio<'static>,
    nrf52840::aes::AesECB<'static>,
>;

/// Supported drivers by the platform
pub struct Platform {
    ble_radio: &'static capsules_extra::ble_advertising_driver::BLE<
        'static,
        nrf52::ble_radio::Radio<'static>,
        capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<
            'static,
            nrf52::rtc::Rtc<'static>,
        >,
    >,
    ieee802154_radio: &'static Ieee802154Driver,
    console: &'static capsules_core::console::Console<'static>,
    alarm: &'static AlarmDriver,
    adc: &'static AdcDriver,
    gpio: &'static GpioDriver,
    led: &'static LedDriver,
    rng: &'static RngDriver,
    button: &'static ButtonDriver,
    screen: &'static ScreenDriver,
    udp_driver: &'static capsules_extra::net::udp::UDPDriver<'static>,
    ipc: kernel::ipc::IPC<{ NUM_PROCS as u8 }>,
    scheduler: &'static RoundRobinSched<'static>,
    syscall_filter: &'static policy::SoilMoistureSyscallFilter,
    systick: cortexm4::systick::SysTick,
}

impl SyscallDriverLookup for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules_core::console::DRIVER_NUM => f(Some(self.console)),
            capsules_core::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules_core::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules_core::led::DRIVER_NUM => f(Some(self.led)),
            capsules_core::button::DRIVER_NUM => f(Some(self.button)),
            capsules_core::adc::DRIVER_NUM => f(Some(self.adc)),
            capsules_core::rng::DRIVER_NUM => f(Some(self.rng)),
            capsules_extra::screen::DRIVER_NUM => f(Some(self.screen)),
            capsules_extra::ble_advertising_driver::DRIVER_NUM => f(Some(self.ble_radio)),
            capsules_extra::ieee802154::DRIVER_NUM => f(Some(self.ieee802154_radio)),
            capsules_extra::net::udp::DRIVER_NUM => f(Some(self.udp_driver)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}

impl KernelResources<nrf52::chip::NRF52<'static, Nrf52840DefaultPeripherals<'static>>>
    for Platform
{
    type SyscallDriverLookup = Self;
    type SyscallFilter = policy::SoilMoistureSyscallFilter;
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm4::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        self.syscall_filter
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
pub unsafe fn start() -> (
    &'static kernel::Kernel,
    Platform,
    &'static nrf52840::chip::NRF52<'static, Nrf52840DefaultPeripherals<'static>>,
) {
    nrf52840::init();

    let ieee802154_ack_buf = static_init!(
        [u8; nrf52840::ieee802154_radio::ACK_BUF_SIZE],
        [0; nrf52840::ieee802154_radio::ACK_BUF_SIZE]
    );

    // Initialize chip peripheral drivers
    let nrf52840_peripherals = static_init!(
        Nrf52840DefaultPeripherals,
        Nrf52840DefaultPeripherals::new(ieee802154_ack_buf)
    );

    // set up circular peripheral dependencies
    nrf52840_peripherals.init();
    let base_peripherals = &nrf52840_peripherals.nrf52;

    // Save a reference to the power module for resetting the board into the
    // bootloader.
    NRF52_POWER = Some(&base_peripherals.pwr_clk);

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    // Do nRF configuration and setup. This is shared code with other nRF-based
    // platforms.
    nrf52_components::startup::NrfStartupComponent::new(
        false,
        BUTTON_RST_PIN,
        nrf52840::uicr::Regulator0Output::DEFAULT,
        &base_peripherals.nvmc,
    )
    .finalize(());

    //--------------------------------------------------------------------------
    // CAPABILITIES
    //--------------------------------------------------------------------------

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);

    //--------------------------------------------------------------------------
    // DEBUG GPIO
    //--------------------------------------------------------------------------

    // Configure kernel debug GPIOs as early as possible. These are used by the
    // `debug_gpio!(0, toggle)` macro. We configure these early so that the
    // macro is available during most of the setup code and kernel execution.
    kernel::debug::assign_gpios(Some(&nrf52840_peripherals.gpio_port[LED_PIN]), None, None);

    //--------------------------------------------------------------------------
    // GPIO
    //--------------------------------------------------------------------------

    let gpio = components::gpio::GpioComponent::new(
        board_kernel,
        capsules_core::gpio::DRIVER_NUM,
        components::gpio_component_helper!(
            nrf52840::gpio::GPIOPin,
            0 => &nrf52840_peripherals.gpio_port[GPIO_SOIL_SENSOR_POWER],
        ),
    )
    .finalize(components::gpio_component_static!(nrf52840::gpio::GPIOPin));

    //--------------------------------------------------------------------------
    // LEDs
    //--------------------------------------------------------------------------

    let led = components::led::LedsComponent::new().finalize(components::led_component_static!(
        Led,
        kernel::hil::led::LedLow::new(&nrf52840_peripherals.gpio_port[LED_PIN]),
    ));

    //--------------------------------------------------------------------------
    // BUTTONS
    //--------------------------------------------------------------------------

    let button = components::button::ButtonComponent::new(
        board_kernel,
        capsules_core::button::DRIVER_NUM,
        components::button_component_helper!(
            nrf52840::gpio::GPIOPin,
            (
                &nrf52840_peripherals.gpio_port[BUTTON_PIN],
                kernel::hil::gpio::ActivationMode::ActiveLow,
                kernel::hil::gpio::FloatingState::PullUp
            )
        ),
    )
    .finalize(components::button_component_static!(
        nrf52840::gpio::GPIOPin
    ));

    //--------------------------------------------------------------------------
    // ALARM & TIMER
    //--------------------------------------------------------------------------

    let rtc = &base_peripherals.rtc;
    let _ = rtc.start();

    let mux_alarm = components::alarm::AlarmMuxComponent::new(rtc)
        .finalize(components::alarm_mux_component_static!(nrf52::rtc::Rtc));
    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules_core::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_static!(nrf52::rtc::Rtc));

    //--------------------------------------------------------------------------
    // UART & CONSOLE & DEBUG
    //--------------------------------------------------------------------------

    // Setup the CDC-ACM over USB driver that we will use for UART.
    // We use the Arduino Vendor ID and Product ID since the device is the same.

    // Create the strings we include in the USB descriptor. We use the hardcoded
    // DEVICEADDR register on the nRF52 to set the serial number.
    let serial_number_buf = static_init!([u8; 17], [0; 17]);
    let serial_number_string: &'static str =
        nrf52::ficr::FICR_INSTANCE.address_str(serial_number_buf);
    let strings = static_init!(
        [&str; 3],
        [
            "MakePython",         // Manufacturer
            "NRF52840 - TockOS",  // Product
            serial_number_string, // Serial number
        ]
    );

    let cdc = components::cdc::CdcAcmComponent::new(
        &nrf52840_peripherals.usbd,
        capsules_extra::usb::cdc::MAX_CTRL_PACKET_SIZE_NRF52840,
        0x2341,
        0x005a,
        strings,
        mux_alarm,
        Some(&baud_rate_reset_bootloader_enter),
    )
    .finalize(components::cdc_acm_component_static!(
        nrf52::usbd::Usbd,
        nrf52::rtc::Rtc
    ));
    CDC_REF_FOR_PANIC = Some(cdc); //for use by panic handler

    // Process Printer for displaying process information.
    let process_printer = components::process_printer::ProcessPrinterTextComponent::new()
        .finalize(components::process_printer_text_component_static!());
    PROCESS_PRINTER = Some(process_printer);

    // Create a shared UART channel for the console and for kernel debug.
    let uart_mux = components::console::UartMuxComponent::new(cdc, 115200)
        .finalize(components::uart_mux_component_static!());

    let pconsole = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        uart_mux,
        mux_alarm,
        process_printer,
        Some(cortexm4::support::reset),
    )
    .finalize(components::process_console_component_static!(
        nrf52::rtc::Rtc<'static>
    ));

    // Setup the console.
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules_core::console::DRIVER_NUM,
        uart_mux,
    )
    .finalize(components::console_component_static!());
    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new(uart_mux)
        .finalize(components::debug_writer_component_static!());

    //--------------------------------------------------------------------------
    // RANDOM NUMBERS
    //--------------------------------------------------------------------------

    let rng = components::rng::RngComponent::new(
        board_kernel,
        capsules_core::rng::DRIVER_NUM,
        &base_peripherals.trng,
    )
    .finalize(components::rng_component_static!(
        nrf52840::trng::Trng<'static>
    ));

    //--------------------------------------------------------------------------
    // ADC
    //--------------------------------------------------------------------------

    base_peripherals.adc.calibrate();

    let adc = components::adc::AdcDedicatedComponent::new(
        &base_peripherals.adc,
        static_init!(
            [nrf52840::adc::AdcChannelSetup; 1],
            [nrf52840::adc::AdcChannelSetup::new(
                nrf52840::adc::AdcChannel::AnalogInput1,
            )]
        ),
        board_kernel,
        capsules_core::adc::DRIVER_NUM,
    )
    .finalize(components::adc_dedicated_component_static!(
        nrf52840::adc::Adc
    ));

    //--------------------------------------------------------------------------
    // SCREEN
    //--------------------------------------------------------------------------

    let i2c_bus = components::i2c::I2CMuxComponent::new(&base_peripherals.twi0, None)
        .finalize(components::i2c_mux_component_static!(nrf52840::i2c::TWI));
    base_peripherals.twi0.configure(
        nrf52840::pinmux::Pinmux::new(I2C_SCL_PIN as u32),
        nrf52840::pinmux::Pinmux::new(I2C_SDA_PIN as u32),
    );

    // I2C address is b011110X, and on this board D/C̅ is GND.
    let ssd1306_i2c = components::i2c::I2CComponent::new(i2c_bus, 0x3c)
        .finalize(components::i2c_component_static!(nrf52840::i2c::TWI));

    // Create the ssd1306 object for the actual screen driver.
    let ssd1306 = components::ssd1306::Ssd1306Component::new(ssd1306_i2c, true)
        .finalize(components::ssd1306_component_static!(nrf52840::i2c::TWI));

    let apps_regions = static_init!(
        [capsules_extra::screen_shared::AppScreenRegion; 2],
        [
            capsules_extra::screen_shared::AppScreenRegion::new(
                kernel::process::ShortId::Fixed(
                    core::num::NonZeroU32::new(crc("soil_moisture_instr")).unwrap()
                ),
                0,      // x
                0,      // y
                16 * 8, // width
                6 * 8   // height
            ),
            capsules_extra::screen_shared::AppScreenRegion::new(
                kernel::process::ShortId::Fixed(
                    core::num::NonZeroU32::new(crc("soil_moisture_data")).unwrap()
                ),
                0,      // x
                6 * 8,  // y
                16 * 8, // width
                2 * 8   // height
            )
        ]
    );

    let screen = components::screen::ScreenSharedComponent::new(
        board_kernel,
        capsules_extra::screen::DRIVER_NUM,
        ssd1306,
        apps_regions,
    )
    .finalize(components::screen_shared_component_static!(1032, Screen));

    //--------------------------------------------------------------------------
    // WIRELESS
    //--------------------------------------------------------------------------

    let ble_radio = components::ble::BLEComponent::new(
        board_kernel,
        capsules_extra::ble_advertising_driver::DRIVER_NUM,
        &base_peripherals.ble_radio,
        mux_alarm,
    )
    .finalize(components::ble_component_static!(
        nrf52840::rtc::Rtc,
        nrf52840::ble_radio::Radio
    ));

    use capsules_extra::net::ieee802154::MacAddress;

    let aes_mux = components::ieee802154::MuxAes128ccmComponent::new(&base_peripherals.ecb)
        .finalize(components::mux_aes128ccm_component_static!(
            nrf52840::aes::AesECB
        ));

    let device_id = nrf52840::ficr::FICR_INSTANCE.id();
    let device_id_bottom_16 = u16::from_le_bytes([device_id[0], device_id[1]]);
    let (ieee802154_radio, mux_mac) = components::ieee802154::Ieee802154Component::new(
        board_kernel,
        capsules_extra::ieee802154::DRIVER_NUM,
        &nrf52840_peripherals.ieee802154_radio,
        aes_mux,
        PAN_ID,
        device_id_bottom_16,
        device_id,
    )
    .finalize(components::ieee802154_component_static!(
        nrf52840::ieee802154_radio::Radio,
        nrf52840::aes::AesECB<'static>
    ));
    use capsules_extra::net::ipv6::ip_utils::IPAddr;

    let local_ip_ifaces = static_init!(
        [IPAddr; 3],
        [
            IPAddr([
                0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
                0x0e, 0x0f,
            ]),
            IPAddr([
                0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
                0x1e, 0x1f,
            ]),
            IPAddr::generate_from_mac(capsules_extra::net::ieee802154::MacAddress::Short(
                device_id_bottom_16
            )),
        ]
    );

    let (udp_send_mux, udp_recv_mux, udp_port_table) = components::udp_mux::UDPMuxComponent::new(
        mux_mac,
        DEFAULT_CTX_PREFIX_LEN,
        DEFAULT_CTX_PREFIX,
        DST_MAC_ADDR,
        MacAddress::Short(device_id_bottom_16),
        local_ip_ifaces,
        mux_alarm,
    )
    .finalize(components::udp_mux_component_static!(
        nrf52840::rtc::Rtc,
        Ieee802154MacDevice
    ));

    // UDP driver initialization happens here
    let udp_driver = components::udp_driver::UDPDriverComponent::new(
        board_kernel,
        capsules_extra::net::udp::DRIVER_NUM,
        udp_send_mux,
        udp_recv_mux,
        udp_port_table,
        local_ip_ifaces,
    )
    .finalize(components::udp_driver_component_static!(nrf52840::rtc::Rtc));

    //--------------------------------------------------------------------------
    // CHIP
    //--------------------------------------------------------------------------

    let chip = static_init!(
        nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>,
        nrf52840::chip::NRF52::new(nrf52840_peripherals)
    );
    CHIP = Some(chip);

    //--------------------------------------------------------------------------
    // SYSCALL FILTERING
    //--------------------------------------------------------------------------

    let syscall_filter = static_init!(
        policy::SoilMoistureSyscallFilter,
        policy::SoilMoistureSyscallFilter {}
    );

    //--------------------------------------------------------------------------
    // APP ID CHECKING
    //--------------------------------------------------------------------------

    // Create the software-based SHA engine.
    let sha = components::sha::ShaSoftware256Component::new()
        .finalize(components::sha_software_256_component_static!());

    // Create the credential checker.
    let checking_policy = components::appid::checker_sha::AppCheckerSha256Component::new(sha)
        .finalize(components::app_checker_sha256_component_static!());

    // Create the AppID assigner.
    let assigner = components::appid::assigner_name::AppIdAssignerNamesComponent::new()
        .finalize(components::appid_assigner_names_component_static!());

    // Create the process checking machine.
    let checker = components::appid::checker::ProcessCheckerMachineComponent::new(checking_policy)
        .finalize(components::process_checker_machine_component_static!());

    // These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    let process_binary_array = static_init!(
        [Option<kernel::process::ProcessBinary>; NUM_PROCS],
        [None, None, None, None, None, None, None, None]
    );

    let loader = static_init!(
        kernel::process::SequentialProcessLoaderMachine<
            nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>,
        >,
        kernel::process::SequentialProcessLoaderMachine::new(
            checker,
            &mut PROCESSES,
            process_binary_array,
            board_kernel,
            chip,
            core::slice::from_raw_parts(
                &_sapps as *const u8,
                &_eapps as *const u8 as usize - &_sapps as *const u8 as usize,
            ),
            core::slice::from_raw_parts_mut(
                &mut _sappmem as *mut u8,
                &_eappmem as *const u8 as usize - &_sappmem as *const u8 as usize,
            ),
            &FAULT_RESPONSE,
            assigner,
            &process_management_capability
        )
    );

    checker.set_client(loader);

    //--------------------------------------------------------------------------
    // FINAL SETUP AND BOARD BOOT
    //--------------------------------------------------------------------------

    // Start all of the clocks. Low power operation will require a better
    // approach than this.
    nrf52_components::NrfClockComponent::new(&base_peripherals.clock).finalize(());

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&PROCESSES)
        .finalize(components::round_robin_component_static!(NUM_PROCS));

    let platform = Platform {
        console,
        adc,
        led,
        button,
        rng,
        screen,
        alarm,
        gpio,
        ble_radio,
        ieee802154_radio,
        udp_driver,
        ipc: kernel::ipc::IPC::new(
            board_kernel,
            kernel::ipc::DRIVER_NUM,
            &memory_allocation_capability,
        ),
        scheduler,
        syscall_filter,
        systick: cortexm4::systick::SysTick::new_with_calibration(64000000),
    };

    //--------------------------------------------------------------------------
    // INIT
    //--------------------------------------------------------------------------

    // Process loader.
    loader.start();

    // Configure the USB stack to enable a serial port over CDC-ACM.
    cdc.enable();
    cdc.attach();

    debug!("Initialization complete. Entering main loop.");
    let _ = pconsole.start();

    ssd1306.init_screen();

    (board_kernel, platform, chip)
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);

    let (board_kernel, platform, chip) = start();
    board_kernel.kernel_loop(&platform, chip, Some(&platform.ipc), &main_loop_capability);
}
