#![no_std]
#![no_main]

use core::sync::atomic::{AtomicI16, Ordering};
use defmt::{info, panic};
use embassy_executor::Spawner;
use embassy_stm32::{Config, bind_interrupts, peripherals, usb};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::microphone::{self, Microphone, Volume};
use embassy_usb::driver::EndpointError;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
});

// Mono input (microphone)
pub const INPUT_CHANNEL_COUNT: usize = 1;

// This example uses a fixed sample rate of 48 kHz.
pub const SAMPLE_RATE_HZ: u32 = 48_000;

// Use 16 bit samples for microphone input.
pub const SAMPLE_WIDTH: uac1::SampleWidth = uac1::SampleWidth::Width2Byte;
pub const SAMPLE_WIDTH_BIT: usize = SAMPLE_WIDTH.in_bit();
pub const SAMPLE_SIZE: usize = SAMPLE_WIDTH as usize;
pub const SAMPLE_SIZE_PER_S: usize = (SAMPLE_RATE_HZ as usize) * INPUT_CHANNEL_COUNT * SAMPLE_SIZE;

// Size of audio samples per 1 ms - for the full-speed USB frame period of 1 ms.
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_S.div_ceil(1000);

// Select mono audio channel (left front).
pub const AUDIO_CHANNELS: [uac1::Channel; INPUT_CHANNEL_COUNT] = [uac1::Channel::LeftFront];

// USB packet size for microphone (synchronous mode, no margin needed)
pub const USB_MAX_PACKET_SIZE: usize = USB_FRAME_SIZE;
pub const USB_MAX_SAMPLE_COUNT: usize = USB_MAX_PACKET_SIZE / SAMPLE_SIZE;

// The data type that is exchanged via the zero-copy channel (a sample vector).
pub type SampleBlock = [u8; USB_MAX_PACKET_SIZE];

const fn init_square_wave() -> [i16; USB_MAX_SAMPLE_COUNT] {
    let mut arr = [0i16; USB_MAX_SAMPLE_COUNT];
    let half = USB_MAX_SAMPLE_COUNT / 2;
    let mut i = 0;
    while i < half {
        arr[i] = i16::MIN;
        i += 1;
    }
    while i < USB_MAX_SAMPLE_COUNT {
        arr[i] = i16::MAX;
        i += 1;
    }
    arr
}

static SQUARE_WAVE: [i16; USB_MAX_SAMPLE_COUNT] = init_square_wave();

/// Applies volume gain to a sample using shift-based approximation.
///
/// Note: This uses LINEAR scaling which is NOT correct for audio (should be exponential),
/// but provides a simple demonstration. Every -6 dB ≈ halves the amplitude.
///
/// The proper way would be to compute amplitude = 10^(dB/20), but that requires
/// floating point or expensive fixed-point operations.
#[inline]
fn apply_volume_gain(sample: i16, volume_8q8: i16) -> i16 {
    // Convert 8.8 fixed point to integer dB
    let db = volume_8q8 / 256;

    // Calculate right shifts needed (every -6 dB = 1 bit shift)
    // This is a linear approximation: -6dB ≈ 50% amplitude
    let shifts = (-db) / 6;

    if shifts >= 15 {
        0  // Essentially silent
    } else if shifts <= 0 {
        sample  // 0 dB or positive (shouldn't happen with 0 max)
    } else {
        sample >> shifts
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

/// Shared volume state accessible from both tasks
struct VolumeState {
    current_volume_8q8: AtomicI16,
}

static VOLUME_STATE: StaticCell<VolumeState> = StaticCell::new();

/// Handles streaming of audio data to the host.
async fn stream_handler<'d, T: usb::Instance + 'd>(
    stream: &mut microphone::Stream<'d, usb::Driver<'d, T>>,
    receiver: &mut zerocopy_channel::Receiver<'static, NoopRawMutex, SampleBlock>,
) -> Result<(), Disconnected> {
    // Pre-fetch first buffer before entering main loop
    info!("Stream handler starting...");
    let mut usb_data = [0u8; USB_MAX_PACKET_SIZE];
    let mut packet_count = 0u32;

    loop {
        info!("Stream handler: Waiting to receive");
        let samples = receiver.receive().await;
        info!("Stream handler: Copying...");
        usb_data.copy_from_slice(samples);
        // Convert current samples to bytes
        // Release the buffer BEFORE writing, so the generator can start filling the next one
        info!("Stream handler: Freeing buffer...");
        receiver.receive_done();

        info!("Stream handler: Waiting for write #{}", packet_count);

        // info!("Starting write for packet #{}", packet_count);
        // Send audio packet to host
        // let write_start = embassy_time::Instant::now();

        stream.write_packet(&usb_data).await?;

        // let write_duration = write_start.elapsed();
        info!(
            // "Packet #{} write completed in {} us",
            "Stream handler: Completed write #{}",
            packet_count,
            // write_duration.as_micros()
        );

        packet_count += 1;
    }
}

/// Generates audio samples (test tone generator for demonstration).
/// In a real application, this would read from a microphone via I2S/SAI/PDM.
#[embassy_executor::task]
async fn audio_generator_task(
    mut sender: zerocopy_channel::Sender<'static, NoopRawMutex, SampleBlock>,
    volume_state: &'static VolumeState,
) {
    loop {
        info!("Audio generator: Waiting for sender lock to generate samples");
        // Obtain a buffer from the channel
        // This will block when all buffers are full, providing natural back-pressure
        // The USB streaming task (which is synchronized to USB polls) will consume buffers
        let samples = sender.send().await;

        info!("Audio generator: Sender available - generating samples");

        // Get current volume
        let volume_8q8 = volume_state.current_volume_8q8.load(Ordering::Relaxed);

        // Apply gain to square wave and convert to bytes
        let mut scaled_samples = [0i16; USB_MAX_SAMPLE_COUNT];
        for (i, &sample) in SQUARE_WAVE.iter().enumerate() {
            scaled_samples[i] = apply_volume_gain(sample, volume_8q8);
        }

        samples.copy_from_slice(&bytemuck::cast::<[i16; USB_MAX_SAMPLE_COUNT], [u8; USB_MAX_PACKET_SIZE]>(
            scaled_samples,
        ));

        info!("Audio generator: Samples generated with volume {}", volume_8q8);

        sender.send_done();
    }
}

/// Sends audio samples to the host.
#[embassy_executor::task]
async fn usb_streaming_task(
    mut stream: microphone::Stream<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>,
    mut receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, SampleBlock>,
) {
    loop {
        stream.wait_connection().await;
        info!("Streaming: USB Audio connected - microphone streaming active");

        _ = stream_handler(&mut stream, &mut receiver).await;

        info!("Streaming: USB Audio disconnected");
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb_device: embassy_usb::UsbDevice<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>) {
    usb_device.run().await;
}

/// Checks for changes on the control monitor of the class.
///
/// In this case, monitor changes of gain or mute state.
#[embassy_executor::task]
async fn usb_control_task(control_monitor: microphone::ControlMonitor<'static>, volume_state: &'static VolumeState) {
    loop {
        control_monitor.changed().await;

        for channel in AUDIO_CHANNELS {
            match control_monitor.gain(channel).unwrap() {
                Volume::Muted => {
                    info!("Channel {} muted", channel);
                    volume_state.current_volume_8q8.store(-25600, Ordering::Relaxed);
                }
                Volume::DeciBel(vol_8q8) => {
                    let db_int = vol_8q8 / 256;
                    info!("Channel {} gain: {} dB (raw: {})", channel, db_int, vol_8q8);
                    volume_state.current_volume_8q8.store(vol_8q8, Ordering::Relaxed);
                }
            }
        }

        let sample_rate = control_monitor.sample_rate_hz();
        info!("Sample rate: {} Hz", sample_rate);
    }
}

// If you are trying this and your USB device doesn't connect, the most
// common issues are the RCC config and vbus_detection
//
// See https://embassy.dev/book/#_the_usb_examples_are_not_working_on_my_board_is_there_anything_else_i_need_to_configure
// for more information.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("USB Audio Microphone Example for STM32H7");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        // Configure clocks for STM32H7
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2), // 400 MHz
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 MHz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }
    let p = embassy_stm32::init(config);

    // Configure all required buffers in a static way.
    info!("USB packet size is {} bytes", USB_MAX_PACKET_SIZE);

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);

    static BOS_DESCRIPTOR: StaticCell<[u8; 32]> = StaticCell::new();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 32]);

    const CONTROL_BUF_SIZE: usize = 64;
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_SIZE]> = StaticCell::new();
    let control_buf = CONTROL_BUF.init([0; CONTROL_BUF_SIZE]);

    static EP_OUT_BUFFER: StaticCell<[u8; CONTROL_BUF_SIZE + USB_MAX_PACKET_SIZE]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; CONTROL_BUF_SIZE + USB_MAX_PACKET_SIZE]);

    static STATE: StaticCell<microphone::State> = StaticCell::new();
    let state = STATE.init(microphone::State::new());

    // Initialize volume state (start at 0 dB = maximum volume)
    let volume_state = VOLUME_STATE.init(VolumeState {
        current_volume_8q8: AtomicI16::new(0),
    });

    // Create the driver, from the HAL.
    let mut usb_config = usb::Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    usb_config.vbus_detection = false;

    let usb_driver = usb::Driver::new_fs(p.USB_OTG_HS, Irqs, p.PA12, p.PA11, ep_out_buffer, usb_config);

    // Basic USB device configuration
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-audio-microphone example");
    config.serial_number = Some("12345678");

    let mut builder = embassy_usb::Builder::new(
        usb_driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [], // no msos descriptors
        control_buf,
    );

    // Create the UAC1 Microphone class components (synchronous mode)
    let (stream, control_monitor) = Microphone::new(
        &mut builder,
        state,
        USB_MAX_PACKET_SIZE as u16,
        SAMPLE_WIDTH,
        &[SAMPLE_RATE_HZ],
        &AUDIO_CHANNELS,
    );

    // Create the USB device
    let usb_device = builder.build();

    // Establish a zero-copy channel for transferring audio samples between tasks
    static SAMPLE_BLOCKS: StaticCell<[SampleBlock; 2]> = StaticCell::new();
    let sample_blocks = SAMPLE_BLOCKS.init([[0; USB_MAX_PACKET_SIZE], [0; USB_MAX_PACKET_SIZE]]);

    static CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, SampleBlock>> = StaticCell::new();
    let channel = CHANNEL.init(zerocopy_channel::Channel::new(sample_blocks));
    let (sender, receiver) = channel.split();

    info!("Starting USB audio microphone tasks...");

    // Launch USB audio tasks.
    spawner.spawn(usb_control_task(control_monitor, volume_state)).unwrap();
    spawner.spawn(usb_streaming_task(stream, receiver)).unwrap();
    spawner.spawn(usb_task(usb_device)).unwrap();
    spawner.spawn(audio_generator_task(sender, volume_state)).unwrap();

    info!("All tasks spawned - USB microphone ready");
}
