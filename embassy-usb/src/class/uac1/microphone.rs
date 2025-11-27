//! USB Audio Class 1.0 - Microphone device
//!
//! Provides a class with a single audio streaming interface (device to host),
//! that advertises itself as a microphone. Supports synchronous mode (no feedback endpoint).
//!
//! Various aspects of the audio stream can be configured, for example:
//! - sample rate
//! - sample resolution
//! - audio channel count and assignment
//!
//! The class provides gain and mute controls for each channel.

use core::cell::{Cell, RefCell};
use core::future::{Future, poll_fn};
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::Poll;

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::waitqueue::WakerRegistration;
use heapless::Vec;

use super::class_codes::*;
use super::terminal_type::TerminalType;
use super::{Channel, ChannelConfig, MAX_AUDIO_CHANNEL_COUNT, MAX_AUDIO_CHANNEL_INDEX, SampleWidth};
use crate::control::{self, InResponse, OutResponse, Recipient, Request, RequestType};
use crate::descriptor::{SynchronizationType, UsageType};
use crate::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointType};
use crate::types::InterfaceNumber;
use crate::{Builder, Handler};

/// Maximum allowed sampling rate (3 bytes) in Hz.
const MAX_SAMPLE_RATE_HZ: u32 = 0x7FFFFF;

/// Arbitrary unique identifier for the input unit (microphone source).
const INPUT_UNIT_ID: u8 = 0x01;

/// Arbitrary unique identifier for the feature unit.
const FEATURE_UNIT_ID: u8 = 0x02;

/// Arbitrary unique identifier for the output unit (USB streaming).
const OUTPUT_UNIT_ID: u8 = 0x03;

// Gain settings go from -25600 to 0, in steps of 256.
// Therefore, the gain settings are 8q8 values in units of dB.
const VOLUME_STEPS_PER_DB: i16 = 256;
const MIN_VOLUME_DB: i16 = -100;
const MAX_VOLUME_DB: i16 = 0;

// Maximum number of supported discrete sample rates.
const MAX_SAMPLE_RATE_COUNT: usize = 10;

/// The volume of an audio channel (used for microphone gain).
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Volume {
    /// The channel is muted.
    Muted,
    /// The channel gain in 8.8 fixed-point format (dB).
    ///
    /// This is a signed 16-bit integer where the value represents dB * 256.
    /// Range: -25600 to 0 (representing -100 dB to 0 dB).
    ///
    /// To get the integer dB value, divide by 256.
    /// Example: -12800 represents -50 dB (-12800 / 256 = -50).
    DeciBel(i16),
}

/// Internal state for the USB Audio Class.
pub struct State<'d> {
    control: Option<Control<'d>>,
    shared: SharedControl<'d>,
}

impl<'d> Default for State<'d> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'d> State<'d> {
    /// Create a new `State`.
    pub fn new() -> Self {
        Self {
            control: None,
            shared: SharedControl::default(),
        }
    }
}

/// Implementation of the USB audio class 1.0 microphone.
pub struct Microphone<'d, D: Driver<'d>> {
    phantom: PhantomData<&'d D>,
}

impl<'d, D: Driver<'d>> Microphone<'d, D> {
    /// Creates a new [`Microphone`] device, split into a stream and a control change notifier.
    ///
    /// The packet size should be chosen, based on the expected transfer size of samples per (micro)frame.
    /// For example, a mono stream at 16 bit resolution and 48 kHz sample rate yields packets of 96 bytes for
    /// full-speed USB (1 ms frame interval) or 12 bytes for high-speed USB (125 us microframe interval).
    ///
    /// This implementation uses synchronous mode (device generates samples synchronized to USB SOF).
    /// No feedback endpoint is required for this mode.
    ///
    /// # Arguments
    ///
    /// * `builder` - The builder for the class.
    /// * `state` - The internal state of the class.
    /// * `max_packet_size` - The maximum packet size per (micro)frame.
    /// * `resolution` - The audio sample resolution.
    /// * `sample_rates_hz` - The supported sample rates in Hz.
    /// * `channels` - The advertised audio channels (up to 12). Entries must be unique, or this function panics.
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State<'d>,
        max_packet_size: u16,
        resolution: SampleWidth,
        sample_rates_hz: &[u32],
        channels: &'d [Channel],
    ) -> (Stream<'d, D>, ControlMonitor<'d>) {
        // The class and subclass fields of the IAD aren't required to match the class and subclass fields of
        // the interfaces in the interface collection that the IAD describes. Microsoft recommends that
        // the first interface of the collection has class and subclass fields that match the class and
        // subclass fields of the IAD.
        let mut func = builder.function(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE);

        // Audio control interface (mandatory) [UAC 4.3.1]
        let mut interface = func.interface();
        let control_interface = interface.interface_number().into();
        let streaming_interface = u8::from(control_interface) + 1;
        let mut alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE, None);

        // Terminal topology for microphone (REVERSED from speaker):
        // Microphone (Input Terminal) -> Feature Unit (mute and gain) -> USB Streaming (Output Terminal)

        // =======================================
        // Input Terminal Descriptor [UAC 3.3.2.1]
        // Microphone input (physical source)
        let terminal_type: u16 = TerminalType::InMicrophone.into();

        // Assemble channel configuration field
        let mut channel_config: u16 = ChannelConfig::None.into();
        for channel in channels {
            let channel: u16 = channel.get_channel_config().into();

            if channel_config & channel != 0 {
                panic!("Invalid channel config, duplicate channel {}.", channel);
            }
            channel_config |= channel;
        }

        let input_terminal_descriptor = [
            INPUT_TERMINAL, // bDescriptorSubtype
            INPUT_UNIT_ID,  // bTerminalID
            terminal_type as u8,
            (terminal_type >> 8) as u8, // wTerminalType (InMicrophone = 0x0201)
            0x00,                       // bAssocTerminal (none)
            channels.len() as u8,       // bNrChannels
            channel_config as u8,
            (channel_config >> 8) as u8, // wChannelConfig
            0x00,                        // iChannelNames (none)
            0x00,                        // iTerminal (none)
        ];

        // ========================================
        // Output Terminal Descriptor [UAC 4.3.2.2]
        // USB Streaming output (to host)
        let terminal_type: u16 = TerminalType::UsbStreaming.into();
        let output_terminal_descriptor = [
            OUTPUT_TERMINAL, // bDescriptorSubtype
            OUTPUT_UNIT_ID,  // bTerminalID
            terminal_type as u8,
            (terminal_type >> 8) as u8, // wTerminalType (UsbStreaming = 0x0101)
            0x00,                       // bAssocTerminal (none)
            FEATURE_UNIT_ID,            // bSourceID (the feature unit)
            0x00,                       // iTerminal (none)
        ];

        // =====================================
        // Feature Unit Descriptor [UAC 4.3.2.5]
        // Mute and gain control
        let controls = MUTE_CONTROL | VOLUME_CONTROL;

        const FEATURE_UNIT_DESCRIPTOR_SIZE: usize = 5;
        let mut feature_unit_descriptor: Vec<u8, { FEATURE_UNIT_DESCRIPTOR_SIZE + MAX_AUDIO_CHANNEL_COUNT + 1 }> =
            Vec::from_slice(&[
                FEATURE_UNIT,         // bDescriptorSubtype (Feature Unit)
                FEATURE_UNIT_ID,      // bUnitID
                INPUT_UNIT_ID,        // bSourceID (connects to microphone input terminal)
                1,                    // bControlSize (one byte per control)
                FU_CONTROL_UNDEFINED, // Master controls (disabled, use only per-channel control)
            ])
            .unwrap();

        // Add per-channel controls
        for _channel in channels {
            feature_unit_descriptor.push(controls).unwrap();
        }
        feature_unit_descriptor.push(0x00).unwrap(); // iFeature (none)

        // ===============================================
        // Format descriptor [UAC 4.5.3]
        // Used later, for operational streaming interface
        let mut format_descriptor: Vec<u8, { 6 + 3 * MAX_SAMPLE_RATE_COUNT }> = Vec::from_slice(&[
            FORMAT_TYPE,               // bDescriptorSubtype
            FORMAT_TYPE_I,             // bFormatType
            channels.len() as u8,      // bNrChannels
            resolution as u8,          // bSubframeSize
            resolution.in_bit() as u8, // bBitResolution
        ])
        .unwrap();

        format_descriptor.push(sample_rates_hz.len() as u8).unwrap();

        for sample_rate_hz in sample_rates_hz {
            assert!(*sample_rate_hz <= MAX_SAMPLE_RATE_HZ);
            format_descriptor.push((sample_rate_hz & 0xFF) as u8).unwrap();
            format_descriptor.push(((sample_rate_hz >> 8) & 0xFF) as u8).unwrap();
            format_descriptor.push(((sample_rate_hz >> 16) & 0xFF) as u8).unwrap();
        }

        // ==================================================
        // Class-specific AC Interface Descriptor [UAC 4.3.2]
        const DESCRIPTOR_HEADER_SIZE: usize = 2;
        const INTERFACE_DESCRIPTOR_SIZE: usize = 7;

        let mut total_descriptor_length = 0;

        for size in [
            INTERFACE_DESCRIPTOR_SIZE,
            input_terminal_descriptor.len(),
            feature_unit_descriptor.len(),
            output_terminal_descriptor.len(),
        ] {
            total_descriptor_length += size + DESCRIPTOR_HEADER_SIZE;
        }

        let interface_descriptor: [u8; INTERFACE_DESCRIPTOR_SIZE] = [
            HEADER_SUBTYPE, // bDescriptorSubtype (Header)
            ADC_VERSION as u8,
            (ADC_VERSION >> 8) as u8, // bcdADC
            total_descriptor_length as u8,
            (total_descriptor_length >> 8) as u8, // wTotalLength
            0x01,                                 // bInCollection (1 streaming interface)
            streaming_interface,                  // baInterfaceNr
        ];

        alt.descriptor(CS_INTERFACE, &interface_descriptor);
        alt.descriptor(CS_INTERFACE, &input_terminal_descriptor);
        alt.descriptor(CS_INTERFACE, &feature_unit_descriptor);
        alt.descriptor(CS_INTERFACE, &output_terminal_descriptor);

        // =====================================================
        // Audio streaming interface, zero-bandwidth [UAC 4.5.1]
        let mut interface = func.interface();
        let alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOSTREAMING_SUBCLASS, PROTOCOL_NONE, None);
        drop(alt);

        // ==================================================
        // Audio streaming interface, operational [UAC 4.5.1]
        let mut alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOSTREAMING_SUBCLASS, PROTOCOL_NONE, None);

        // AS General descriptor links to OUTPUT terminal (USB Streaming)
        alt.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,     // bDescriptorSubtype
                OUTPUT_UNIT_ID, // bTerminalLink (links to USB streaming OUTPUT terminal)
                0x00,           // bDelay (none)
                PCM as u8,
                (PCM >> 8) as u8, // wFormatTag (PCM format)
            ],
        );

        alt.descriptor(CS_INTERFACE, &format_descriptor);

        // Allocate IN endpoint for streaming (device to host)
        let streaming_endpoint = alt.alloc_endpoint_in(EndpointType::Isochronous, None, max_packet_size, 1);

        // Write the descriptor for the streaming endpoint (synchronous mode, no feedback)
        alt.endpoint_descriptor(
            streaming_endpoint.info(),
            SynchronizationType::Synchronous,
            UsageType::DataEndpoint,
            &[
                0x00, // bRefresh (0)
                0x00, // bSynchAddress (none - synchronous mode)
            ],
        );

        alt.descriptor(
            CS_ENDPOINT,
            &[
                EP_GENERAL,            // bDescriptorSubtype (General)
                SAMPLING_FREQ_CONTROL, // bmAttributes (support sampling frequency control)
                0x02,                  // bLockDelayUnits (PCM)
                0x0000 as u8,
                (0x0000 >> 8) as u8, // wLockDelay (0)
            ],
        );

        // Free up the builder.
        drop(func);

        // Store channel information
        state.shared.channels = channels;

        state.control = Some(Control {
            shared: &state.shared,
            streaming_endpoint_address: streaming_endpoint.info().addr.into(),
            control_interface_number: control_interface,
        });

        builder.handler(state.control.as_mut().unwrap());

        let control = &state.shared;

        (Stream { streaming_endpoint }, ControlMonitor { shared: control })
    }
}

/// Audio settings for the feature unit.
///
/// Contains gain and mute control.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AudioSettings {
    /// Channel mute states.
    muted: [bool; MAX_AUDIO_CHANNEL_COUNT],
    /// Channel gain levels in 8.8 format (in dB).
    volume_8q8_db: [i16; MAX_AUDIO_CHANNEL_COUNT],
}

impl Default for AudioSettings {
    fn default() -> Self {
        AudioSettings {
            muted: [false; MAX_AUDIO_CHANNEL_COUNT],
            volume_8q8_db: [MAX_VOLUME_DB * VOLUME_STEPS_PER_DB; MAX_AUDIO_CHANNEL_COUNT],
        }
    }
}

struct Control<'d> {
    control_interface_number: InterfaceNumber,
    streaming_endpoint_address: u8,
    shared: &'d SharedControl<'d>,
}

/// Shared data between [`Control`] and the [`Microphone`] class.
struct SharedControl<'d> {
    /// The collection of audio settings (gain, mute states).
    audio_settings: CriticalSectionMutex<Cell<AudioSettings>>,

    /// Channel assignments.
    channels: &'d [Channel],

    /// The audio sample rate in Hz.
    sample_rate_hz: AtomicU32,

    // Notification mechanism.
    waker: RefCell<WakerRegistration>,
    changed: AtomicBool,
}

impl<'d> Default for SharedControl<'d> {
    fn default() -> Self {
        SharedControl {
            audio_settings: CriticalSectionMutex::new(Cell::new(AudioSettings::default())),
            channels: &[],
            sample_rate_hz: AtomicU32::new(0),
            waker: RefCell::new(WakerRegistration::new()),
            changed: AtomicBool::new(false),
        }
    }
}

impl<'d> SharedControl<'d> {
    fn changed(&self) -> impl Future<Output = ()> + '_ {
        poll_fn(|context| {
            if self.changed.load(Ordering::Relaxed) {
                self.changed.store(false, Ordering::Relaxed);
                Poll::Ready(())
            } else {
                self.waker.borrow_mut().register(context.waker());
                Poll::Pending
            }
        })
    }
}

/// Used for writing audio frames to the host.
pub struct Stream<'d, D: Driver<'d>> {
    streaming_endpoint: D::EndpointIn,
}

impl<'d, D: Driver<'d>> Stream<'d, D> {
    /// Writes a single packet to the IN endpoint
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.streaming_endpoint.write(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.streaming_endpoint.wait_enabled().await;
    }
}

/// Control status change monitor
///
/// Await [`ControlMonitor::changed`] for being notified of configuration changes. Afterwards, the updated
/// configuration settings can be read with [`ControlMonitor::gain`] and [`ControlMonitor::sample_rate_hz`].
pub struct ControlMonitor<'d> {
    shared: &'d SharedControl<'d>,
}

impl<'d> ControlMonitor<'d> {
    fn audio_settings(&self) -> AudioSettings {
        let audio_settings = self.shared.audio_settings.lock(|x| x.get());

        audio_settings
    }

    fn get_logical_channel(&self, search_channel: Channel) -> Option<usize> {
        let index = self.shared.channels.iter().position(|&c| c == search_channel)?;

        // The logical channels start at one (zero is the master channel).
        Some(index + 1)
    }

    /// Get the gain of a selected channel.
    pub fn gain(&self, channel: Channel) -> Option<Volume> {
        let channel_index = self.get_logical_channel(channel)?;

        if self.audio_settings().muted[channel_index] {
            return Some(Volume::Muted);
        }

        Some(Volume::DeciBel(
            self.audio_settings().volume_8q8_db[channel_index]
        ))
    }

    /// Get the streaming endpoint's sample rate in Hz.
    pub fn sample_rate_hz(&self) -> u32 {
        self.shared.sample_rate_hz.load(Ordering::Relaxed)
    }

    /// Return a future for when the control settings change.
    pub async fn changed(&self) {
        self.shared.changed().await;
    }
}

impl<'d> Control<'d> {
    fn changed(&mut self) {
        self.shared.changed.store(true, Ordering::Relaxed);
        self.shared.waker.borrow_mut().wake();
    }

    fn interface_set_mute_state(
        &mut self,
        audio_settings: &mut AudioSettings,
        channel_index: u8,
        data: &[u8],
    ) -> OutResponse {
        let mute_state = data[0] != 0;

        match channel_index as usize {
            ..=MAX_AUDIO_CHANNEL_INDEX => {
                audio_settings.muted[channel_index as usize] = mute_state;
            }
            _ => {
                debug!("Failed to set channel {} mute state: {}", channel_index, mute_state);
                return OutResponse::Rejected;
            }
        }

        debug!("Set channel {} mute state: {}", channel_index, mute_state);
        OutResponse::Accepted
    }

    fn interface_set_volume(
        &mut self,
        audio_settings: &mut AudioSettings,
        channel_index: u8,
        data: &[u8],
    ) -> OutResponse {
        let volume = i16::from_ne_bytes(data[..2].try_into().expect("Failed to read volume."));

        match channel_index as usize {
            ..=MAX_AUDIO_CHANNEL_INDEX => {
                audio_settings.volume_8q8_db[channel_index as usize] = volume;
            }
            _ => {
                debug!("Failed to set channel {} gain: {}", channel_index, volume);
                return OutResponse::Rejected;
            }
        }

        debug!("Set channel {} gain: {}", channel_index, volume);
        OutResponse::Accepted
    }

    fn interface_set_request(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        let interface_number = req.index as u8;
        let entity_index = (req.index >> 8) as u8;
        let channel_index = req.value as u8;
        let control_unit = (req.value >> 8) as u8;

        if interface_number != self.control_interface_number.into() {
            debug!("Unhandled interface set request for interface {}", interface_number);
            return None;
        }

        if entity_index != FEATURE_UNIT_ID {
            debug!("Unsupported interface set request for entity {}", entity_index);
            return Some(OutResponse::Rejected);
        }

        // Handle SET_CUR (set current value) and SET_RES (set resolution)
        // For SET_RES, we just acknowledge it but don't need to process the data
        if req.request != SET_CUR && req.request != SET_RES {
            debug!("Unsupported interface set request type {}", req.request);
            return Some(OutResponse::Rejected);
        }

        // If it's SET_RES, just accept it without processing
        if req.request == SET_RES {
            debug!(
                "Accepting SET_RES request for entity {} control {} channel {}",
                entity_index, control_unit, channel_index
            );
            return Some(OutResponse::Accepted);
        }

        let mut audio_settings = self.shared.audio_settings.lock(|x| x.get());
        let response = match control_unit {
            MUTE_CONTROL => self.interface_set_mute_state(&mut audio_settings, channel_index, data),
            VOLUME_CONTROL => self.interface_set_volume(&mut audio_settings, channel_index, data),
            _ => OutResponse::Rejected,
        };

        if response == OutResponse::Rejected {
            return Some(response);
        }

        // Store updated settings
        self.shared.audio_settings.lock(|x| x.set(audio_settings));

        self.changed();

        Some(OutResponse::Accepted)
    }

    fn endpoint_set_request(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        let control_selector = (req.value >> 8) as u8;
        let endpoint_address = req.index as u8;

        if endpoint_address != self.streaming_endpoint_address {
            debug!(
                "Unhandled endpoint set request for endpoint {} and control {} with data {:?}",
                endpoint_address, control_selector, data
            );
            return None;
        }

        if control_selector != SAMPLING_FREQ_CONTROL {
            debug!(
                "Unsupported endpoint set request for control selector {}",
                control_selector
            );
            return Some(OutResponse::Rejected);
        }

        let sample_rate_hz: u32 = (data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16;
        self.shared.sample_rate_hz.store(sample_rate_hz, Ordering::Relaxed);

        debug!("Set endpoint {} sample rate to {} Hz", endpoint_address, sample_rate_hz);

        self.changed();

        Some(OutResponse::Accepted)
    }

    fn interface_get_request<'r>(&'r mut self, req: Request, buf: &'r mut [u8]) -> Option<InResponse<'r>> {
        let interface_number = req.index as u8;
        let entity_index = (req.index >> 8) as u8;
        let channel_index = req.value as u8;
        let control_unit = (req.value >> 8) as u8;

        if interface_number != self.control_interface_number.into() {
            debug!("Unhandled interface get request for interface {}.", interface_number);
            return None;
        }

        if entity_index != FEATURE_UNIT_ID {
            // Only this function unit can be handled at the moment.
            debug!("Unsupported interface get request for entity {}.", entity_index);
            return Some(InResponse::Rejected);
        }

        let audio_settings = self.shared.audio_settings.lock(|x| x.get());

        match req.request {
            GET_CUR => match control_unit {
                VOLUME_CONTROL => {
                    let volume: i16;

                    match channel_index as usize {
                        ..=MAX_AUDIO_CHANNEL_INDEX => volume = audio_settings.volume_8q8_db[channel_index as usize],
                        _ => return Some(InResponse::Rejected),
                    }

                    buf[0] = volume as u8;
                    buf[1] = (volume >> 8) as u8;

                    debug!("Got channel {} gain: {}.", channel_index, volume);
                    return Some(InResponse::Accepted(&buf[..2]));
                }
                MUTE_CONTROL => {
                    let mute_state: bool;

                    match channel_index as usize {
                        ..=MAX_AUDIO_CHANNEL_INDEX => mute_state = audio_settings.muted[channel_index as usize],
                        _ => return Some(InResponse::Rejected),
                    }

                    buf[0] = mute_state.into();
                    debug!("Got channel {} mute state: {}.", channel_index, mute_state);
                    return Some(InResponse::Accepted(&buf[..1]));
                }
                _ => return Some(InResponse::Rejected),
            },
            GET_MIN => match control_unit {
                VOLUME_CONTROL => {
                    let min_volume = MIN_VOLUME_DB * VOLUME_STEPS_PER_DB;
                    buf[0] = min_volume as u8;
                    buf[1] = (min_volume >> 8) as u8;
                    return Some(InResponse::Accepted(&buf[..2]));
                }
                _ => return Some(InResponse::Rejected),
            },
            GET_MAX => match control_unit {
                VOLUME_CONTROL => {
                    let max_volume = MAX_VOLUME_DB * VOLUME_STEPS_PER_DB;
                    buf[0] = max_volume as u8;
                    buf[1] = (max_volume >> 8) as u8;
                    return Some(InResponse::Accepted(&buf[..2]));
                }
                _ => return Some(InResponse::Rejected),
            },
            GET_RES => match control_unit {
                VOLUME_CONTROL => {
                    buf[0] = VOLUME_STEPS_PER_DB as u8;
                    buf[1] = (VOLUME_STEPS_PER_DB >> 8) as u8;
                    return Some(InResponse::Accepted(&buf[..2]));
                }
                _ => return Some(InResponse::Rejected),
            },
            _ => return Some(InResponse::Rejected),
        }
    }

    fn endpoint_get_request<'r>(&'r mut self, req: Request, buf: &'r mut [u8]) -> Option<InResponse<'r>> {
        let control_selector = (req.value >> 8) as u8;
        let endpoint_address = req.index as u8;

        if endpoint_address != self.streaming_endpoint_address {
            debug!("Unhandled endpoint get request for endpoint {}.", endpoint_address);
            return None;
        }

        if control_selector != SAMPLING_FREQ_CONTROL as u8 {
            debug!(
                "Unsupported endpoint get request for control selector {}.",
                control_selector
            );
            return Some(InResponse::Rejected);
        }

        let sample_rate_hz = self.shared.sample_rate_hz.load(Ordering::Relaxed);

        buf[0] = (sample_rate_hz & 0xFF) as u8;
        buf[1] = ((sample_rate_hz >> 8) & 0xFF) as u8;
        buf[2] = ((sample_rate_hz >> 16) & 0xFF) as u8;

        Some(InResponse::Accepted(&buf[..3]))
    }
}

impl<'d> Handler for Control<'d> {
    /// Called when the USB device has been enabled or disabled.
    fn enabled(&mut self, enabled: bool) {
        debug!("USB device enabled: {}", enabled);
    }

    /// Called when the host has set the address of the device to `addr`.
    fn addressed(&mut self, addr: u8) {
        debug!("Host set address to: {}", addr);
    }

    /// Called when the host has enabled or disabled the configuration of the device.
    fn configured(&mut self, configured: bool) {
        debug!("USB device configured: {}", configured);
    }

    /// Called when remote wakeup feature is enabled or disabled.
    fn remote_wakeup_enabled(&mut self, enabled: bool) {
        debug!("USB remote wakeup enabled: {}", enabled);
    }

    /// Called when a "set alternate setting" control request is done on the interface.
    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate_setting: u8) {
        debug!(
            "USB set interface number {} to alt setting {}.",
            iface, alternate_setting
        );
    }

    /// Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self) {
        let shared = self.shared;
        shared.audio_settings.lock(|x| x.set(AudioSettings::default()));

        shared.changed.store(true, Ordering::Relaxed);
        shared.waker.borrow_mut().wake();
    }

    /// Called when the bus has entered or exited the suspend state.
    fn suspended(&mut self, suspended: bool) {
        debug!("USB device suspended: {}", suspended);
    }

    // Handle control set requests.
    fn control_out(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        match req.request_type {
            RequestType::Class => match req.recipient {
                Recipient::Interface => self.interface_set_request(req, data),
                Recipient::Endpoint => self.endpoint_set_request(req, data),
                _ => Some(OutResponse::Rejected),
            },
            _ => None,
        }
    }

    // Handle control get requests.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        match req.request_type {
            RequestType::Class => match req.recipient {
                Recipient::Interface => self.interface_get_request(req, buf),
                Recipient::Endpoint => self.endpoint_get_request(req, buf),
                _ => None,
            },
            _ => None,
        }
    }
}
