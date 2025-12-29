#![no_std]
#![no_main]

extern crate alloc;

mod display;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::String;
use alloc::vec;
use core::panic::PanicInfo;
// Removed log imports - using println! instead

// WiFi imports - simplified
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use esp_hal::rng::Rng;
use esp_wifi::EspWifiController;
use esp_wifi::wifi::{AccessPointInfo, ClientConfiguration, Configuration, WifiController};

// ESP32 HAL imports - only what we need
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

// Display imports
use display::{DISPLAY_COMPONENTS};

// Slint platform imports
use slint::PhysicalPosition;
use slint::platform::{PointerEventButton, WindowEvent};

// Touch controller imports
esp_bootloader_esp_idf::esp_app_desc!();
slint::include_modules!();

// Shared state for WiFi scan results
static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, alloc::vec::Vec<AccessPointInfo>> =
    Mutex::new(alloc::vec::Vec::new());
static WIFI_SCAN_UPDATED: AtomicBool = AtomicBool::new(false);

// Display constants for ST7262 - 800x480 parallel display
const LCD_H_RES: u16 = 800;
const LCD_V_RES: u16 = 480;
const LCD_H_RES_USIZE: usize = 800;
const LCD_V_RES_USIZE: usize = 480;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;

struct EspEmbassyBackend {
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl EspEmbassyBackend {
    fn new(window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>) -> Self {
        Self { window }
    }
}

impl slint::platform::Platform for EspEmbassyBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        embassy_time::Instant::now()
            .duration_since(embassy_time::Instant::from_secs(0))
            .into()
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("PANIC: {}", info);
    loop {}
}

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}



#[esp_hal_embassy::main]
async fn main(spawner: embassy_executor::Spawner) {
    // Initialize peripherals first
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz));

    // Initialize BOTH heap allocators - WiFi first in internal RAM, then PSRAM for GUI
    // Step 1: Initialize internal RAM heap for WiFi (must be first)
    println!("heap first");
    esp_alloc::heap_allocator!(size: 180 * 1024);

    // Step 2: Initialize PSRAM heap for GUI and other data (with error handling)
    println!("Initializing PSRAM allocator...");
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    println!("PSRAM allocator initialized");

    // Initialize logger
    println!("Peripherals initialized");

    println!("Starting Slint ESP32-S3 Workshop");

    // Initialize WiFi directly in main function
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new(peripherals.RNG);

    println!("Initializing WiFi...");
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(timg0.timer0, rng.clone()).expect("Failed to initialize WiFi")
    );
    println!("WiFi controller initialized");

    let (wifi_controller, _interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI)
        .expect("Failed to create WiFi interface");
    println!("WiFi interface created");

    // Initialize embassy timer for task scheduling BEFORE spawning tasks
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
    println!("Embassy timer initialized");

    // Don't initialize the standard display platform - we'll use a custom one

    // Initialize display hardware with parallel LCD pins
    display::init_display_hardware(
        // Parallel LCD data pins D0-D15 (reordered to match new function signature)
        peripherals.GPIO21,   // D0 (moved from GPIO15 to avoid I2C conflict)
        peripherals.GPIO47,   // D1 (moved from GPIO16 to avoid I2C conflict)
        peripherals.GPIO48,  // D2
        peripherals.GPIO45,  // D3
        peripherals.GPIO38,  // D4
        peripherals.GPIO9,  // D5
        peripherals.GPIO10,  // D6
        peripherals.GPIO11,  // D7
        peripherals.GPIO12,  // D8
        peripherals.GPIO13,  // D9
        peripherals.GPIO14,  // D10
        peripherals.GPIO7,  // D11
        peripherals.GPIO17,   // D12
        peripherals.GPIO18,   // D13
        peripherals.GPIO3,   // D14
        peripherals.GPIO46,   // D15
        // Parallel LCD control pins
        peripherals.GPIO39,  // PCLK
        peripherals.GPIO40,  // HSYNC
        peripherals.GPIO41,  // VSYNC
        peripherals.GPIO42,  // HENABLE (DE)
        // Touch I2C pins (updated per requirements)
        peripherals.GPIO15,  // Touch SDA (updated from GPIO5 to GPIO15)
        peripherals.GPIO16,  // Touch SCL (updated from GPIO6 to GPIO16)
        // Peripherals
        peripherals.LCD_CAM,
        peripherals.DMA_CH0,
        peripherals.I2C0,
    )
    .expect("Failed to initialize display hardware");

    // Store WiFi controller for the render loop task
    let wifi_ctrl = wifi_controller;

    // Create custom Slint window and backend
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    window.set_size(slint::PhysicalSize::new(800, 480));

    let backend = Box::new(EspEmbassyBackend::new(window.clone()));
    slint::platform::set_platform(backend).expect("backend already initialized");
    println!("Custom Slint backend initialized");

    // Initial liveness check
    println!("System initialization complete - board is alive and ready");
    // Create the UI
    let ui = MainWindow::new().unwrap();

    // Create empty WiFi network model with some placeholder data
    let placeholder_networks = vec![
        WifiNetwork {
            ssid: "WiFi Scanning...".into(),
        },
        WifiNetwork {
            ssid: "Please wait".into(),
        },
    ];

    let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(placeholder_networks));
    ui.set_wifi_network_model(wifi_model.clone().into());

    // Set up WiFi refresh handler with real WiFi functionality
    ui.on_wifi_refresh(move || {
        println!("WiFi refresh requested - checking for scan results");

        // Check if we have new scan results
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            // Access the scan results
            let scan_results = WIFI_SCAN_RESULTS.try_lock();
            if let Ok(results) = scan_results {
                let mut networks = alloc::vec::Vec::new();

                for ap in results.iter() {
                    networks.push(WifiNetwork {
                        ssid: ap.ssid.as_str().into(),
                    });
                }

                if networks.is_empty() {
                    networks.push(WifiNetwork {
                        ssid: "No networks found".into(),
                    });
                }

                println!("Updated UI with {} real networks", networks.len());
                wifi_model.set_vec(networks);

                // Reset the update flag
                WIFI_SCAN_UPDATED.store(false, Ordering::Relaxed);
            } else {
                println!("Could not access scan results (locked)");
            }
        } else {
            println!("No new scan results available");
        }
    });

    // Trigger initial refresh
    ui.invoke_wifi_refresh();

    // Spawn WiFi scanning task
    println!("Spawning WiFi scan task");
    spawner.spawn(wifi_scan_task(wifi_ctrl)).ok();

    // Spawn graphics rendering task
    println!("Spawning graphics rendering task");
    spawner
        .spawn(graphics_task(window.clone(), ui.as_weak()))
        .ok();

    // Spawn display DMA task (separate from graphics rendering)
    println!("Spawning display DMA task");
    spawner
        .spawn(display_dma_task())
        .ok();

    // === Touch Polling Integration ===
    println!("Starting continuous touch polling and Slint integration...");

    let mut status_counter = 0u32;
    let mut touch_ticker = Ticker::every(Duration::from_millis(16)); // ~60Hz touch polling
    let mut last_touch_state: Option<gt911::Point> = None;
    let mut last_touch_position = slint::LogicalPosition::new(0.0, 0.0);

    loop {
        // Poll touch events if touch controller is available
        DISPLAY_COMPONENTS.with_mut(|display_hardware| {
            if let Ok(touch_data_option) = display_hardware.touch.get_touch(&mut display_hardware.i2c) {
                match (last_touch_state.as_ref(), touch_data_option.as_ref()) {
                    // Touch press event (transition from None to Some)
                    (None, Some(touch_data)) => {
                        let physical_position =
                            PhysicalPosition::new(touch_data.x as i32, touch_data.y as i32);
                        let logical_position =
                            physical_position.to_logical(window.scale_factor());
                        last_touch_position = logical_position;

                        let pointer_event = WindowEvent::PointerPressed {
                            position: logical_position,
                            button: PointerEventButton::Left,
                        };

                        window.dispatch_event(pointer_event);
                        println!(
                            "Touch PRESSED at x={}, y={} (logical: {:.1}, {:.1}, scale_factor={})",
                            touch_data.x,
                            touch_data.y,
                            logical_position.x,
                            logical_position.y,
                            window.scale_factor()
                        );
                    }
                    // Touch release event (transition from Some to None)
                    (Some(_), None) => {
                        // Send PointerReleased at the last known position
                        let pointer_released = WindowEvent::PointerReleased {
                            position: last_touch_position,
                            button: PointerEventButton::Left,
                        };
                        window.dispatch_event(pointer_released);

                        // Also send PointerExited to complete the interaction cycle
                        let pointer_exited = WindowEvent::PointerExited;
                        window.dispatch_event(pointer_exited);

                        println!(
                            "Touch RELEASED at (logical: {:.1}, {:.1}) + EXITED",
                            last_touch_position.x,
                            last_touch_position.y
                        );
                    }
                    // Touch move event (both states are Some but potentially different positions)
                    (Some(old_touch), Some(new_touch)) => {
                        // Only dispatch move event if position actually changed
                        if old_touch.x != new_touch.x || old_touch.y != new_touch.y {
                            let physical_position =
                                PhysicalPosition::new(new_touch.x as i32, new_touch.y as i32);
                            let logical_position =
                                physical_position.to_logical(window.scale_factor());
                            last_touch_position = logical_position;

                            let pointer_event = WindowEvent::PointerMoved {
                                position: logical_position,
                            };

                            window.dispatch_event(pointer_event);
                            println!(
                                "Touch MOVED to x={}, y={} (logical: {:.1}, {:.1}, scale_factor={})",
                                new_touch.x,
                                new_touch.y,
                                logical_position.x,
                                logical_position.y,
                                window.scale_factor()
                            );
                        }
                    }
                    // No state change
                    _ => {}
                }

                last_touch_state = touch_data_option;
            }
        });

        // Status logging (less frequent than touch polling)
        if status_counter % 600 == 0 {
            // Every ~10 seconds at 60Hz
            println!(
                "Main task status check #{} - ESP32-S3-Box-3 alive with touch polling",
                status_counter / 60
            );
        }

        status_counter += 1;
        touch_ticker.next().await;
    }
}

// Graphics rendering task - handles display output only
#[embassy_executor::task]
async fn graphics_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
) {
    println!("=== Graphics rendering task started ====");

    let mut ticker = Ticker::every(Duration::from_millis(16)); // ~60fps
    let mut frame_counter = 0u32;

    println!(
        "Graphics task initialized for {}x{} rendering",
        LCD_H_RES, LCD_V_RES
    );

    loop {
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Check for new WiFi scan results and trigger UI refresh if available
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                println!("Triggered UI refresh for new WiFi scan results");
            }
        }

        // Render the frame to the PSRAM framebuffer
        let rendered = window.draw_if_needed(|renderer| {
            // Access the global display instance to get the framebuffer
            if let Some(()) = DISPLAY_COMPONENTS.with_mut(|display_hardware| {
                if let Some(framebuffer) = display_hardware.get_framebuffer() {
                    // Create a simple line buffer provider that writes to the PSRAM framebuffer
                    struct PSRAMLineBuffer<'a> {
                        framebuffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
                    }
                    
                    impl<'a> slint::platform::software_renderer::LineBufferProvider for PSRAMLineBuffer<'a> {
                        type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
                        
                        fn process_line(
                            &mut self,
                            line: usize,
                            range: core::ops::Range<usize>,
                            render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
                        ) {
                            let line_start = line * LCD_H_RES_USIZE;
                            let buffer_range = (line_start + range.start)..(line_start + range.end);
                            if buffer_range.end <= self.framebuffer.len() {
                                render_fn(&mut self.framebuffer[buffer_range]);
                            }
                        }
                    }
                    
                    let line_buffer = PSRAMLineBuffer { framebuffer };
                    renderer.render_by_line(line_buffer);

                    if frame_counter % 60 == 0 {
                        println!("Frame {} rendered to PSRAM framebuffer", frame_counter);
                    }
                } else {
                    println!("PSRAM framebuffer not available!");
                }
            }) {
                // Successfully rendered
            } else {
                println!("Display not available in graphics task!");
            }
        });

        // If a frame was rendered, log it
        if rendered {
            if frame_counter % 60 == 0 {
                println!(
                    "Frame {} rendered to PSRAM on ESP32-S3-Box-3",
                    frame_counter
                );
            }
        }

        frame_counter = frame_counter.wrapping_add(1);

        // Log periodic status
        if frame_counter % 300 == 0 {
            // Every ~5 seconds at 60fps
            println!(
                "Graphics: Frame {}, ESP32-S3-Box-3 rendering active",
                frame_counter
            );
        }

        ticker.next().await;
    }
}

// WiFi scanning task
#[embassy_executor::task]
async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    println!("=== WiFi scan task started ====");

    // Start WiFi
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: String::new(),
        password: String::new(),
        ..Default::default()
    });

    match wifi_controller.set_configuration(&client_config) {
        Ok(_) => println!("WiFi configuration set successfully"),
        Err(e) => println!("Failed to set WiFi configuration: {:?}", e),
    }

    match wifi_controller.start_async().await {
        Ok(_) => println!("WiFi started successfully!"),
        Err(e) => println!("Failed to start WiFi: {:?}", e),
    }

    // Wait a bit for WiFi to initialize
    embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;

    loop {
        println!("Performing WiFi scan...");

        match wifi_controller.scan_n_async(10).await {
            Ok(results) => {
                println!("Found {} networks:", results.len());
                for (i, ap) in results.iter().enumerate() {
                    println!(
                        "  {}: SSID: {}, Signal: {:?}, Auth: {:?}, Channel: {}",
                        i + 1,
                        ap.ssid.as_str(),
                        ap.signal_strength,
                        ap.auth_method,
                        ap.channel
                    );
                }

                // Store scan results in shared state
                if let Ok(mut scan_results) = WIFI_SCAN_RESULTS.try_lock() {
                    scan_results.clear();
                    scan_results.extend_from_slice(&results);
                    WIFI_SCAN_UPDATED.store(true, Ordering::Relaxed);
                    println!("Stored {} scan results for UI", scan_results.len());
                } else {
                    println!("Could not store scan results (mutex locked)");
                }
            }
            Err(e) => {
                println!("WiFi scan failed: {:?}", e);
            }
        }

        // Wait 10 seconds before next scan
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}

// Display DMA task - handles sending frames to the display
#[embassy_executor::task]
async fn display_dma_task() {
    println!("=== Display DMA task started ====");

    let mut ticker = Ticker::every(Duration::from_millis(16)); // ~60fps
    let mut dma_frame_counter = 0u32;

    loop {
        // Send the current framebuffer to the display via DMA
        if let Some(()) = DISPLAY_COMPONENTS.with_mut(|display_hardware| {
            match display_hardware.send_frame() {
                Ok(()) => {
                    if dma_frame_counter % 60 == 0 {
                        println!("DMA Frame {} sent to parallel LCD successfully", dma_frame_counter);
                    }
                }
                Err(e) => {
                    if dma_frame_counter % 60 == 0 {
                        println!("DMA Frame {} - Display error: {}", dma_frame_counter, e);
                    }
                }
            }
        }) {
            // Successfully processed
        } else {
            println!("Display not available in DMA task!");
        }

        dma_frame_counter = dma_frame_counter.wrapping_add(1);

        // Log periodic status
        if dma_frame_counter % 300 == 0 {
            // Every ~5 seconds at 60fps
            println!(
                "DMA: Frame {}, ESP32-S3-Box-3 display DMA active",
                dma_frame_counter
            );
        }

        ticker.next().await;
    }
}