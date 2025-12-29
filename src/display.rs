// Display implementation for ESP32-S3-Box-3 with ST7262 800x480 parallel display
// Based on ESP-HAL DPI (RGB parallel) interface examples
// Adapted from SPI-based ILI9486 to parallel ST7262

use embedded_hal::delay::DelayNs;
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{Level},
    i2c::master::I2c,
    lcd_cam::{
        LcdCam,
        lcd::{
            ClockMode, Polarity, Phase,
            dpi::{Config as DpiConfig, Dpi, Format, FrameTiming}
        }
    },
    time::Rate,
    dma::{DmaTxBuf},
};
use gt911::Gt911Blocking;
use esp_println::println;

// Display constants for ST7262 - 800x480 parallel display
const LCD_H_RES: u16 = 800;
const LCD_V_RES: u16 = 480;
const LCD_H_RES_USIZE: usize = 800;
const LCD_V_RES_USIZE: usize = 480;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;

// Full frame buffer size for ST7262 800x480 display
const DMA_FRAME_SIZE: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize) * 2; // 800*480*2 = 768KB

// Global storage for display components - using a safer approach
use core::cell::UnsafeCell;

pub struct DisplayComponentsContainer {
    inner: UnsafeCell<Option<DisplayHardware>>,
}

// Safe wrapper for accessing display components
impl DisplayComponentsContainer {
    const fn new() -> Self {
        Self {
            inner: UnsafeCell::new(None),
        }
    }

    fn set(&self, hardware: DisplayHardware) {
        unsafe {
            *self.inner.get() = Some(hardware);
        }
    }

    pub fn with_mut<R>(&self, f: impl FnOnce(&mut DisplayHardware) -> R) -> Option<R> {
        unsafe {
            if let Some(ref mut hardware) = *self.inner.get() {
                Some(f(hardware))
            } else {
                None
            }
        }
    }
}

// SAFETY: This is only safe because we're in a single-threaded embedded environment
unsafe impl Sync for DisplayComponentsContainer {}

pub static DISPLAY_COMPONENTS: DisplayComponentsContainer = DisplayComponentsContainer::new();

pub struct DisplayHardware {
    pub display: Option<Dpi<'static, Blocking>>, // Make it optional so we can take ownership temporarily
    pub touch: Gt911Blocking<I2c<'static, esp_hal::Blocking>>,
    pub i2c: I2c<'static, esp_hal::Blocking>,
    pub dma_buffer: Option<DmaTxBuf>, // Make it optional so we can take ownership temporarily
    pub framebuffer: Option<&'static mut [slint::platform::software_renderer::Rgb565Pixel]>, // PSRAM framebuffer
}

impl DisplayHardware {
    /// Send a frame to the display using DMA transfer
    /// This method sends the complete frame at once, as required by the DPI interface
    pub fn send_frame(&mut self) -> Result<(), &'static str> {
        // Take ownership of both the display and DMA buffer
        let display = self.display.take()
            .ok_or("Display not available")?;
        let mut dma_buffer = self.dma_buffer.take()
            .ok_or("DMA buffer not available")?;
        
        // Set the DMA buffer length to the full frame size
        dma_buffer.set_length(DMA_FRAME_SIZE);
        
        println!("Sending complete frame to parallel LCD: {}x{} pixels ({} bytes)", 
                 LCD_H_RES, LCD_V_RES, DMA_FRAME_SIZE);
        
        // Send the complete frame via DMA to the parallel LCD
        match display.send(false, dma_buffer) {
            Ok(transfer) => {
                // Wait for the transfer to complete
                let (result, display_back, buffer_back) = transfer.wait();
                
                // Restore the display and buffer
                self.display = Some(display_back);
                self.dma_buffer = Some(buffer_back);
                
                match result {
                    Ok(()) => {
                        println!("Complete frame sent successfully to parallel LCD");
                        Ok(())
                    }
                    Err(error) => {
                        println!("DMA transfer completed with error: {:?}", error);
                        Err("DMA transfer error")
                    }
                }
            }
            Err((error, display_back, buffer_back)) => {
                // Restore the display and buffer on error
                self.display = Some(display_back);
                self.dma_buffer = Some(buffer_back);
                println!("Failed to start DMA transfer: {:?}", error);
                Err("Failed to start DMA transfer")
            }
        }
    }

    /// Get mutable access to the framebuffer for rendering
    pub fn get_framebuffer(&mut self) -> Option<&mut [slint::platform::software_renderer::Rgb565Pixel]> {
        self.framebuffer.as_deref_mut()
    }
}

pub fn init_display_hardware(
    // Parallel LCD data pins D0-D15 (matching reference firmware pin assignments)
    gpio21: esp_hal::peripherals::GPIO21<'static>, // D0/B0
    gpio47: esp_hal::peripherals::GPIO47<'static>, // D1/B1
    gpio48: esp_hal::peripherals::GPIO48<'static>, // D2/B2
    gpio45: esp_hal::peripherals::GPIO45<'static>, // D3/B3
    gpio38: esp_hal::peripherals::GPIO38<'static>, // D4/B4
    gpio9: esp_hal::peripherals::GPIO9<'static>,   // D5/G0
    gpio10: esp_hal::peripherals::GPIO10<'static>, // D6/G1
    gpio11: esp_hal::peripherals::GPIO11<'static>, // D7/G2
    gpio12: esp_hal::peripherals::GPIO12<'static>, // D8/G3
    gpio13: esp_hal::peripherals::GPIO13<'static>, // D9/G4
    gpio14: esp_hal::peripherals::GPIO14<'static>, // D10/G5
    gpio7: esp_hal::peripherals::GPIO7<'static>,   // D11/R0
    gpio17: esp_hal::peripherals::GPIO17<'static>, // D12/R1
    gpio18: esp_hal::peripherals::GPIO18<'static>, // D13/R2
    gpio3: esp_hal::peripherals::GPIO3<'static>,   // D14/R3
    gpio46: esp_hal::peripherals::GPIO46<'static>, // D15/R4
    // Parallel LCD control pins
    gpio39: esp_hal::peripherals::GPIO39<'static>, // PCLK
    gpio40: esp_hal::peripherals::GPIO40<'static>, // HSYNC
    gpio41: esp_hal::peripherals::GPIO41<'static>, // VSYNC
    gpio42: esp_hal::peripherals::GPIO42<'static>, // HENABLE (DE)
    // Touch I2C pins
    gpio15: esp_hal::peripherals::GPIO15<'static>, // Touch SDA
    gpio16: esp_hal::peripherals::GPIO16<'static>, // Touch SCL
    // Peripherals
    lcd_cam: esp_hal::peripherals::LCD_CAM<'static>,
    dma_channel: esp_hal::peripherals::DMA_CH0<'static>,
    i2c0: esp_hal::peripherals::I2C0<'static>,
) -> Result<(), &'static str> {
    let mut _delay = Delay::new();

    // Skip touch INT/RESET initialization since they go through PCA9557 I/O expander
    // The reference firmware sets INT to -1, indicating it's not used directly

    // Create DMA buffers for the parallel LCD using PSRAM
    // Following the example approach with PSRAM allocation
    
    // Allocate framebuffer in PSRAM - this will be shared between rendering and DMA
    let framebuffer: &'static mut [slint::platform::software_renderer::Rgb565Pixel] = {
        use alloc::boxed::Box;
        let fb_box: Box<[slint::platform::software_renderer::Rgb565Pixel; LCD_BUFFER_SIZE]> = 
            Box::new([slint::platform::software_renderer::Rgb565Pixel(0); LCD_BUFFER_SIZE]);
        Box::leak(fb_box)
    };
    
    println!("PSRAM framebuffer allocated: {} bytes for {}x{} frame", 
             DMA_FRAME_SIZE, LCD_H_RES, LCD_V_RES);
    
    // Create DMA buffer that uses the same PSRAM memory as the framebuffer
    // This avoids copying data between framebuffer and DMA buffer
    let framebuffer_bytes: &'static mut [u8] = unsafe {
        core::slice::from_raw_parts_mut(
            framebuffer.as_mut_ptr() as *mut u8,
            DMA_FRAME_SIZE
        )
    };
    
    // Verify PSRAM buffer alignment for DMA
    let buf_ptr = framebuffer_bytes.as_ptr() as usize;
    println!("PSRAM buffer allocated at address: 0x{:08X}", buf_ptr);
    println!("PSRAM buffer alignment modulo 64: {}", buf_ptr % 64);
    
    // Create DMA descriptors for the full frame
    use esp_hal::dma::{CHUNK_SIZE, DmaDescriptor, ExternalBurstConfig};
    const MAX_FRAME_BYTES: usize = DMA_FRAME_SIZE;
    const MAX_NUM_DMA_DESC: usize = MAX_FRAME_BYTES.div_ceil(CHUNK_SIZE);
    
    // Allocate DMA descriptors in DMA-capable memory
    let tx_descriptors: &'static mut [DmaDescriptor] = {
        use alloc::boxed::Box;
        let desc_box: Box<[DmaDescriptor; MAX_NUM_DMA_DESC]> = 
            Box::new([DmaDescriptor::EMPTY; MAX_NUM_DMA_DESC]);
        Box::leak(desc_box)
    };
    
    // Create DMA buffer with external burst configuration for PSRAM
    let dma_tx_buf = DmaTxBuf::new_with_config(
        tx_descriptors,
        framebuffer_bytes,
        ExternalBurstConfig::Size64,
    ).map_err(|_| "Failed to create DMA TX buffer with PSRAM config")?;

    println!("DMA buffer created with PSRAM backing and 64-byte burst config");

    // Parallel LCD initialization using ESP-HAL DPI interface
    let lcd_cam = LcdCam::new(lcd_cam);
    
    // Configure display timing parameters for ST7262 800x480 @ 21MHz
    // Using exact parameters from reference firmware
    let config = DpiConfig::default()
        .with_frequency(Rate::from_hz(21_000_000)) // 21MHz pixel clock as specified
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleHigh, // pclk_idle_high = 1
            phase: Phase::ShiftLow,       // Data valid on falling edge
        })
        .with_format(Format {
            enable_2byte_mode: true, // 16-bit RGB565
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: 800,
            horizontal_total_width: 800 + 8 + 4 + 8, // Active + front porch + sync + back porch = 820
            horizontal_blank_front_porch: 8,          // hsync_front_porch = 8

            vertical_active_height: 480,
            vertical_total_height: 480 + 8 + 4 + 8,  // Active + front porch + sync + back porch = 500
            vertical_blank_front_porch: 8,            // vsync_front_porch = 8

            hsync_width: 4,                           // hsync_pulse_width = 4
            vsync_width: 4,                           // vsync_pulse_width = 4

            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)  // vsync_polarity = 0 means active low, so idle high
        .with_hsync_idle_level(Level::High)  // hsync_polarity = 0 means active low, so idle high
        .with_de_idle_level(Level::Low)      // DE (HENABLE) active high (idle low)
        .with_disable_black_region(false);

    // Initialize parallel LCD with pin assignments - pass DMA channel directly
    let display = Dpi::new(lcd_cam.lcd, dma_channel, config)
        .map_err(|_| "Failed to create DPI interface")?
        .with_vsync(gpio41)    // VSYNC
        .with_hsync(gpio40)    // HSYNC  
        .with_de(gpio42)       // HENABLE (DE)
        .with_pclk(gpio39)     // PCLK
        // 16-bit parallel data pins D0-D15 (matching reference firmware)
        .with_data0(gpio21)    // D0/B0
        .with_data1(gpio47)    // D1/B1
        .with_data2(gpio48)    // D2/B2
        .with_data3(gpio45)    // D3/B3
        .with_data4(gpio38)    // D4/B4
        .with_data5(gpio9)     // D5/G0
        .with_data6(gpio10)    // D6/G1
        .with_data7(gpio11)    // D7/G2
        .with_data8(gpio12)    // D8/G3
        .with_data9(gpio13)    // D9/G4
        .with_data10(gpio14)   // D10/G5
        .with_data11(gpio7)    // D11/R0
        .with_data12(gpio17)   // D12/R1
        .with_data13(gpio18)   // D13/R2
        .with_data14(gpio3)    // D14/R3
        .with_data15(gpio46);  // D15/R4

    // Set up the backlight
    //let mut backlight = Output::new(gpio47, Level::Low, OutputConfig::default());
    //backlight.set_high();

    println!("Parallel LCD initialized successfully");

    // I2C initialization for touch and I/O expander
    let mut i2c = I2c::new(
        i2c0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .map_err(|_| "Failed to create I2C")?
    .with_sda(gpio15)   // Touch SDA
    .with_scl(gpio16);  // Touch SCL

    // Try to initialize touch first at the known working address
    println!("Attempting to initialize touch at address 0x5D");
    let mut touch = Gt911Blocking::new(0x5D);

    match touch.init(&mut i2c) {
        Ok(_) => println!("Touch initialized at address 0x5D"),
        Err(e) => {
            println!("Touch initialization failed at address 0x5D: {:?}", e);
            
            // If touch fails, try to reset it via PCA9557 and try again
            println!("Attempting touch reset via PCA9557...");
            
            // Initialize PCA9557 I/O expander for backlight and touch control
            // PCA9557 I2C address confirmed as 0x18
            const PCA9557_ADDR: u8 = 0x18;
            
            // Configure PCA9557: IO1 (backlight) and IO2 (touch reset) as outputs
            let config_reg = [0x03, 0xF9]; // Register 0x03 = Configuration, bits 1,2 as outputs (0xF9 = 11111001)
            i2c.write(PCA9557_ADDR, &config_reg).ok();
            
            // Reset touch controller: pull reset low, then high
            // IO1=1 (backlight on), IO2=0 (reset active low)
            let touch_reset_low = [0x01, 0x02]; // Only backlight on (bit 1), reset low (bit 2 = 0)
            i2c.write(PCA9557_ADDR, &touch_reset_low).ok();
            _delay.delay_ms(10);
            
            // Release reset: IO1=1 (backlight on), IO2=1 (reset released)
            let touch_reset_high = [0x01, 0x06]; // Backlight (bit 1) + reset released (bit 2) = 0x02 + 0x04 = 0x06
            i2c.write(PCA9557_ADDR, &touch_reset_high).ok();
            _delay.delay_ms(100);
            
            // Try touch initialization again
            match touch.init(&mut i2c) {
                Ok(_) => println!("Touch initialized after reset via PCA9557"),
                Err(e2) => {
                    println!("Touch still failed after reset: {:?}", e2);
                    println!("Attempting fallback address 0x14");
                    let touch_fallback = Gt911Blocking::new(0x14);
                    match touch_fallback.init(&mut i2c) {
                        Ok(_) => {
                            println!("Touch initialized at fallback address 0x14");
                            touch = touch_fallback;
                        }
                        Err(e3) => println!("Touch initialization failed at all addresses: {:?}", e3),
                    }
                }
            }
        }
    }

    // Ensure backlight is on via PCA9557 (in case we didn't initialize it above)
    const PCA9557_ADDR: u8 = 0x18;
    let config_reg = [0x03, 0xF9]; // IO1 (backlight) and IO2 (touch reset) as outputs
    i2c.write(PCA9557_ADDR, &config_reg).ok();
    let backlight_on = [0x01, 0x06]; // IO1=1 (backlight), IO2=1 (touch reset released) = 0x02 + 0x04 = 0x06
    match i2c.write(PCA9557_ADDR, &backlight_on) {
        Ok(_) => println!("Backlight enabled via PCA9557 IO1, touch reset released via IO2"),
        Err(_) => println!("Warning: Could not enable backlight via PCA9557"),
    }

    // Store components globally for the platform to use
    DISPLAY_COMPONENTS.set(DisplayHardware {
        display: Some(display),
        touch,
        i2c,
        dma_buffer: Some(dma_tx_buf),
        framebuffer: Some(framebuffer),
    });

    println!("Display hardware initialization complete");
    Ok(())
}

/// Provides a draw buffer for the MinimalSoftwareWindow renderer
/// Note: This struct is currently unused but kept for potential future use
#[allow(dead_code)]
struct DrawBuffer<'a, Display> {
    display: &'a mut Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Dpi<'static, Blocking>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        _line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];
        render_fn(buffer);

        // For parallel LCD, we collect all lines in the buffer and send the entire frame
        // The DPI interface requires sending complete frames via DMA, not line-by-line
        // The actual frame transmission happens after all lines are rendered
    }
}

/// Hardware draw buffer that renders directly to the display
/// 
/// This implementation collects pixel data from Slint and uses the DisplayHardware
/// send_frame method to perform the actual DMA transfer to the parallel LCD.
pub struct HardwareDrawBuffer<'a, Display> {
    pub display: &'a mut Display,
    pub buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<'a, Display> HardwareDrawBuffer<'a, Display> {
    pub fn new(
        display: &'a mut Display,
        buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
    ) -> Self {
        Self { 
            display, 
            buffer,
        }
    }
}

impl HardwareDrawBuffer<'_, Dpi<'static, Blocking>> {
    /// Send the complete frame buffer to the parallel LCD display
    /// This method is now just a placeholder - the actual sending is handled
    /// by the DisplayHardware::send_frame method
    pub fn send_frame_to_display(&mut self) -> Result<(), &'static str> {
        // The actual frame sending is now handled at a higher level
        // in the graphics task using DisplayHardware::send_frame
        Ok(())
    }
}

impl slint::platform::software_renderer::LineBufferProvider
    for &mut HardwareDrawBuffer<'_, Dpi<'static, Blocking>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        _line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];
        render_fn(buffer);

        // For parallel LCD with DPI interface, we collect all lines in the buffer
        // The actual frame transmission will happen when the complete frame is ready
        // This is handled by the rendering system calling us for each line
        
        // Mark that we've processed this line
        // The DPI interface requires sending complete frames via DMA
        // We'll send the frame after all lines are rendered (handled externally)
    }
}
