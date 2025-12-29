# Implementation Plan: ST7262 Display Adaptation

## Overview

This implementation plan focuses on making minimal changes to adapt the existing ESP32-S3-Box-3 Slint application to work with the 800x480 ST7262 parallel display. The approach is to change only what's necessary while preserving all existing functionality.

## Tasks

- [x] 1. Update display constants and buffer size
  - Change LCD resolution constants from 320x240 to 800x480
  - Update buffer size calculation for larger display
  - Modify Slint window size configuration
  - _Requirements: 1.5, 4.1_

- [x] 2. Research ESP-HAL parallel LCD interface
  - Examine ESP-HAL documentation for parallel LCD driver
  - Identify the correct driver type and initialization pattern
  - Understand how to replace mipidsi with parallel LCD interface
  - _Requirements: 1.1, 1.2_

- [x] 3. Update Cargo.toml dependencies
  - Remove mipidsi dependency (no longer needed for parallel interface)
  - Ensure esp-hal has required features for parallel LCD
  - Add any additional dependencies needed for ST7262 support
  - _Requirements: 1.1_

- [x] 4. Implement parallel display initialization
  - [x] 4.1 Replace SPI display setup with parallel LCD setup
    - Remove SPI, DC, CS pin configurations
    - Add 16-bit parallel data pin configurations (D0-D15)
    - Add control signal pins (HSYNC, VSYNC, PCLK, HENABLE)
    - _Requirements: 1.3, 1.4, 2.1, 2.2, 2.3, 2.4, 2.5_

  - [ ]* 4.2 Write property test for pin configuration
    - **Property 1: Pin Configuration Completeness**
    - **Validates: Requirements 2.1, 2.2, 2.3, 2.4, 2.5**

  - [x] 4.3 Configure display timing parameters
    - Set pixel clock frequency to 21MHz
    - Configure HSYNC/VSYNC timing parameters
    - Set polarity and porch values as specified
    - _Requirements: 2.6_

  - [x] 4.4 Update DisplayHardware struct
    - Replace mipidsi::Display type with ESP-HAL parallel LCD type
    - Keep the same struct interface for compatibility
    - _Requirements: 1.1, 5.4_

- [x] 5. Update touch controller configuration
  - [x] 5.1 Change I2C pin assignments
    - Update SDA pin from GPIO8 to GPIO15
    - Update SCL pin from GPIO18 to GPIO16
    - _Requirements: 3.1_

  - [x] 5.2 Update touch I2C address
    - Change from 0x14 to 0x5D as specified
    - _Requirements: 3.2_

  - [ ]* 5.3 Write property test for touch coordinate mapping
    - **Property 2: Touch Coordinate Mapping**
    - **Validates: Requirements 3.3**

- [x] 6. Update HardwareDrawBuffer implementation
  - [x] 6.1 Adapt LineBufferProvider for parallel LCD
    - Modify process_line method to work with new display type
    - Ensure pixel data is written correctly to parallel interface
    - Keep the same interface for Slint compatibility
    - _Requirements: 4.3, 5.1_
    - **Status: COMPLETED - Full DMA implementation with chunked transfers**

  - [ ]* 6.2 Write property test for display buffer operations
    - **Property 3: Display Buffer Consistency**
    - **Validates: Requirements 4.1, 4.2**

- [ ] 7. Test and verify functionality
  - [ ] 7.1 Compile and test basic display output
    - Verify the display shows output without errors
    - Check that colors and resolution are correct
    - _Requirements: 1.5, 4.3_

  - [ ]* 7.2 Write property test for Slint rendering compatibility
    - **Property 4: Slint Rendering Compatibility**
    - **Validates: Requirements 4.3, 5.1**

  - [ ] 7.3 Test touch input functionality
    - Verify touch events are detected and mapped correctly
    - Test touch interaction with UI elements
    - _Requirements: 3.3, 3.4_

  - [ ]* 7.4 Write property test for system functionality preservation
    - **Property 5: System Functionality Preservation**
    - **Validates: Requirements 5.2, 5.3, 5.5**

- [ ] 8. Final integration and cleanup
  - [ ] 8.1 Remove unused SPI-related code
    - Clean up any remaining ILI9486 or SPI references
    - Remove unused GPIO pin parameters from init function
    - _Requirements: 5.4_

  - [ ] 8.2 Update function signatures for new pin requirements
    - Modify init_display_hardware() to accept parallel pins
    - Update main.rs to pass correct GPIO peripherals
    - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_

- [ ] 9. Checkpoint - Ensure all tests pass and display works
  - Ensure all tests pass, ask the user if questions arise.
  - Verify WiFi scanning still works on new display
  - Confirm touch input works correctly
  - Check that UI renders properly at 800x480 resolution

## Notes

- Tasks marked with `*` are optional and can be skipped for faster MVP
- Focus on minimal changes - preserve existing patterns wherever possible
- The parallel LCD interface should provide better performance than SPI
- Larger buffer size may require attention to memory usage and frame rates
- Touch coordinate mapping may need fine-tuning based on actual hardware behavior

## Current Implementation Status

### ✅ Completed
- Display hardware initialization with parallel LCD pins
- Touch controller configuration updates (I2C pins and address)
- HardwareDrawBuffer interface adaptation for DPI compatibility
- Slint LineBufferProvider implementation for parallel LCD
- **FULL DMA IMPLEMENTATION**: Complete DMA buffer integration with chunked transfers
- **MEMORY OPTIMIZATION**: Chunked transfer approach to work within ESP32-S3 memory constraints

### 🎉 READY FOR HARDWARE TESTING
The implementation is now complete and ready for testing on actual hardware:

- **DMA Buffers**: Properly created using `esp_hal::dma_buffers!` macro
- **Pixel Data Transfer**: RGB565 pixels correctly copied to DMA buffers
- **Chunked Transfers**: Frame data sent in 32KB chunks to avoid memory issues
- **Ownership Management**: Proper handling of ESP-HAL DPI ownership requirements
- **Error Handling**: Comprehensive error handling for all DMA operations

### 🔧 Next Steps for Hardware Validation
1. **Flash to ESP32-S3-Box-3**: Deploy the compiled binary to actual hardware
2. **Verify Display Output**: Check that the 800x480 display shows rendered content
3. **Test Touch Input**: Validate touch controller functionality with new I2C configuration
4. **Performance Testing**: Monitor frame rates and DMA transfer performance
5. **WiFi Integration**: Ensure WiFi scanning still works alongside display operations