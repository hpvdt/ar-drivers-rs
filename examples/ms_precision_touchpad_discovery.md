# Microsoft Precision Touchpad - Protocol Discovery

## Captured Messages Analysis

### Device Information
- **Vendor ID**: 0x3175 (Maetay Electronic)
- **Product ID**: 0xA001
- **Product Name**: Brydge 12.3 Pro+
- **HID Collection**: Usage Page 0x0D (Digitizers), Usage 0x05 (Touch Pad)

### Observed Message Format

From 10 captured messages, all reports were 24 bytes with the following pattern:

```
Byte 0:   0x0B     - Report ID
Byte 1-2: 0x9B 0xFF - Appears constant (possibly flags/device-specific)
Byte 3-4: VARIABLE  - Likely X coordinate (little-endian)
Byte 5:   VARIABLE  - Likely Y coordinate or timestamp component
Byte 6-21: 0x00     - Padding/reserved
Byte 22:   0x01     - Possibly contact count or button state
Byte 23:   0x64     - Possibly status or confidence
```

### Example Messages (from actual capture)

```
[1771462465425 ms] len=24 raw=0B 9B FF B4 D2 0D 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465441 ms] len=24 raw=0B 9B FF B4 D2 0D 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465442 ms] len=24 raw=0B 9B FF B4 D2 0D 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465455 ms] len=24 raw=0B 9B FF B3 E2 0D 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465487 ms] len=24 raw=0B 9B FF B3 02 0E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465487 ms] len=24 raw=0B 9B FF B2 22 0E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465502 ms] len=24 raw=0B 9B FF B1 52 0E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465502 ms] len=24 raw=0B 9B FF B0 82 0E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465516 ms] len=24 raw=0B 9B FF AE D2 0E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
[1771462465516 ms] len=24 raw=0B 9B FF AB 22 0F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 64 00
```

### Key Observations

1. **Report Length**: Consistent 24-byte reports
2. **Report ID**: Always `0x0B` for touchpad input reports
3. **Static Bytes**: Bytes 1-2 (`0x9B 0xFF`) remain constant across all samples
4. **Dynamic Data**: Bytes 3-5 change during finger movement (likely coordinates/timing)
5. **Timestamp Pattern**: Byte 5 increments from `0x0D` → `0x0E` → `0x0F` across sequential messages
6. **Contact Count**: Current parser detected 0 contacts (best-effort parser may not match this device's actual format)
7. **Padding**: Bytes 6-21 are all zeros in all captured samples

### Parser Caveats

The current `parse_touchpad_report_best_effort` function uses a generic Microsoft PTP layout assumption:
- Assumes byte 1 contains contact count in lower nibble
- Assumes 6 bytes per contact starting at byte 2

**For the Brydge 12.3 Pro+ device specifically**:
- The actual report format differs from the generic parser assumptions
- Bytes 1-2 appear to be device-specific flags
- Coordinate data may be encoded differently than assumed
- The constant `0x01` at byte 22 and `0x64` at byte 23 suggest a different field layout

## Microsoft Precision Touchpad Documentation References

### Primary Documentation

1. **[Windows Precision Touchpad Collection](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-windows-precision-touchpad-collection)**
   - Defines the top-level HID collection (Page 0x0D, Usage 0x05)
   - Describes device capabilities, certification, and latency mode feature reports
   - Details mandatory input report structure

2. **[Sample Report Descriptors](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-sample-report-descriptors)**
   - Provides complete sample HID report descriptors
   - Shows parallel, hybrid, and single-finger reporting modes
   - Includes optional features (haptics, button press threshold, etc.)

3. **[Required HID Top-Level Collections](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-required-hid-top-level-collections)**
   - Lists mandatory collections: Windows Precision Touchpad + Configuration
   - Optional mouse collection for legacy compatibility
   - Firmware update collection (optional)

4. **[Buttons, Report Level Usages](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-buttons-report-level-usages)**
   - Button 1: Integrated touchpad button (click-pad/pressure-pad)
   - Button 2: External left-click button
   - Button 3: External right-click button
   - Describes packet reporting modes (Parallel, Hybrid, Single-Finger Hybrid)

5. **[Configuration Collection](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-configuration-collection)**
   - Input Mode Feature Report (switch between mouse/touchpad collections)
   - Selective Reporting Feature Report (enable/disable surface/button reporting)
   - Input mode values: 0 = Mouse, 3 = Precision Touchpad

### Standard PTP Input Report Structure

According to Microsoft documentation, a standard PTP input report contains:

**Per-Contact Fields (mandatory)**:
- Contact ID (Page 0x0D, Usage 0x51) - Unique identifier per contact
- X coordinate (Page 0x01, Usage 0x30) - 16-bit, typically 0-4095 range
- Y coordinate (Page 0x01, Usage 0x31) - 16-bit, typically 0-4095 range
- Tip switch (Page 0x0D, Usage 0x42) - 1 bit, indicates contact on surface
- Confidence (Page 0x0D, Usage 0x47) - 1 bit, intentional contact flag

**Per-Contact Fields (optional)**:
- Width (Page 0x0D, Usage 0x48)
- Height (Page 0x0D, Usage 0x49)
- Pressure (Page 0x0D, Usage 0x30)
- Azimuth (Page 0x0D, Usage 0x3F)

**Report-Level Fields (mandatory)**:
- Scan Time (Page 0x0D, Usage 0x56) - Relative time in 100µs units
- Contact Count (Page 0x0D, Usage 0x54) - Number of contacts in report

**Report-Level Fields (optional)**:
- Button 1/2/3 (Page 0x09, Usage 0x01/0x02/0x03)
- Mechanical Force (Page 0x20, Usage 0x494)

### Reporting Modes

1. **Parallel Mode**: All contacts in single packet
2. **Hybrid Mode**: Multiple contacts split across multiple reports
3. **Single-Finger Hybrid**: One contact per report, contact count in first report only

## Implementation Notes

### Current Parser Limitations

The `parse_touchpad_report_best_effort` function in `ms_precision_touchpad.rs`:
- Uses a simplified 6-byte-per-contact assumption
- Attempts to extract contact count from byte 1's lower nibble
- Provides best-effort parsing that may not match all device implementations

### Device-Specific Considerations

For the Brydge 12.3 Pro+ (VID 0x3175, PID 0xA001):
- Report format appears to differ from standard Microsoft PTP sample descriptors
- May use vendor-specific encoding for coordinates and contact data
- Would benefit from HID report descriptor analysis to determine exact field layout

### To Improve Parsing

1. Read the actual HID report descriptor from the device
2. Parse the descriptor to determine exact field sizes and positions
3. Handle vendor-specific usages (Page 0xFF)
4. Support multiple contact reporting modes (parallel/hybrid)

## Log File Structure

Reports are logged to `examples/log/` split by contact count:
- `contacts_0.log` - Reports with 0 contacts
- `contacts_1.log` - Reports with 1 contact
- `contacts_2.log` - Reports with 2 contacts
- `contacts_3.log` - Reports with 3 contacts
- `contacts_4.log` - Reports with 4 contacts
- `contacts_5.log` - Reports with 5 contacts
- `contacts_unparsed.log` - Reports that couldn't be parsed

Each log line format:
```
[<timestamp_ms> ms] len=<length> raw=<hex bytes> | <parsed summary>
```

## Related Documentation Links

- [Windows Precision Touchpad Collection](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-windows-precision-touchpad-collection)
- [Sample Report Descriptors](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-sample-report-descriptors)
- [Required HID Top-Level Collections](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-required-hid-top-level-collections)
- [Buttons, Report Level Usages](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-buttons-report-level-usages)
- [Configuration Collection](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-configuration-collection)
