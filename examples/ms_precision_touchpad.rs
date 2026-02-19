// Copyright (C) 2026
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use clap::Parser;
use hidapi::{DeviceInfo, HidApi};
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::Path;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const DIGITIZER_USAGE_PAGE: u16 = 0x0D;
const TOUCHPAD_USAGE: u16 = 0x05;
const MICROSOFT_VID: u16 = 0x045E;
const LOG_DIR: &str = "examples/log";

#[derive(Parser, Debug)]
#[command(about = "Read reports from a Microsoft Precision Touchpad-style HID collection")]
struct CliArgs {
    /// List matching touchpad HID collections and exit
    #[arg(long)]
    list_only: bool,

    /// Print only raw hex reports (skip best-effort parser)
    #[arg(long)]
    raw_only: bool,

    /// Stop after reading this many non-empty reports
    #[arg(long)]
    max_reports: Option<usize>,

    /// HID read timeout in milliseconds
    #[arg(long, default_value_t = 2500000)]
    read_timeout_ms: i32,

    /// Filter selected device by substring in HID path
    #[arg(long)]
    path_contains: Option<String>,
}

#[derive(Debug, Clone, Copy)]
struct FingerContact {
    contact_id: u8,
    x: u16,
    y: u16,
    tip_switch: bool,
}

#[derive(Debug)]
struct TouchpadReport {
    report_id: u8,
    contact_count: u8,
    button: bool,
    contacts: Vec<FingerContact>,
}

fn is_touchpad_collection(device: &DeviceInfo) -> bool {
    device.usage_page() == DIGITIZER_USAGE_PAGE && device.usage() == TOUCHPAD_USAGE
}

fn score_device(device: &DeviceInfo) -> u8 {
    let mut score = 0;
    if device.vendor_id() == MICROSOFT_VID {
        score += 2;
    }
    if device.product_string().is_some() {
        score += 1;
    }
    score
}

fn choose_touchpad_device(devices: &[DeviceInfo], path_filter: Option<&str>) -> Option<DeviceInfo> {
    devices
        .iter()
        .filter(|d| {
            path_filter
                .map(|needle| d.path().to_string_lossy().contains(needle))
                .unwrap_or(true)
        })
        .filter(|d| is_touchpad_collection(d))
        .max_by_key(|d| score_device(d))
        .cloned()
}

fn timestamp_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::from_millis(0))
        .as_millis()
}

fn hex_bytes(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len() * 3);
    for (i, b) in data.iter().enumerate() {
        if i > 0 {
            out.push(' ');
        }
        out.push_str(&format!("{:02X}", b));
    }
    out
}

fn ensure_log_dir() {
    if !Path::new(LOG_DIR).exists() {
        fs::create_dir_all(LOG_DIR).expect("Failed to create log directory");
    }
}

fn get_log_file_path(contact_count: Option<u8>) -> String {
    match contact_count {
        Some(count) => format!("{}/contacts_{}.log", LOG_DIR, count),
        None => format!("{}/contacts_unparsed.log", LOG_DIR),
    }
}

fn append_to_log_file(
    contact_count: Option<u8>,
    timestamp: u128,
    bytes_read: usize,
    raw_hex: &str,
    parsed_summary: Option<&str>,
) {
    let log_path = get_log_file_path(contact_count);

    if let Ok(mut file) = OpenOptions::new().create(true).append(true).open(&log_path) {
        let log_line = match parsed_summary {
            Some(summary) => format!(
                "[{} ms] len={} raw={} | {}\n",
                timestamp, bytes_read, raw_hex, summary
            ),
            None => format!("[{} ms] len={} raw={}\n", timestamp, bytes_read, raw_hex),
        };

        let _ = file.write_all(log_line.as_bytes());
    }
}

fn parse_touchpad_report_best_effort(data: &[u8]) -> Option<TouchpadReport> {
    if data.len() < 3 {
        return None;
    }

    // Generic PTP-like best-effort parser.
    // Layout assumption:
    //   Byte 0: report id
    //   Byte 1: flags (lower nibble may encode contact count, bit 4 may encode button)
    //   Byte 2..: contact array (6 bytes per contact in this simplified layout)
    let report_id = data[0];
    let flags = data[1];
    let mut contact_count = flags & 0x0F;
    let button = (flags & 0x10) != 0;

    let bytes_per_contact = 6usize;
    let payload = &data[2..];
    let max_contacts_by_len = (payload.len() / bytes_per_contact) as u8;
    if contact_count == 0 || contact_count > max_contacts_by_len {
        contact_count = max_contacts_by_len;
    }

    if contact_count == 0 {
        return Some(TouchpadReport {
            report_id,
            contact_count,
            button,
            contacts: Vec::new(),
        });
    }

    let mut contacts = Vec::with_capacity(contact_count as usize);
    for i in 0..contact_count as usize {
        let offset = i * bytes_per_contact;
        let c = &payload[offset..offset + bytes_per_contact];
        contacts.push(FingerContact {
            contact_id: c[0] & 0x7F,
            x: u16::from_le_bytes([c[1], c[2]]),
            y: u16::from_le_bytes([c[3], c[4]]),
            tip_switch: (c[0] & 0x80) != 0 || (c[5] & 0x01) != 0,
        });
    }

    Some(TouchpadReport {
        report_id,
        contact_count,
        button,
        contacts,
    })
}

fn main() {
    let args = CliArgs::parse();

    println!("Microsoft Precision Touchpad reader");
    println!("===================================\n");

    let api = HidApi::new().expect("failed to initialize HID API");
    let devices: Vec<DeviceInfo> = api.device_list().cloned().collect();

    println!("Scanning HID devices for Digitizer/Touchpad collections...\n");
    let mut found_any = false;
    for d in &devices {
        if is_touchpad_collection(d) {
            found_any = true;
            println!(
                "candidate: VID=0x{:04X} PID=0x{:04X} UP=0x{:04X} U=0x{:04X}",
                d.vendor_id(),
                d.product_id(),
                d.usage_page(),
                d.usage()
            );
            println!("  manufacturer: {:?}", d.manufacturer_string());
            println!("  product:      {:?}", d.product_string());
            println!("  path:         {:?}\n", d.path());
        }
    }

    if !found_any {
        eprintln!("No touchpad HID collections found (usage page 0x0D, usage 0x05).");
        eprintln!("Tip: move your finger on the touchpad and rerun this example.");
        return;
    }

    if args.list_only {
        println!("--list-only requested, exiting.");
        return;
    }

    let touchpad = match choose_touchpad_device(&devices, args.path_contains.as_deref()) {
        Some(device) => device,
        None => {
            eprintln!("No touchpad device matched selection filters.");
            if let Some(needle) = args.path_contains {
                eprintln!("Given --path-contains={:?}", needle);
            }
            return;
        }
    };
    println!(
        "Opening selected device: VID=0x{:04X} PID=0x{:04X}, path={:?}\n",
        touchpad.vendor_id(),
        touchpad.product_id(),
        touchpad.path()
    );

    let device = match touchpad.open_device(&api) {
        Ok(device) => device,
        Err(e) => {
            eprintln!("Failed to open touchpad device: {}", e);
            eprintln!("On macOS, grant Terminal/your IDE input access in:");
            eprintln!("  System Settings -> Privacy & Security -> Input Monitoring");
            return;
        }
    };

    ensure_log_dir();
    println!("Logging reports to: {}/", LOG_DIR);
    println!("Reading reports (Ctrl+C to stop)...\n");

    let mut buffer = [0u8; 256];
    let mut report_count = 0usize;
    loop {
        match device.read_timeout(&mut buffer, args.read_timeout_ms) {
            Ok(0) => continue,
            Ok(bytes_read) => {
                let data = &buffer[..bytes_read];
                report_count += 1;
                let ts = timestamp_ms();
                let raw_hex = hex_bytes(data);

                println!("[{} ms] len={} raw={}", ts, bytes_read, raw_hex);

                let (contact_count_opt, parsed_summary) = if !args.raw_only {
                    if let Some(parsed) = parse_touchpad_report_best_effort(data) {
                        let mut summary_parts = vec![
                            format!("report_id={}", parsed.report_id),
                            format!("contacts={}", parsed.contact_count),
                            format!("button={}", parsed.button),
                        ];

                        for c in &parsed.contacts {
                            summary_parts.push(format!(
                                "finger={} x={} y={} tip={}",
                                c.contact_id, c.x, c.y, c.tip_switch
                            ));
                        }

                        println!(
                            "  parsed: report_id={} contacts={} button={}",
                            parsed.report_id, parsed.contact_count, parsed.button
                        );
                        for c in parsed.contacts {
                            println!(
                                "    finger={} x={} y={} tip={}",
                                c.contact_id, c.x, c.y, c.tip_switch
                            );
                        }

                        (Some(parsed.contact_count), Some(summary_parts.join(" | ")))
                    } else {
                        (None, None)
                    }
                } else {
                    (None, None)
                };

                append_to_log_file(
                    contact_count_opt,
                    ts,
                    bytes_read,
                    &raw_hex,
                    parsed_summary.as_deref(),
                );

                if args
                    .max_reports
                    .map(|max_reports| report_count >= max_reports)
                    .unwrap_or(false)
                {
                    println!(
                        "Reached --max-reports={} (read {} non-empty reports), exiting.",
                        args.max_reports.unwrap_or(report_count),
                        report_count
                    );
                    println!("Log files written to: {}/", LOG_DIR);
                    return;
                }
            }
            Err(e) => {
                eprintln!("read error: {}", e);
            }
        }
    }
}
