// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use std::env;
use std::error::Error;
use std::io::stdout;
use std::time::{Duration, Instant};

use ar_drivers::any_glasses_or_dummy;
use ar_drivers::fusion::{rub_to_frd, BadMagDataCause, FusionState};
use ar_drivers::GlassesEvent;
use nalgebra::Vector3;
use ratatui::backend::CrosstermBackend;
use ratatui::widgets::{Block, Clear, Paragraph, Widget};
use ratatui::{Terminal, TerminalOptions, Viewport};

const FOOTER_HEIGHT: u16 = 8;

struct LatestReadings {
    acc_gyro: String,
    acc_gyro_source: String,
    magnetometer: String,
    calibration: String,
    magnetometer_source: String,
    raw_event: String,
}

impl LatestReadings {
    fn new() -> Self {
        Self {
            acc_gyro: String::new(),
            acc_gyro_source: String::new(),
            magnetometer: String::new(),
            calibration: String::new(),
            magnetometer_source: String::new(),
            raw_event: String::new(),
        }
    }

    fn text(&self) -> String {
        [
            self.acc_gyro.as_str(),
            self.acc_gyro_source.as_str(),
            self.magnetometer.as_str(),
            self.calibration.as_str(),
            self.magnetometer_source.as_str(),
            self.raw_event.as_str(),
        ]
        .join("\n")
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut glasses = any_glasses_or_dummy()?;
    let serial = glasses.serial()?;
    let mut latest = LatestReadings::new();
    let mut fusion = FusionState::new(glasses);

    let backend = CrosstermBackend::new(stdout());
    let mut terminal = Terminal::with_options(
        backend,
        TerminalOptions {
            viewport: Viewport::Inline(FOOTER_HEIGHT),
        },
    )?;

    insert_log(
        &mut terminal,
        vec![format!("Got glasses, serial={}", serial)],
    )?;
    terminal.draw(|frame| {
        render_footer(frame, &latest);
    })?;

    let run_for = env::var("READ_SENSORS_TUI_SECONDS")
        .ok()
        .and_then(|seconds| seconds.parse::<u64>().ok())
        .map(Duration::from_secs);
    let started_at = Instant::now();

    loop {
        if run_for
            .map(|duration| started_at.elapsed() >= duration)
            .unwrap_or(false)
        {
            break;
        }

        let event = fusion.glasses.read_event()?;
        let lines = format_event(&mut fusion, event, &mut latest);
        insert_log(&mut terminal, lines)?;
        terminal.draw(|frame| {
            render_footer(frame, &latest);
        })?;
    }

    Ok(())
}

fn format_event(
    fusion: &mut FusionState,
    event: GlassesEvent,
    latest: &mut LatestReadings,
) -> Vec<String> {
    match event {
        GlassesEvent::AccGyro {
            accelerometer,
            gyroscope,
            timestamp,
        } => {
            let acc_frd = rub_to_frd(&accelerometer);
            let gyr_frd = rub_to_frd(&gyroscope);
            let reading = format!(
                "AccGyro FRD: accelerometer={} gyroscope={} timestamp={:>12}",
                format_vec3(&acc_frd),
                format_vec3(&gyr_frd),
                timestamp
            );
            let source = format!("  - converted from raw {}", format_glasses_event(&event));
            latest.acc_gyro.clone_from(&reading);
            latest.acc_gyro_source.clone_from(&source);
            vec![reading, source]
        }
        GlassesEvent::Magnetometer {
            magnetometer,
            timestamp,
        } => {
            let mag_frd = rub_to_frd(&magnetometer);
            let reading = format!(
                "Magnetometer FRD: mag={} timestamp={:>12}",
                format_vec3(&mag_frd),
                timestamp
            );
            let calibration = match fusion.getCalibratedMag(mag_frd) {
                Ok(calibrated) => {
                    format!("  calibrated (normalized): {}", format_vec3(&calibrated))
                }
                Err(cause) => {
                    format!(
                        "  calibration unavailable: {}",
                        format_bad_mag_data_cause(&cause)
                    )
                }
            };
            let source = format!("  - converted from raw {}", format_glasses_event(&event));
            latest.magnetometer.clone_from(&reading);
            latest.calibration.clone_from(&calibration);
            latest.magnetometer_source.clone_from(&source);
            vec![reading, calibration, source]
        }
        _ => {
            let raw = format!("Raw event: {}", format_glasses_event(&event));
            latest.raw_event.clone_from(&raw);
            vec![raw]
        }
    }
}

fn insert_log(
    terminal: &mut Terminal<CrosstermBackend<std::io::Stdout>>,
    lines: Vec<String>,
) -> std::io::Result<()> {
    let text = lines.join("\n");
    let height = text.lines().count().max(1).min(u16::MAX as usize) as u16;
    terminal.insert_before(height, move |buffer| {
        Paragraph::new(text).render(buffer.area, buffer);
    })
}

fn render_footer(frame: &mut ratatui::Frame<'_>, latest: &LatestReadings) {
    let area = frame.area();
    frame.render_widget(Clear, area);
    frame.render_widget(Paragraph::new(latest.text()).block(Block::bordered()), area);
}

fn format_float(value: f32) -> String {
    format!("{:+10.4}", value)
}

fn format_vec3(vector: &Vector3<f32>) -> String {
    format!(
        "[x={}, y={}, z={}]",
        format_float(vector.x),
        format_float(vector.y),
        format_float(vector.z)
    )
}

fn format_glasses_event(event: &GlassesEvent) -> String {
    match event {
        GlassesEvent::AccGyro {
            accelerometer,
            gyroscope,
            timestamp,
        } => format!(
            "AccGyro {{ accelerometer: {}, gyroscope: {}, timestamp: {:>12} }}",
            format_vec3(accelerometer),
            format_vec3(gyroscope),
            timestamp
        ),
        GlassesEvent::Magnetometer {
            magnetometer,
            timestamp,
        } => format!(
            "Magnetometer {{ magnetometer: {}, timestamp: {:>12} }}",
            format_vec3(magnetometer),
            timestamp
        ),
        _ => format!("{:?}", event),
    }
}

fn format_bad_mag_data_cause(cause: &BadMagDataCause) -> String {
    match cause {
        BadMagDataCause::InsufficientSamples => "InsufficientSamples".to_string(),
        BadMagDataCause::NumericallyUnstableCalibration { offset, scale } => format!(
            "NumericallyUnstableCalibration {{ offset: {}, scale: {} }}",
            format_vec3(offset),
            format_vec3(scale)
        ),
        BadMagDataCause::WeakCalibratedReading { norm, min_norm } => format!(
            "WeakCalibratedReading {{ norm: {}, min_norm: {} }}",
            format_float(*norm),
            format_float(*min_norm)
        ),
    }
}
