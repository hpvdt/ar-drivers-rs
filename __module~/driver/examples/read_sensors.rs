// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use std::{
    io::{self, Stdout},
    sync::mpsc::{self, Receiver, Sender},
    thread,
    time::Duration,
};

use ar_drivers::{any_glasses_or_dummy, ARGlasses, GlassesEvent};
use crossterm::{
    cursor::{Hide, Show},
    event::{self, Event as TerminalEvent, KeyCode, KeyEvent, KeyEventKind, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use nalgebra::Vector3;
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, Wrap},
    Frame, Terminal,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut glasses = any_glasses_or_dummy()?;
    let device_name = glasses.name().to_owned();
    let serial = glasses
        .serial()
        .unwrap_or_else(|error| format!("unavailable ({error})"));

    let mut terminal = TerminalGuard::new()?;
    let state = DashboardState::new(device_name, serial);
    let (sender, receiver) = mpsc::channel();

    thread::spawn(move || read_events(glasses, sender));
    run_app(&mut terminal, receiver, state)?;

    Ok(())
}

fn read_events(mut glasses: Box<dyn ARGlasses>, sender: Sender<SensorMessage>) {
    loop {
        match glasses.read_event() {
            Ok(event) => {
                if sender.send(SensorMessage::Event(event)).is_err() {
                    break;
                }
            }
            Err(error) => {
                if sender
                    .send(SensorMessage::ReadError(error.to_string()))
                    .is_err()
                {
                    break;
                }

                thread::sleep(Duration::from_millis(100));
            }
        }
    }
}

fn run_app(
    terminal: &mut TerminalGuard,
    receiver: Receiver<SensorMessage>,
    mut state: DashboardState,
) -> io::Result<()> {
    loop {
        drain_messages(&receiver, &mut state);
        terminal.draw(&state)?;

        if event::poll(Duration::from_millis(50))? {
            if let TerminalEvent::Key(key) = event::read()? {
                if should_quit(key) {
                    break;
                }
            }
        }
    }

    Ok(())
}

fn drain_messages(receiver: &Receiver<SensorMessage>, state: &mut DashboardState) {
    while let Ok(message) = receiver.try_recv() {
        match message {
            SensorMessage::Event(event) => state.apply_event(event),
            SensorMessage::ReadError(error) => state.apply_error(error),
        }
    }
}

fn should_quit(key: KeyEvent) -> bool {
    if key.kind != KeyEventKind::Press {
        return false;
    }

    matches!(key.code, KeyCode::Esc | KeyCode::Char('q'))
        || (matches!(key.code, KeyCode::Char('c')) && key.modifiers.contains(KeyModifiers::CONTROL))
}

struct TerminalGuard {
    terminal: Terminal<CrosstermBackend<Stdout>>,
}

impl TerminalGuard {
    fn new() -> io::Result<Self> {
        let mut terminal = Terminal::new(CrosstermBackend::new(io::stdout()))?;

        enable_raw_mode()?;
        if let Err(error) = execute!(terminal.backend_mut(), EnterAlternateScreen, Hide) {
            let _ = disable_raw_mode();
            return Err(error);
        }

        let mut guard = Self { terminal };
        guard.terminal.clear()?;
        Ok(guard)
    }

    fn draw(&mut self, state: &DashboardState) -> io::Result<()> {
        self.terminal.draw(|frame| render(frame, state))?;
        Ok(())
    }
}

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        let _ = execute!(self.terminal.backend_mut(), Show, LeaveAlternateScreen);
        let _ = self.terminal.show_cursor();
    }
}

enum SensorMessage {
    Event(GlassesEvent),
    ReadError(String),
}

#[derive(Clone, Debug)]
struct DashboardState {
    device_name: String,
    serial: String,
    total_events: u64,
    last_error: Option<String>,
    acc_gyro: EventSlot<AccGyroData>,
    magnetometer: EventSlot<MagnetometerData>,
    key_press: EventSlot<KeyPressData>,
    proximity: EventSlot<ProximityData>,
    ambient_light: EventSlot<AmbientLightData>,
    vsync: EventSlot<VSyncData>,
}

impl DashboardState {
    fn new(device_name: impl Into<String>, serial: impl Into<String>) -> Self {
        Self {
            device_name: device_name.into(),
            serial: serial.into(),
            total_events: 0,
            last_error: None,
            acc_gyro: EventSlot::default(),
            magnetometer: EventSlot::default(),
            key_press: EventSlot::default(),
            proximity: EventSlot::default(),
            ambient_light: EventSlot::default(),
            vsync: EventSlot::default(),
        }
    }

    fn apply_event(&mut self, event: GlassesEvent) {
        self.total_events += 1;

        match event {
            GlassesEvent::AccGyro {
                accelerometer,
                gyroscope,
                timestamp,
            } => self.acc_gyro.update(AccGyroData {
                accelerometer,
                gyroscope,
                timestamp,
            }),
            GlassesEvent::Magnetometer {
                magnetometer,
                timestamp,
            } => self.magnetometer.update(MagnetometerData {
                magnetometer,
                timestamp,
            }),
            GlassesEvent::KeyPress(key) => self.key_press.update(KeyPressData { key }),
            GlassesEvent::ProximityNear => self.proximity.update(ProximityData {
                state: ProximityState::Near,
            }),
            GlassesEvent::ProximityFar => self.proximity.update(ProximityData {
                state: ProximityState::Far,
            }),
            GlassesEvent::AmbientLight(value) => {
                self.ambient_light.update(AmbientLightData { value })
            }
            GlassesEvent::VSync => self.vsync.update(VSyncData {
                event_number: self.total_events,
            }),
        }
    }

    fn apply_error(&mut self, error: String) {
        self.last_error = Some(error);
    }
}

#[derive(Clone, Debug)]
struct EventSlot<T> {
    value: Option<T>,
    count: u64,
}

impl<T> Default for EventSlot<T> {
    fn default() -> Self {
        Self {
            value: None,
            count: 0,
        }
    }
}

impl<T> EventSlot<T> {
    fn update(&mut self, value: T) {
        self.value = Some(value);
        self.count += 1;
    }
}

#[derive(Clone, Debug, PartialEq)]
struct AccGyroData {
    accelerometer: Vector3<f32>,
    gyroscope: Vector3<f32>,
    timestamp: u64,
}

#[derive(Clone, Debug, PartialEq)]
struct MagnetometerData {
    magnetometer: Vector3<f32>,
    timestamp: u64,
}

#[derive(Clone, Debug, PartialEq)]
struct KeyPressData {
    key: u8,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct ProximityData {
    state: ProximityState,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum ProximityState {
    Near,
    Far,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct AmbientLightData {
    value: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct VSyncData {
    event_number: u64,
}

fn render(frame: &mut Frame<'_>, state: &DashboardState) {
    let root = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(4),
            Constraint::Min(8),
            Constraint::Length(3),
        ])
        .split(frame.area());

    render_header(frame, root[0], state);

    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(root[1]);
    let top = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(60), Constraint::Percentage(40)])
        .split(rows[0]);
    let bottom = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(25),
            Constraint::Percentage(25),
            Constraint::Percentage(25),
            Constraint::Percentage(25),
        ])
        .split(rows[1]);

    render_panel(
        frame,
        top[0],
        "AccGyro",
        state.acc_gyro.count,
        acc_gyro_lines(&state.acc_gyro),
        Color::Cyan,
    );
    render_panel(
        frame,
        top[1],
        "Magnetometer",
        state.magnetometer.count,
        magnetometer_lines(&state.magnetometer),
        Color::Magenta,
    );
    render_panel(
        frame,
        bottom[0],
        "KeyPress",
        state.key_press.count,
        key_press_lines(&state.key_press),
        Color::Yellow,
    );
    render_panel(
        frame,
        bottom[1],
        "Proximity",
        state.proximity.count,
        proximity_lines(&state.proximity),
        Color::Green,
    );
    render_panel(
        frame,
        bottom[2],
        "AmbientLight",
        state.ambient_light.count,
        ambient_light_lines(&state.ambient_light),
        Color::Blue,
    );
    render_panel(
        frame,
        bottom[3],
        "VSync",
        state.vsync.count,
        vsync_lines(&state.vsync),
        Color::LightRed,
    );

    render_footer(frame, root[2], state);
}

fn render_header(frame: &mut Frame<'_>, area: Rect, state: &DashboardState) {
    let lines = vec![
        Line::from(vec![
            Span::styled(
                "ar-drivers read_sensors",
                Style::default()
                    .fg(Color::White)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw("  "),
            Span::styled("press q, Esc, or Ctrl-C to exit", muted_style()),
        ]),
        field(
            "Device",
            format!("{}  serial {}", state.device_name, state.serial),
        ),
    ];

    let header = Paragraph::new(lines)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Sensor Events "),
        )
        .wrap(Wrap { trim: false });
    frame.render_widget(header, area);
}

fn render_footer(frame: &mut Frame<'_>, area: Rect, state: &DashboardState) {
    let error = state.last_error.as_deref().unwrap_or("none");
    let footer = Paragraph::new(vec![
        field("Total events", state.total_events.to_string()),
        field("Last error", error.to_owned()),
    ])
    .block(Block::default().borders(Borders::ALL).title(" Status "))
    .wrap(Wrap { trim: false });

    frame.render_widget(footer, area);
}

fn render_panel(
    frame: &mut Frame<'_>,
    area: Rect,
    title: &str,
    count: u64,
    lines: Vec<Line<'static>>,
    color: Color,
) {
    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(color))
        .title(format!(" {title} ({count}) "));
    let paragraph = Paragraph::new(lines)
        .block(block)
        .wrap(Wrap { trim: false });

    frame.render_widget(paragraph, area);
}

fn acc_gyro_lines(slot: &EventSlot<AccGyroData>) -> Vec<Line<'static>> {
    match &slot.value {
        Some(data) => vec![
            field("Timestamp", format!("{} us", data.timestamp)),
            field("Accel", format_vec3(&data.accelerometer)),
            field("Gyro", format_vec3(&data.gyroscope)),
        ],
        None => waiting_lines(),
    }
}

fn magnetometer_lines(slot: &EventSlot<MagnetometerData>) -> Vec<Line<'static>> {
    match &slot.value {
        Some(data) => vec![
            field("Timestamp", format!("{} us", data.timestamp)),
            field("Mag", format_vec3(&data.magnetometer)),
        ],
        None => waiting_lines(),
    }
}

fn key_press_lines(slot: &EventSlot<KeyPressData>) -> Vec<Line<'static>> {
    match &slot.value {
        Some(data) => vec![field("Last key", data.key.to_string())],
        None => waiting_lines(),
    }
}

fn proximity_lines(slot: &EventSlot<ProximityData>) -> Vec<Line<'static>> {
    match &slot.value {
        Some(data) => {
            let value = match data.state {
                ProximityState::Near => "near",
                ProximityState::Far => "far",
            };
            vec![field("Last state", value)]
        }
        None => waiting_lines(),
    }
}

fn ambient_light_lines(slot: &EventSlot<AmbientLightData>) -> Vec<Line<'static>> {
    match &slot.value {
        Some(data) => vec![field("Level", data.value.to_string())],
        None => waiting_lines(),
    }
}

fn vsync_lines(slot: &EventSlot<VSyncData>) -> Vec<Line<'static>> {
    match &slot.value {
        Some(data) => vec![field("Last event #", data.event_number.to_string())],
        None => waiting_lines(),
    }
}

fn waiting_lines() -> Vec<Line<'static>> {
    vec![Line::styled("Waiting for event...", muted_style())]
}

fn field(label: &str, value: impl Into<String>) -> Line<'static> {
    Line::from(vec![
        Span::styled(format!("{label:<13}"), muted_style()),
        Span::raw(value.into()),
    ])
}

fn format_vec3(value: &Vector3<f32>) -> String {
    format!("x {:>9.3}  y {:>9.3}  z {:>9.3}", value.x, value.y, value.z)
}

fn muted_style() -> Style {
    Style::default().fg(Color::DarkGray)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOTAL: usize = 0;
    const ACC_GYRO: usize = 1;
    const MAGNETOMETER: usize = 2;
    const KEY_PRESS: usize = 3;
    const PROXIMITY: usize = 4;
    const AMBIENT_LIGHT: usize = 5;
    const VSYNC: usize = 6;

    #[test]
    fn classifies_each_event_into_one_fixed_slot() {
        let events = [
            (
                GlassesEvent::AccGyro {
                    accelerometer: Vector3::new(1.0, 2.0, 3.0),
                    gyroscope: Vector3::new(4.0, 5.0, 6.0),
                    timestamp: 7,
                },
                ACC_GYRO,
            ),
            (
                GlassesEvent::Magnetometer {
                    magnetometer: Vector3::new(8.0, 9.0, 10.0),
                    timestamp: 11,
                },
                MAGNETOMETER,
            ),
            (GlassesEvent::KeyPress(12), KEY_PRESS),
            (GlassesEvent::ProximityNear, PROXIMITY),
            (GlassesEvent::ProximityFar, PROXIMITY),
            (GlassesEvent::AmbientLight(13), AMBIENT_LIGHT),
            (GlassesEvent::VSync, VSYNC),
        ];

        for (event, expected_slot) in events {
            assert_only_slot_changes(event, expected_slot);
        }
    }

    #[test]
    fn stores_latest_event_payload() {
        let mut state = DashboardState::new("device", "serial");

        state.apply_event(GlassesEvent::AccGyro {
            accelerometer: Vector3::new(1.0, 2.0, 3.0),
            gyroscope: Vector3::new(4.0, 5.0, 6.0),
            timestamp: 7,
        });
        state.apply_event(GlassesEvent::KeyPress(42));
        state.apply_event(GlassesEvent::ProximityFar);

        let acc_gyro = state.acc_gyro.value.as_ref().unwrap();
        assert_eq!(acc_gyro.accelerometer, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(acc_gyro.gyroscope, Vector3::new(4.0, 5.0, 6.0));
        assert_eq!(acc_gyro.timestamp, 7);
        assert_eq!(state.key_press.value.as_ref().unwrap().key, 42);
        assert_eq!(
            state.proximity.value.as_ref().unwrap().state,
            ProximityState::Far
        );
    }

    #[test]
    fn stores_last_read_error_without_counting_an_event() {
        let mut state = DashboardState::new("device", "serial");

        state.apply_error("read failed".to_owned());

        assert_eq!(state.last_error.as_deref(), Some("read failed"));
        assert_eq!(counts(&state)[TOTAL], 0);
    }

    fn assert_only_slot_changes(event: GlassesEvent, expected_slot: usize) {
        let mut state = DashboardState::new("device", "serial");
        let before = counts(&state);

        state.apply_event(event);

        let after = counts(&state);
        for index in 0..after.len() {
            let expected_delta = u64::from(index == TOTAL || index == expected_slot);
            assert_eq!(
                after[index],
                before[index] + expected_delta,
                "unexpected count for slot {index}"
            );
        }
    }

    fn counts(state: &DashboardState) -> [u64; 7] {
        [
            state.total_events,
            state.acc_gyro.count,
            state.magnetometer.count,
            state.key_press.count,
            state.proximity.count,
            state.ambient_light.count,
            state.vsync.count,
        ]
    }
}
