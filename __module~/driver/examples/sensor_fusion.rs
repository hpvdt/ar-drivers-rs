// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use std::{
    io::{self, Stdout},
    sync::mpsc::{self, Receiver, SyncSender},
    thread,
    time::Duration,
};

use ar_drivers::fusion::{AhrsCorrection, Correction, Corrections, Fusion};
use nalgebra::Vector3;
use ratatui::{
    backend::CrosstermBackend,
    crossterm::{
        cursor::{Hide, Show},
        event::{self, Event as TerminalEvent, KeyCode, KeyEvent, KeyEventKind, KeyModifiers},
        execute,
        terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    },
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, Wrap},
    Frame, Terminal,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut fusion = <dyn Fusion>::any_cf()?;
    let (device_name, serial) = {
        let glasses = fusion.glasses();
        let device_name = glasses.name().to_owned();
        let serial = glasses
            .serial()
            .unwrap_or_else(|error| format!("unavailable ({error})"));
        (device_name, serial)
    };

    let ahrs = AhrsCorrection::left_fru_down(fusion);
    let mut terminal = TerminalGuard::new()?;
    let state = DashboardState::new(device_name, serial);
    let (sender, receiver) = mpsc::sync_channel(1);

    thread::spawn(move || update_fusion(ahrs, sender));
    run_app(&mut terminal, receiver, state)?;

    Ok(())
}

fn update_fusion(mut ahrs: AhrsCorrection, sender: SyncSender<FusionMessage>) {
    loop {
        ahrs.update();

        if sender
            .send(FusionMessage::Snapshot(FusionSnapshot::from_ahrs(&ahrs)))
            .is_err()
        {
            break;
        }
    }
}

fn run_app(
    terminal: &mut TerminalGuard,
    receiver: Receiver<FusionMessage>,
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

fn drain_messages(receiver: &Receiver<FusionMessage>, state: &mut DashboardState) {
    for _ in 0..64 {
        match receiver.try_recv() {
            Ok(FusionMessage::Snapshot(snapshot)) => state.apply_snapshot(snapshot),
            Err(_) => break,
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

enum FusionMessage {
    Snapshot(FusionSnapshot),
}

#[derive(Clone, Debug)]
struct DashboardState {
    device_name: String,
    serial: String,
    updates: u64,
    last_error: Option<String>,
    latest: Option<FusionSnapshot>,
}

impl DashboardState {
    fn new(device_name: impl Into<String>, serial: impl Into<String>) -> Self {
        Self {
            device_name: device_name.into(),
            serial: serial.into(),
            updates: 0,
            last_error: None,
            latest: None,
        }
    }

    fn apply_snapshot(&mut self, snapshot: FusionSnapshot) {
        self.updates += 1;
        self.latest = Some(snapshot);
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct FusionSnapshot {
    quaternion: QuaternionData,
    euler_deg: Vector3<f32>,
    corrections: CorrectionsData,
    inconsistency: f32,
}

impl FusionSnapshot {
    fn from_ahrs(ahrs: &AhrsCorrection) -> Self {
        let quaternion = ahrs.attitude_quaternion();

        Self {
            quaternion: QuaternionData {
                i: quaternion.i,
                j: quaternion.j,
                k: quaternion.k,
                w: quaternion.w,
            },
            euler_deg: ahrs.attitude_euler_deg(),
            corrections: CorrectionsData::from(ahrs.corrections()),
            inconsistency: ahrs.inconsistency(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct QuaternionData {
    i: f32,
    j: f32,
    k: f32,
    w: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CorrectionsData {
    acc: CorrectionData,
    gyro: CorrectionData,
    mag: CorrectionData,
}

impl From<Corrections> for CorrectionsData {
    fn from(corrections: Corrections) -> Self {
        Self {
            acc: CorrectionData::from(corrections.acc),
            gyro: CorrectionData::from(corrections.gyro),
            mag: CorrectionData::from(corrections.mag),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CorrectionData {
    prev: f32,
    avg: f32,
}

impl From<Correction> for CorrectionData {
    fn from(correction: Correction) -> Self {
        Self {
            prev: correction.prev,
            avg: correction.avg,
        }
    }
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
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(rows[0]);
    let bottom = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(65), Constraint::Percentage(35)])
        .split(rows[1]);

    render_panel(
        frame,
        top[0],
        "Quaternion",
        state.updates,
        quaternion_lines(state),
        Color::Cyan,
    );
    render_panel(
        frame,
        top[1],
        "Euler Degrees",
        state.updates,
        euler_lines(state),
        Color::Magenta,
    );
    render_panel(
        frame,
        bottom[0],
        "Corrections",
        state.updates,
        correction_lines(state),
        Color::Green,
    );
    render_panel(
        frame,
        bottom[1],
        "Fusion",
        state.updates,
        fusion_lines(state),
        Color::Yellow,
    );

    render_footer(frame, root[2], state);
}

fn render_header(frame: &mut Frame<'_>, area: Rect, state: &DashboardState) {
    let lines = vec![
        Line::from(vec![
            Span::styled(
                "ar-drivers sensor_fusion",
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
                .title(" Fusion State "),
        )
        .wrap(Wrap { trim: false });
    frame.render_widget(header, area);
}

fn render_footer(frame: &mut Frame<'_>, area: Rect, state: &DashboardState) {
    let error = state.last_error.as_deref().unwrap_or("none");
    let footer = Paragraph::new(vec![
        field("Updates", state.updates.to_string()),
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

fn quaternion_lines(state: &DashboardState) -> Vec<Line<'static>> {
    match state.latest {
        Some(snapshot) => vec![
            field("i", format_float(snapshot.quaternion.i)),
            field("j", format_float(snapshot.quaternion.j)),
            field("k", format_float(snapshot.quaternion.k)),
            field("w", format_float(snapshot.quaternion.w)),
        ],
        None => waiting_lines(),
    }
}

fn euler_lines(state: &DashboardState) -> Vec<Line<'static>> {
    match state.latest {
        Some(snapshot) => vec![field("Euler", format_vec3(&snapshot.euler_deg))],
        None => waiting_lines(),
    }
}

fn correction_lines(state: &DashboardState) -> Vec<Line<'static>> {
    match state.latest {
        Some(snapshot) => vec![
            field("Acc", format_correction(snapshot.corrections.acc)),
            field("Gyro", format_correction(snapshot.corrections.gyro)),
            field("Mag", format_correction(snapshot.corrections.mag)),
        ],
        None => waiting_lines(),
    }
}

fn fusion_lines(state: &DashboardState) -> Vec<Line<'static>> {
    match state.latest {
        Some(snapshot) => vec![
            field("Updates", state.updates.to_string()),
            field("Inconsistency", format_float(snapshot.inconsistency)),
        ],
        None => waiting_lines(),
    }
}

fn waiting_lines() -> Vec<Line<'static>> {
    vec![Line::styled("Waiting for update...", muted_style())]
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

fn format_correction(value: CorrectionData) -> String {
    format!(
        "prev {}  avg {}",
        format_float(value.prev),
        format_float(value.avg)
    )
}

fn format_float(value: f32) -> String {
    format!("{value:>10.7}")
}

fn muted_style() -> Style {
    Style::default().fg(Color::DarkGray)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stores_latest_snapshot_and_counts_updates() {
        let mut state = DashboardState::new("device", "serial");
        let snapshot = sample_snapshot(1.0);

        state.apply_snapshot(snapshot);

        assert_eq!(state.updates, 1);
        assert_eq!(state.latest, Some(snapshot));
    }

    #[test]
    fn replaces_latest_snapshot_without_losing_update_count() {
        let mut state = DashboardState::new("device", "serial");
        let first = sample_snapshot(1.0);
        let second = sample_snapshot(2.0);

        state.apply_snapshot(first);
        state.apply_snapshot(second);

        assert_eq!(state.updates, 2);
        assert_eq!(state.latest, Some(second));
    }

    fn sample_snapshot(seed: f32) -> FusionSnapshot {
        FusionSnapshot {
            quaternion: QuaternionData {
                i: seed,
                j: seed + 1.0,
                k: seed + 2.0,
                w: seed + 3.0,
            },
            euler_deg: Vector3::new(seed + 4.0, seed + 5.0, seed + 6.0),
            corrections: CorrectionsData {
                acc: CorrectionData {
                    prev: seed + 7.0,
                    avg: seed + 8.0,
                },
                gyro: CorrectionData {
                    prev: seed + 9.0,
                    avg: seed + 10.0,
                },
                mag: CorrectionData {
                    prev: seed + 11.0,
                    avg: seed + 12.0,
                },
            },
            inconsistency: seed + 13.0,
        }
    }
}
