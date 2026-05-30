use crate::{rw, rw_write, Rw};
use crate::fusion::{Fusion, AhrsCorrection};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::thread::JoinHandle;

pub struct Connection {
    pub fusion: Rw<AhrsCorrection>,
    pub terminating: Arc<AtomicBool>,
    pub interrupting: Arc<AtomicBool>, // when interrupting, update is paused, opening the fusion mutex for reading
    pub thread: Option<JoinHandle<()>>,
}

static CONNECTION: Mutex<Option<Connection>> = Mutex::new(None);

impl Connection {
    const ORDERING: Ordering = Ordering::SeqCst;

    fn get() -> crate::Result<MutexGuard<'static, Option<Connection>>> {
        let mut existing = Self::get_locked()?;

        if existing.is_none() {
            let fusion = <dyn Fusion>::any_cf()?;
            // let ahrs = AHRS::frd(fusion);
            let ahrs = AhrsCorrection::left_fru_down(fusion);

            *existing = Some(Connection {
                fusion: rw(ahrs),
                terminating: Arc::new(AtomicBool::new(false)),
                interrupting: Arc::new(AtomicBool::new(false)), // thread: None,
                thread: None,
            });
        }

        Ok(existing)
    }

    fn get_locked() -> crate::Result<MutexGuard<'static, Option<Connection>>> {
        CONNECTION
            .lock()
            .map_err(|_| crate::Error::ConcurrencyError)
    }

    pub fn _init(&self) -> crate::Result<()> {
        self.terminating.store(false, Self::ORDERING);
        self.interrupting.store(false, Self::ORDERING);

        Ok(())
    }

    fn _start(&mut self) -> crate::Result<()> {
        if self.thread.is_some() {
            return Ok(());
        }

        self._init()?;

        let _fusion = self.fusion.clone();
        let _terminating = self.terminating.clone();
        let _interrupting = self.interrupting.clone();

        let handle = thread::spawn(move || loop {
            if _terminating.load(Self::ORDERING) {
                break;
            }

            if _interrupting.load(Self::ORDERING) {
                // println!("busy, no update")
            } else {
                let mut ff = rw_write(&_fusion);

                ff.update();
                // println!("UPDATE!")
            }
        });

        self.thread = Some(handle);

        Ok(())
    }

    pub fn start() -> crate::Result<()> {
        let mut existing = Self::get()?;
        let conn = existing.as_mut().ok_or(crate::Error::ConcurrencyError)?;

        conn._start()
    }

    fn _stop(&mut self) -> crate::Result<()> {
        self.terminating.store(true, Self::ORDERING);

        if let Some(handle) = self.thread.take() {
            handle
                .join()
                .map_err(|_| crate::Error::Other("connection thread panicked"))?;
        }

        Ok(())
    }

    pub fn stop() -> crate::Result<()> {
        let mut existing = Self::get_locked()?;

        if let Some(mut conn) = existing.take() {
            conn._stop()?;
        }

        Ok(())
    }

    pub fn read_fusion<T>(f: &dyn Fn(&mut AhrsCorrection) -> T) -> crate::Result<T> {
        let (_fusion, _interrupting) = {
            let existing = Self::get()?;

            let conn = existing.as_ref().ok_or(crate::Error::ConcurrencyError)?;
            (conn.fusion.clone(), conn.interrupting.clone())
        };

        _interrupting.store(true, Self::ORDERING);
        let mut ahrs = rw_write(&_fusion);

        let result = f(&mut ahrs);

        _interrupting.store(false, Self::ORDERING);
        Ok(result)
    }
}

impl Drop for Connection {
    fn drop(&mut self) {
        let _ = self._stop();
    }
}
