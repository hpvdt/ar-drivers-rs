use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::thread::JoinHandle;
use crate::{AHRS, Fusion, Rw, rw, rw_write};

pub struct Connection {
    pub fusion: Rw<AHRS>,
    pub terminating: Arc<AtomicBool>,
    pub interrupting: Arc<AtomicBool>, // when interrupting, update is paused, opening the fusion mutex for reading
    pub thread: Option<JoinHandle<()>>,
}

static mut CONNECTION: Option<Connection> = None;

impl Connection {
    const ORDERING: Ordering = Ordering::SeqCst;

    fn new() -> Self {
        Connection {
            fusion: {
                let fusion = <dyn Fusion>::any_cf().unwrap();
                // let ahrs = AHRS::frd(fusion);
                let ahrs = AHRS::left_fru_down(fusion);
                rw(ahrs)
            },
            terminating: Arc::new(AtomicBool::new(false)),
            interrupting: Arc::new(AtomicBool::new(false)), // thread: None,
            thread: None,
        }
    }

    fn get() -> crate::Result<&'static mut Connection> {
        unsafe {
            let existing = &mut CONNECTION;
            let neo = existing.get_or_insert_with(|| Connection::new());
            Ok(neo)
        }
    }

    pub fn _init(&self) -> crate::Result<()> {
        self.terminating.store(false, Self::ORDERING);
        self.interrupting.store(false, Self::ORDERING);

        Ok(())
    }

    fn _start(&mut self) -> crate::Result<()> {
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

    pub fn start() -> crate::Result<&'static Connection> {
        let conn = Self::get()?;

        conn._start()?;

        Ok(conn)
    }

    fn _stop(&mut self) -> crate::Result<()> {
        self.terminating.store(true, Self::ORDERING);

        self.thread.take().unwrap().join().unwrap();

        Ok(())
    }

    pub fn stop() -> crate::Result<()> {
        unsafe {
            let existing = &mut CONNECTION;
            *existing = None;
            Ok(())
        }
    }

    pub fn read_fusion<T>(f: &dyn Fn(&mut AHRS) -> T) -> crate::Result<T> {
        let conn = Self::get()?;
        let _fusion = &conn.fusion;

        conn.interrupting.store(true, Self::ORDERING);
        let mut ahrs = rw_write(&_fusion);

        let result = f(&mut ahrs);

        conn.interrupting.store(false, Self::ORDERING);
        Ok(result)
    }
}

impl Drop for Connection {
    fn drop(&mut self) {
        self._stop().unwrap()
    }
}