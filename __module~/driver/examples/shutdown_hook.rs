use std::{thread, time::Duration};

// TODO: this is just an example, should be moved to mavlink-stream later
struct ShutdownHook<F: FnOnce()> {
    hook: Option<F>,
}

impl<F: FnOnce()> ShutdownHook<F> {
    fn new(hook: F) -> Self {
        Self { hook: Some(hook) }
    }
}

impl<F: FnOnce()> Drop for ShutdownHook<F> {
    fn drop(&mut self) {
        if let Some(hook) = self.hook.take() {
            hook();
        }
    }
}

fn main() {
    let _shutdown_hook = ShutdownHook::new(|| {
        println!("shutdown hook invoked automatically");
    });

    println!("running for 1 second...");
    thread::sleep(Duration::from_secs(1));
    println!("main is returning normally");
}
