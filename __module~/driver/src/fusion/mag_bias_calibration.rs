use nalgebra::Vector3;

pub(super) struct MagBiasCalibration<const N: usize> {
    samples: [Vector3<f32>; N],
    sum: Vector3<f32>,
    count: usize,
    next_index: usize,
    first_timestamp: Option<u64>,
    enabled: bool,
    bias: Vector3<f32>,
}

impl<const N: usize> MagBiasCalibration<N> {
    pub(super) const ENABLE_AFTER_US: u64 = 5_000_000;

    pub(super) fn new() -> Self {
        debug_assert!(N > 0);

        Self {
            samples: [Vector3::zeros(); N],
            sum: Vector3::zeros(),
            count: 0,
            next_index: 0,
            first_timestamp: None,
            enabled: false,
            bias: Vector3::zeros(),
        }
    }

    pub(super) fn corrected_sample(
        &mut self,
        sample: Vector3<f32>,
        timestamp: u64,
    ) -> Vector3<f32> {
        self.record(sample, timestamp);
        match self.bias() {
            Some(bias) => sample - bias,
            None => sample,
        }
    }

    pub(super) fn record(&mut self, sample: Vector3<f32>, timestamp: u64) {
        if self.first_timestamp.is_none() {
            self.first_timestamp = Some(timestamp);
        }

        if self.count < N {
            self.samples[self.next_index] = sample;
            self.sum += sample;
            self.count += 1;
        } else {
            let old_sample = self.samples[self.next_index];
            self.samples[self.next_index] = sample;
            self.sum += sample - old_sample;
        }

        self.next_index = (self.next_index + 1) % N;
        self.bias = self.sum / self.count as f32;

        if !self.enabled {
            let has_elapsed = self
                .first_timestamp
                .map(|first_timestamp| {
                    timestamp.saturating_sub(first_timestamp) >= Self::ENABLE_AFTER_US
                })
                .unwrap_or(false);
            self.enabled = self.count == N || has_elapsed;
        }
    }

    pub(super) fn bias(&self) -> Option<Vector3<f32>> {
        if self.enabled {
            Some(self.bias)
        } else {
            None
        }
    }

    #[cfg(test)]
    pub(super) fn is_enabled(&self) -> bool {
        self.enabled
    }
}
