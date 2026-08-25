use super::Detector;

impl Detector<'_> {
    pub(super) fn detect_brute_force(&mut self) {
        for index_a in 0..self.proxies.len() {
            for index_b in index_a + 1..self.proxies.len() {
                let a = self.proxies[index_a];
                let b = self.proxies[index_b];
                self.detect_pair(a, b);
            }
        }
    }
}
