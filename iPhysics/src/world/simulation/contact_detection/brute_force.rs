use super::{AabbProxy, detect_pair};
use crate::body::{Body, StaticBody};
use crate::world::{ActiveContact, StepStats};
use alloc::vec::Vec;

pub(super) fn detect(
    bodies: &[Body],
    static_bodies: &[StaticBody],
    active_contacts: &mut Vec<ActiveContact>,
    proxies: &[AabbProxy],
    stats: &mut StepStats,
) {
    for index_a in 0..proxies.len() {
        for index_b in index_a + 1..proxies.len() {
            detect_pair(
                bodies,
                static_bodies,
                active_contacts,
                proxies[index_a],
                proxies[index_b],
                stats,
            );
        }
    }
}
