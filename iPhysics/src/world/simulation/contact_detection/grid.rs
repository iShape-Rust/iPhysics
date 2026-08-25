use super::{AabbProxy, detect_pair};
use crate::body::{Body, StaticBody};
use crate::geometry::Aabb;
use crate::world::{ActiveContact, GridBroadPhase, StepStats};
use alloc::vec::Vec;

#[derive(Debug, Clone, Default)]
pub(super) struct Scratch {
    column_counts: Vec<u32>,
    column_offsets: Vec<u32>,
    column_proxies: Vec<AabbProxy>,
}

impl Scratch {
    pub(super) const fn new() -> Self {
        Self {
            column_counts: Vec::new(),
            column_offsets: Vec::new(),
            column_proxies: Vec::new(),
        }
    }

    pub(super) fn clear(&mut self) {
        self.column_counts.clear();
        self.column_offsets.clear();
        self.column_proxies.clear();
    }
}

#[derive(Debug, Clone, Copy)]
struct ColumnLayout {
    min_x: i32,
    shift: u32,
    column_count: usize,
}

impl ColumnLayout {
    fn new(proxies: &[AabbProxy], settings: GridBroadPhase) -> Option<Self> {
        let first = proxies.first()?;
        let mut min_x = first.aabb.min().raw()[0];
        let mut max_x = first.aabb.max().raw()[0];
        for proxy in &proxies[1..] {
            min_x = min_x.min(proxy.aabb.min().raw()[0]);
            max_x = max_x.max(proxy.aabb.max().raw()[0]);
        }

        let shift = 16 + u32::from(settings.column_width_power());
        let span = max_x - min_x;
        let column_count = ((span >> shift) + 1) as usize;
        Some(Self {
            min_x,
            shift,
            column_count,
        })
    }

    #[inline(always)]
    fn column(self, x: i32) -> usize {
        debug_assert!(x >= self.min_x);
        ((x - self.min_x) >> self.shift) as usize
    }

    #[inline(always)]
    fn first(self, aabb: Aabb) -> usize {
        self.column(aabb.min().raw()[0])
    }

    #[inline(always)]
    fn last(self, aabb: Aabb) -> usize {
        self.column(aabb.max().raw()[0])
    }
}

pub(super) fn detect(
    bodies: &[Body],
    static_bodies: &[StaticBody],
    active_contacts: &mut Vec<ActiveContact>,
    proxies: &[AabbProxy],
    scratch: &mut Scratch,
    settings: GridBroadPhase,
    stats: &mut StepStats,
) {
    let Some(layout) = ColumnLayout::new(proxies, settings) else {
        return;
    };

    scratch.column_counts.clear();
    scratch.column_counts.resize(layout.column_count, 0);
    for proxy in proxies {
        for column in layout.first(proxy.aabb)..=layout.last(proxy.aabb) {
            scratch.column_counts[column] = scratch.column_counts[column]
                .checked_add(1)
                .expect("broad-phase column body count exceeds u32");
        }
    }

    scratch.column_offsets.clear();
    scratch.column_offsets.reserve(layout.column_count + 1);
    scratch.column_offsets.push(0);
    let mut entry_count = 0_u32;
    for &count in &scratch.column_counts {
        if count >= 2 {
            entry_count = entry_count
                .checked_add(count)
                .expect("broad-phase column entry count exceeds u32");
        }
        scratch.column_offsets.push(entry_count);
    }

    scratch
        .column_proxies
        .resize(entry_count as usize, proxies[0]);
    for (column, cursor) in scratch.column_counts.iter_mut().enumerate() {
        *cursor = scratch.column_offsets[column];
    }
    for &proxy in proxies {
        for column in layout.first(proxy.aabb)..=layout.last(proxy.aabb) {
            if scratch.column_offsets[column] == scratch.column_offsets[column + 1] {
                continue;
            }
            let cursor = &mut scratch.column_counts[column];
            scratch.column_proxies[*cursor as usize] = proxy;
            *cursor += 1;
        }
    }

    for column in 0..layout.column_count {
        let start = scratch.column_offsets[column] as usize;
        let end = scratch.column_offsets[column + 1] as usize;
        let column_proxies = &scratch.column_proxies[start..end];
        for index_a in 0..column_proxies.len() {
            let a = column_proxies[index_a];
            let a_starts_here = layout.first(a.aabb) == column;
            for &b in &column_proxies[index_a + 1..] {
                if !a_starts_here && layout.first(b.aabb) != column {
                    continue;
                }
                detect_pair(bodies, static_bodies, active_contacts, a, b, stats);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::Position;
    use crate::world::ContactBodyIndex;

    #[test]
    fn full_geometry_span_fits_i32_column_math() {
        let min = Position::MIN_POINT;
        let max = Position::MAX_POINT;
        let proxies = [
            AabbProxy {
                aabb: Aabb::from_raw_unchecked(min, min, 0, 0),
                body: ContactBodyIndex::Dynamic(0),
            },
            AabbProxy {
                aabb: Aabb::from_raw_unchecked(max, max, 0, 0),
                body: ContactBodyIndex::Dynamic(1),
            },
        ];

        let layout = ColumnLayout::new(&proxies, GridBroadPhase::new(0).unwrap()).unwrap();

        assert_eq!(max - min, i32::MAX - 1);
        assert_eq!(layout.column_count, 1 << 15);
        assert_eq!(layout.first(proxies[0].aabb), 0);
        assert_eq!(layout.last(proxies[1].aabb), (1 << 15) - 1);
    }
}
