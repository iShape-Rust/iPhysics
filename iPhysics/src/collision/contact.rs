use crate::body::BodyId;
use crate::geometry::{GeometryPoint, UnitVector};
use crate::quantity::Length;
use core::num::NonZeroU32;

/// `normal` points from `body_a` toward `body_b`; the solver response applied
/// to `body_a` therefore acts in the opposite direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Contact {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub point: GeometryPoint,
    pub normal: UnitVector,
    pub penetration: Length,
    pub(crate) key: ContactKey,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) enum ColliderFeature {
    Circle,
    ConvexVertex(u8),
    ConvexEdge(u8),
}

/// Compact contact metadata used by collision detection and the solver.
///
/// ```text
/// 31       30       24        16         8         4         0
/// +--------+--------+---------+----------+---------+---------+
/// | cached | correct| unused  | part B   | part A  | feat. B | feat. A
/// +--------+--------+---------+----------+---------+---------+
/// ```
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) struct ContactKey(u32);

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) struct CacheContactKey(NonZeroU32);

impl ContactKey {
    const FEATURE_MASK: u32 = 0x0f;
    const FEATURE_B_SHIFT: u32 = 4;
    const PART_A_SHIFT: u32 = 8;
    const PART_B_SHIFT: u32 = 16;
    const PART_MASK: u32 = 0xff;
    const FEATURES_MASK: u32 = 0xff;
    const CORRECT_POSITION: u32 = 1 << 30;
    const CACHEABLE: u32 = 1 << 31;
    const FLAGS: u32 = Self::CORRECT_POSITION | Self::CACHEABLE;

    #[inline(always)]
    pub(crate) const fn new(feature_a: ColliderFeature, feature_b: ColliderFeature) -> Self {
        Self(
            Self::CACHEABLE
                | Self::encode_feature(feature_a)
                | (Self::encode_feature(feature_b) << Self::FEATURE_B_SHIFT),
        )
    }

    #[inline(always)]
    pub(crate) fn with_parts(self, part_a: Option<usize>, part_b: Option<usize>) -> Self {
        let base = self.0 & (Self::FLAGS | Self::FEATURES_MASK);
        let (Some(part_a), Some(part_b)) = (Self::encode_part(part_a), Self::encode_part(part_b))
        else {
            return Self(base & !Self::CACHEABLE);
        };
        Self(
            base
                | (part_a << Self::PART_A_SHIFT)
                | (part_b << Self::PART_B_SHIFT),
        )
    }

    #[inline(always)]
    pub(crate) const fn flipped(self) -> Self {
        let feature_a = self.0 & Self::FEATURE_MASK;
        let feature_b = (self.0 >> Self::FEATURE_B_SHIFT) & Self::FEATURE_MASK;
        let part_a = (self.0 >> Self::PART_A_SHIFT) & Self::PART_MASK;
        let part_b = (self.0 >> Self::PART_B_SHIFT) & Self::PART_MASK;
        Self(
            (self.0 & Self::FLAGS)
                | feature_b
                | (feature_a << Self::FEATURE_B_SHIFT)
                | (part_b << Self::PART_A_SHIFT)
                | (part_a << Self::PART_B_SHIFT),
        )
    }

    #[inline(always)]
    pub(crate) const fn with_correct_position(self, correct: bool) -> Self {
        if correct {
            Self(self.0 | Self::CORRECT_POSITION)
        } else {
            Self(self.0 & !Self::CORRECT_POSITION)
        }
    }

    #[inline(always)]
    pub(crate) const fn correct_position(self) -> bool {
        self.0 & Self::CORRECT_POSITION != 0
    }

    #[inline(always)]
    #[cfg(test)]
    pub(crate) const fn feature_a(self) -> ColliderFeature {
        Self::decode_feature((self.0 & Self::FEATURE_MASK) as u8)
    }

    #[inline(always)]
    #[cfg(test)]
    pub(crate) const fn feature_b(self) -> ColliderFeature {
        Self::decode_feature(
            ((self.0 >> Self::FEATURE_B_SHIFT) & Self::FEATURE_MASK) as u8,
        )
    }

    #[inline(always)]
    #[cfg(test)]
    pub(crate) const fn part_a(self) -> Option<usize> {
        Self::decode_part(((self.0 >> Self::PART_A_SHIFT) & Self::PART_MASK) as u8)
    }

    #[inline(always)]
    #[cfg(test)]
    pub(crate) const fn part_b(self) -> Option<usize> {
        Self::decode_part(((self.0 >> Self::PART_B_SHIFT) & Self::PART_MASK) as u8)
    }

    #[inline(always)]
    pub(crate) const fn cache_key(self) -> Option<CacheContactKey> {
        if self.0 & Self::CACHEABLE == 0 {
            None
        } else {
            match NonZeroU32::new(self.0 & !Self::CORRECT_POSITION) {
                Some(raw) => Some(CacheContactKey(raw)),
                None => unreachable!(),
            }
        }
    }

    #[inline(always)]
    const fn encode_feature(feature: ColliderFeature) -> u32 {
        match feature {
            ColliderFeature::Circle => 1,
            ColliderFeature::ConvexVertex(index) => {
                debug_assert!(index < 6);
                2 + index as u32
            }
            ColliderFeature::ConvexEdge(index) => {
                debug_assert!(index < 6);
                8 + index as u32
            }
        }
    }

    #[inline(always)]
    #[cfg(test)]
    const fn decode_feature(code: u8) -> ColliderFeature {
        match code {
            1 => ColliderFeature::Circle,
            2..=7 => ColliderFeature::ConvexVertex(code - 2),
            8..=13 => ColliderFeature::ConvexEdge(code - 8),
            _ => unreachable!(),
        }
    }

    #[inline(always)]
    fn encode_part(part: Option<usize>) -> Option<u32> {
        match part {
            None => Some(0),
            Some(index) if index < u8::MAX as usize => Some(index as u32 + 1),
            Some(_) => None,
        }
    }

    #[inline(always)]
    #[cfg(test)]
    const fn decode_part(code: u8) -> Option<usize> {
        if code == 0 {
            None
        } else {
            Some(code as usize - 1)
        }
    }
}

impl Contact {
    /// Reverses the contact endpoints while preserving the same geometry.
    #[inline(always)]
    pub(crate) fn flipped(self) -> Self {
        Self {
            body_a: self.body_b,
            body_b: self.body_a,
            normal: -self.normal,
            key: self.key.flipped(),
            ..self
        }
    }

    #[inline(always)]
    pub(crate) fn with_parts(mut self, part_a: Option<usize>, part_b: Option<usize>) -> Self {
        self.key = self.key.with_parts(part_a, part_b);
        self
    }

    #[inline(always)]
    #[cfg(test)]
    pub(crate) const fn feature_a(self) -> ColliderFeature {
        self.key.feature_a()
    }

    #[inline(always)]
    #[cfg(test)]
    pub(crate) const fn feature_b(self) -> ColliderFeature {
        self.key.feature_b()
    }

    #[inline(always)]
    #[cfg(test)]
    pub(crate) const fn part_a(self) -> Option<usize> {
        self.key.part_a()
    }

    #[inline(always)]
    #[cfg(test)]
    pub(crate) const fn part_b(self) -> Option<usize> {
        self.key.part_b()
    }
}

/// Fixed-capacity contact manifold produced by one collider pair.
///
/// Curved contacts have one point. Two overlapping polygon faces keep both
/// ends of their clipped overlap so the solver can balance torque.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ContactManifold {
    first: Contact,
    second: Option<Contact>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_key_round_trips_and_flips_features_and_parts() {
        let key = ContactKey::new(
            ColliderFeature::ConvexVertex(4),
            ColliderFeature::ConvexEdge(2),
        )
        .with_parts(Some(7), Some(11));

        assert_eq!(key.feature_a(), ColliderFeature::ConvexVertex(4));
        assert_eq!(key.feature_b(), ColliderFeature::ConvexEdge(2));
        assert_eq!(key.part_a(), Some(7));
        assert_eq!(key.part_b(), Some(11));
        assert!(key.cache_key().is_some());

        let corrected = key.with_correct_position(true);
        assert!(corrected.correct_position());
        assert_eq!(corrected.cache_key(), key.cache_key());

        let flipped = corrected.flipped();
        assert!(flipped.correct_position());
        assert_eq!(flipped.feature_a(), ColliderFeature::ConvexEdge(2));
        assert_eq!(flipped.feature_b(), ColliderFeature::ConvexVertex(4));
        assert_eq!(flipped.part_a(), Some(11));
        assert_eq!(flipped.part_b(), Some(7));
    }

    #[test]
    fn oversized_composite_part_only_disables_caching() {
        let key = ContactKey::new(ColliderFeature::Circle, ColliderFeature::Circle)
            .with_parts(None, Some(u8::MAX as usize));

        assert_eq!(key.feature_a(), ColliderFeature::Circle);
        assert_eq!(key.feature_b(), ColliderFeature::Circle);
        assert_eq!(key.part_a(), None);
        assert_eq!(key.part_b(), None);
        assert!(key.cache_key().is_none());
    }
}

impl ContactManifold {
    #[inline(always)]
    pub(crate) const fn one(contact: Contact) -> Self {
        Self {
            first: contact,
            second: None,
        }
    }

    #[inline(always)]
    pub(crate) const fn two(first: Contact, second: Contact) -> Self {
        Self {
            first,
            second: Some(second),
        }
    }

    #[cfg(test)]
    #[inline(always)]
    pub(crate) const fn first(self) -> Contact {
        self.first
    }

    #[inline(always)]
    pub(crate) fn into_contacts(self) -> impl Iterator<Item = Contact> {
        [Some(self.first), self.second].into_iter().flatten()
    }

    #[inline(always)]
    pub(crate) fn with_parts(self, part_a: Option<usize>, part_b: Option<usize>) -> Self {
        Self {
            first: self.first.with_parts(part_a, part_b),
            second: self
                .second
                .map(|contact| contact.with_parts(part_a, part_b)),
        }
    }
}
