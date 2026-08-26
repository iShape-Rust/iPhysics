use crate::body::BodyId;
use crate::collision::ColliderFeature;

pub(super) const HOT_CONTACT_CAPACITY: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(super) struct ContactIdentity {
    pub(super) body_a: BodyId,
    pub(super) body_b: BodyId,
    pub(super) part_a: Option<usize>,
    pub(super) part_b: Option<usize>,
    pub(super) feature_a: ColliderFeature,
    pub(super) feature_b: ColliderFeature,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct HotContact {
    pub(super) identity: ContactIdentity,
    pub(super) normal_velocity_change_q10: u64,
    pub(super) tangent_velocity_change_q10: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct HotContacts {
    entries: [Option<HotContact>; HOT_CONTACT_CAPACITY],
}

impl HotContacts {
    pub(super) const EMPTY: Self = Self {
        entries: [None; HOT_CONTACT_CAPACITY],
    };

    #[inline]
    pub(super) fn find(self, identity: ContactIdentity) -> Option<HotContact> {
        self.entries
            .iter()
            .flatten()
            .copied()
            .find(|contact| contact.identity == identity)
    }

    pub(super) fn insert(&mut self, contact: HotContact) {
        if contact.normal_velocity_change_q10 == 0 {
            return;
        }

        if let Some(existing) = self
            .entries
            .iter_mut()
            .flatten()
            .find(|entry| entry.identity == contact.identity)
        {
            if contact.normal_velocity_change_q10 > existing.normal_velocity_change_q10 {
                *existing = contact;
            }
            return;
        }

        let mut insert_at = HOT_CONTACT_CAPACITY;
        for (index, entry) in self.entries.iter().enumerate() {
            if entry.is_none_or(|entry| {
                contact.normal_velocity_change_q10 > entry.normal_velocity_change_q10
            }) {
                insert_at = index;
                break;
            }
        }
        if insert_at == HOT_CONTACT_CAPACITY {
            return;
        }

        for index in (insert_at + 1..HOT_CONTACT_CAPACITY).rev() {
            self.entries[index] = self.entries[index - 1];
        }
        self.entries[insert_at] = Some(contact);
    }

    #[cfg(test)]
    pub(super) fn len(self) -> usize {
        self.entries.iter().flatten().count()
    }

    #[cfg(test)]
    pub(super) fn has_tangent(self) -> bool {
        self.entries
            .iter()
            .flatten()
            .any(|contact| contact.tangent_velocity_change_q10 != 0)
    }
}

impl Default for HotContacts {
    fn default() -> Self {
        Self::EMPTY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity(other: u64, part_b: Option<usize>, feature: u8) -> ContactIdentity {
        ContactIdentity {
            body_a: BodyId::new(1),
            body_b: BodyId::new(other),
            part_a: None,
            part_b,
            feature_a: ColliderFeature::ConvexVertex(feature),
            feature_b: ColliderFeature::ConvexEdge(0),
        }
    }

    #[test]
    fn matching_uses_bodies_parts_and_features() {
        let wanted = identity(2, Some(4), 1);
        let mut cache = HotContacts::EMPTY;
        cache.insert(HotContact {
            identity: wanted,
            normal_velocity_change_q10: 100,
            tangent_velocity_change_q10: -7,
        });

        assert_eq!(cache.find(wanted).unwrap().tangent_velocity_change_q10, -7);
        assert!(cache.find(identity(3, Some(4), 1)).is_none());
        assert!(cache.find(identity(2, Some(5), 1)).is_none());
        assert!(cache.find(identity(2, Some(4), 2)).is_none());
    }

    #[test]
    fn overflow_keeps_three_largest_normal_values() {
        let mut cache = HotContacts::EMPTY;
        for (other, normal) in [(2, 20), (3, 50), (4, 10), (5, 30)] {
            cache.insert(HotContact {
                identity: identity(other, None, 0),
                normal_velocity_change_q10: normal,
                tangent_velocity_change_q10: 0,
            });
        }

        assert_eq!(cache.len(), 3);
        assert!(cache.find(identity(2, None, 0)).is_some());
        assert!(cache.find(identity(3, None, 0)).is_some());
        assert!(cache.find(identity(4, None, 0)).is_none());
        assert!(cache.find(identity(5, None, 0)).is_some());
    }
}
