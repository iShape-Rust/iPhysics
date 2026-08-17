/// Stable body identifier assigned by the game or networking layer.
///
/// Storage order is derived from this value, but the ID itself contains no
/// index or generation and remains unchanged while the body exists.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(u64);

impl BodyId {
    #[inline(always)]
    pub const fn new(raw: u64) -> Self {
        Self(raw)
    }

    #[inline(always)]
    pub const fn raw(self) -> u64 {
        self.0
    }

    #[inline(always)]
    pub const fn from_raw(raw: u64) -> Self {
        Self(raw)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stable_id_is_eight_bytes_and_round_trips() {
        let id = BodyId::new(0x1234_5678_9abc);

        assert_eq!(core::mem::size_of::<BodyId>(), 8);
        assert_eq!(id.raw(), 0x1234_5678_9abc);
        assert_eq!(BodyId::from_raw(id.raw()), id);
    }
}
