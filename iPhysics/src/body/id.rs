/// Stable deterministic body identifier.
///
/// IDs, rather than storage indices, define collision-pair ordering and must
/// therefore be assigned identically by all multiplayer peers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(u32);

impl BodyId {
    #[inline(always)]
    pub const fn new(raw: u32) -> Self {
        Self(raw)
    }

    #[inline(always)]
    pub const fn raw(self) -> u32 {
        self.0
    }
}
