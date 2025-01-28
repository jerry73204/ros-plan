use super::{table::SharedTableInner, Shared};
use crate::utils::arc_rwlock::ArcRwLock;
use parking_lot::{ArcRwLockReadGuard, RawRwLock, RwLockReadGuard, RwLockWriteGuard};
use stable_vec::StableVec;
use std::sync::Weak;

#[derive(Debug)]
pub struct Owned<T> {
    pub(super) id: usize,
    pub(super) inner_weak: Weak<SharedTableInner<T>>,
    pub(super) entry_arc: ArcRwLock<T>,

    /// The read guard locks the table to prevent the entry from being
    /// removed from the table.
    pub(super) _tab_guard: ArcRwLockReadGuard<RawRwLock, StableVec<ArcRwLock<T>>>,
}

impl<T> Owned<T> {
    pub fn read(&self) -> RwLockReadGuard<T> {
        self.entry_arc.read()
    }

    pub fn write(&self) -> RwLockWriteGuard<T> {
        self.entry_arc.write()
    }

    pub fn downgrade(&self) -> Shared<T> {
        Shared {
            id: self.id,
            inner_weak: Some(self.inner_weak.clone()),
            tab_name: None,
        }
    }

    pub fn id(&self) -> usize {
        self.id
    }
}
