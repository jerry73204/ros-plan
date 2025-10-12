use std::sync::Arc;

use crate::utils::arc_rwlock::ArcRwLock;
use parking_lot::RwLockReadGuard;
use stable_vec::StableVec;

use super::{table::SharedTableInner, Shared};

#[derive(Debug)]
pub struct SharedTableReadGuard<'a, T> {
    pub(super) tab_guard: RwLockReadGuard<'a, StableVec<ArcRwLock<T>>>,
    pub(super) inner_arc: Arc<SharedTableInner<T>>,
}

impl<T> SharedTableReadGuard<'_, T> {
    pub fn iter(&self) -> impl Iterator<Item = (usize, Shared<T>)> + '_ {
        self.tab_guard.iter().map(|(id, _arc)| {
            let owned = Shared {
                id,
                inner_weak: Some(Arc::downgrade(&self.inner_arc)),
                tab_name: None,
            };
            (id, owned)
        })
    }
}
