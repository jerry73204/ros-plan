use super::arc_rwlock::{ArcRwLock, WeakRwLock};
use parking_lot::{ArcRwLockReadGuard, RawRwLock, RwLockReadGuard, RwLockWriteGuard};
use serde::{ser::SerializeMap, Serialize, Serializer};
use stable_vec::StableVec;

#[derive(Debug)]
pub struct SharedTable<T> {
    inner: ArcRwLock<StableVec<ArcRwLock<T>>>,
}

impl<T> SharedTable<T> {
    pub fn insert(&self, value: T) -> Shared<T> {
        let entry = ArcRwLock::new(value);
        let mut guard = self.inner.write();
        let id = guard.push(entry);
        let ref_ = self.inner.clone().downgrade();
        Shared { id, tab_weak: ref_ }
    }

    pub fn get(&self, id: usize) -> Option<Owned<T>> {
        let tab_guard = self.inner.read_arc();
        tab_guard.get(id)?;
        Some(Owned {
            id,
            tab_guard,
            tab_weak: self.inner.downgrade(),
        })
    }
}

impl<T> Default for SharedTable<T> {
    fn default() -> Self {
        Self {
            inner: ArcRwLock::new(StableVec::new()),
        }
    }
}

impl<T> Serialize for SharedTable<T>
where
    T: Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let vec = self.inner.read();
        let len = vec.iter().count();
        let mut map = serializer.serialize_map(Some(len))?;

        for (key, rwlock) in vec.iter() {
            let value = rwlock.read();
            map.serialize_entry(&key, &*value)?;
        }

        map.end()
    }
}

#[derive(Debug)]
pub struct Owned<T> {
    id: usize,
    tab_guard: ArcRwLockReadGuard<RawRwLock, StableVec<ArcRwLock<T>>>,
    tab_weak: WeakRwLock<StableVec<ArcRwLock<T>>>,
}

impl<T> Owned<T> {
    pub fn read(&self) -> RwLockReadGuard<T> {
        let entry = &self.tab_guard[self.id];
        entry.read()
    }

    pub fn write(&self) -> RwLockWriteGuard<T> {
        let entry = &self.tab_guard[self.id];
        entry.write()
    }

    pub fn downgrade(self) -> Shared<T> {
        Shared {
            id: self.id,
            tab_weak: self.tab_weak,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Shared<T> {
    id: usize,
    tab_weak: WeakRwLock<StableVec<ArcRwLock<T>>>,
}

impl<T> Shared<T> {
    pub fn new_null(id: usize) -> Self {
        Self {
            id,
            tab_weak: WeakRwLock::new_null(),
        }
    }

    pub fn upgrade(&self) -> Option<Owned<T>> {
        let tab_arc = self.tab_weak.upgrade()?;
        let tab_guard = tab_arc.read_arc();
        Some(Owned {
            id: self.id,
            tab_guard,
            tab_weak: self.tab_weak.clone(),
        })
    }
}

impl<T> Serialize for Shared<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.id.serialize(serializer)
    }
}
