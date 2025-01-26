use super::arc_rwlock::{ArcRwLock, WeakRwLock};
use indexmap::IndexMap;
use parking_lot::{ArcRwLockReadGuard, RawRwLock, RwLockReadGuard, RwLockWriteGuard};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use stable_vec::StableVec;

#[derive(Debug)]
#[repr(transparent)]
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
        let entry_arc = tab_guard.get(id)?.clone();

        Some(Owned {
            id,
            tab_weak: self.inner.downgrade(),
            entry_arc,
            _tab_guard: tab_guard,
        })
    }

    pub fn read_inner(&self) -> RwLockReadGuard<'_, StableVec<ArcRwLock<T>>> {
        self.inner.read()
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
        let guards: Vec<_> = vec.iter().map(|(k, v)| (k, v.read())).collect();
        let refs: IndexMap<usize, _> = guards.iter().map(|(k, v)| (*k, &**v)).collect();
        refs.serialize(serializer)
    }
}

impl<'de, T> Deserialize<'de> for SharedTable<T>
where
    T: Deserialize<'de>,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let map = IndexMap::<usize, T>::deserialize(deserializer)?;
        let Some(max_id) = map.keys().copied().max() else {
            return Ok(Self {
                inner: ArcRwLock::from(StableVec::new()),
            });
        };

        let mut vec = StableVec::with_capacity(max_id + 1);
        for (k, v) in map {
            vec.insert(k, ArcRwLock::from(v));
        }
        Ok(Self {
            inner: ArcRwLock::from(vec),
        })
    }
}

#[derive(Debug)]
pub struct Owned<T> {
    id: usize,
    tab_weak: WeakRwLock<StableVec<ArcRwLock<T>>>,
    entry_arc: ArcRwLock<T>,

    /// The read guard locks the table to prevent the entry from being
    /// removed from the table.
    _tab_guard: ArcRwLockReadGuard<RawRwLock, StableVec<ArcRwLock<T>>>,
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
            tab_weak: self.tab_weak.clone(),
        }
    }

    pub fn id(&self) -> usize {
        self.id
    }
}

#[derive(Debug)]
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
        let entry_arc = tab_guard[self.id].clone();

        Some(Owned {
            id: self.id,
            tab_weak: self.tab_weak.clone(),
            entry_arc,
            _tab_guard: tab_guard,
        })
    }

    pub fn id(&self) -> usize {
        self.id
    }
}

impl<T> Clone for Shared<T> {
    fn clone(&self) -> Self {
        Self {
            id: self.id,
            tab_weak: self.tab_weak.clone(),
        }
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

impl<'de, T> Deserialize<'de> for Shared<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let id = usize::deserialize(deserializer)?;
        Ok(Self {
            id,
            tab_weak: WeakRwLock::new_null(),
        })
    }
}
