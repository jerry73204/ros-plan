use super::arc_rwlock::{ArcRwLock, WeakRwLock};
use indexmap::IndexMap;
use parking_lot::{ArcRwLockReadGuard, RawRwLock, RwLockReadGuard, RwLockWriteGuard};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use stable_vec::StableVec;

#[derive(Debug)]
pub struct SharedTable<T> {
    name: String,
    inner: ArcRwLock<StableVec<ArcRwLock<T>>>,
}

impl<T> SharedTable<T> {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            inner: ArcRwLock::new(StableVec::new()),
        }
    }

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

    pub fn read(&self) -> SharedTableReadGuard<T> {
        SharedTableReadGuard {
            tab_guard: self.inner.read_arc(),
            tab_arc: self.inner.clone(),
        }
    }

    pub fn contains(&self, value: &Shared<T>) -> bool {
        if !std::ptr::eq(self.inner.as_ptr(), value.tab_weak.as_ptr()) {
            return false;
        }
        let guard = self.inner.read();
        guard.get(value.id).is_some()
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
                name: "TODO".to_string(),
                inner: ArcRwLock::from(StableVec::new()),
            });
        };

        let mut vec = StableVec::with_capacity(max_id + 1);
        for (k, v) in map {
            vec.insert(k, ArcRwLock::from(v));
        }
        Ok(Self {
            name: "TODO".to_string(),
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

    pub fn with_read<R, F>(&self, f: F) -> R
    where
        F: FnOnce(RwLockReadGuard<T>) -> R,
    {
        self.tab_weak.with_read(|tab_guard| {
            let entry_guard = tab_guard[self.id].read();
            f(entry_guard)
        })
    }

    pub fn with_write<R, F>(&self, f: F) -> R
    where
        F: FnOnce(RwLockWriteGuard<T>) -> R,
    {
        self.tab_weak.with_read(|tab_guard| {
            let entry_guard = tab_guard[self.id].write();
            f(entry_guard)
        })
    }

    #[must_use]
    pub fn ptr_eq(&self, other: &Self) -> bool {
        let ptr_eq = std::ptr::eq(self.tab_weak.as_ptr(), other.tab_weak.as_ptr());
        let id_eq = self.id == other.id;
        ptr_eq && id_eq
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

#[derive(Debug)]
pub struct SharedTableReadGuard<T> {
    tab_guard: ArcRwLockReadGuard<RawRwLock, StableVec<ArcRwLock<T>>>,
    tab_arc: ArcRwLock<StableVec<ArcRwLock<T>>>,
}

impl<T> SharedTableReadGuard<T> {
    pub fn iter(&self) -> impl Iterator<Item = (usize, Owned<T>)> + '_ {
        self.tab_guard.iter().map(|(id, arc)| {
            let owned = Owned {
                id,
                tab_weak: self.tab_arc.downgrade(),
                entry_arc: arc.clone(),
                _tab_guard: self.tab_arc.read_arc(),
            };
            (id, owned)
        })
    }
}
