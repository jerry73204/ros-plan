use crate::utils::arc_rwlock::ArcRwLock;
use indexmap::IndexMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use stable_vec::StableVec;
use std::sync::Arc;

use super::{Shared, SharedTableReadGuard};

#[derive(Debug)]
pub struct SharedTable<T> {
    inner: Arc<SharedTableInner<T>>,
}

impl<T> SharedTable<T> {
    pub fn new(name: &str) -> Self {
        Self {
            inner: Arc::new(SharedTableInner {
                name: name.to_string(),
                table: ArcRwLock::new(StableVec::new()),
            }),
        }
    }

    pub fn insert(&self, value: T) -> Shared<T> {
        let entry = ArcRwLock::new(value);
        let mut guard = self.inner.table.write();
        let id = guard.push(entry);
        let ref_ = Arc::downgrade(&self.inner);
        Shared {
            id,
            inner_weak: Some(ref_),
            tab_name: None,
        }
    }

    pub fn get(&self, id: usize) -> Option<Shared<T>> {
        let tab_guard = self.inner.table.read();
        tab_guard.get(id)?;

        Some(Shared {
            id,
            inner_weak: Some(Arc::downgrade(&self.inner)),
            tab_name: None,
        })
    }

    pub fn read(&self) -> SharedTableReadGuard<T> {
        SharedTableReadGuard {
            tab_guard: self.inner.table.read(),
            inner_arc: self.inner.clone(),
        }
    }

    pub fn contains(&self, value: &Shared<T>) -> bool {
        if !std::ptr::eq(Arc::as_ptr(&self.inner), value.inner_weak().as_ptr()) {
            return false;
        }
        let tab_guard = self.inner.table.read();
        tab_guard.get(value.id).is_some()
    }

    pub fn bind(&self, shared: &mut Shared<T>) {
        let Shared {
            id,
            inner_weak,
            tab_name,
        } = shared;

        let tab_name = match (inner_weak.is_some(), tab_name.take()) {
            (false, Some(tab_name)) => tab_name,
            (true, None) => panic!("the reference is already bound to the table"),
            _ => unreachable!(),
        };
        assert_eq!(
            self.inner.name, tab_name,
            "cannot bind a reference bound to {tab_name} with the table name {}",
            self.inner.name
        );

        let guard = self.inner.table.read();
        assert!(
            guard.get(*id).is_some(),
            "unable to bind the reference with index {id} to the table"
        );
        *inner_weak = Some(Arc::downgrade(&self.inner));
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
        let vec = self.inner.table.read();
        let guards: Vec<_> = vec.iter().map(|(k, v)| (k, v.read())).collect();
        let vec: IndexMap<usize, _> = guards.iter().map(|(k, v)| (*k, &**v)).collect();
        SerializedSharedTable {
            name: self.inner.name.clone(),
            table: vec,
        }
        .serialize(serializer)
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
        let SerializedSharedTable { name, table: map } =
            SerializedSharedTable::<T>::deserialize(deserializer)?;

        let Some(max_id) = map.keys().copied().max() else {
            return Ok(Self {
                inner: Arc::new(SharedTableInner {
                    name,
                    table: ArcRwLock::new(StableVec::new()),
                }),
            });
        };

        let mut vec = StableVec::with_capacity(max_id + 1);
        for (k, v) in map {
            vec.insert(k, ArcRwLock::from(v));
        }
        Ok(Self {
            inner: Arc::new(SharedTableInner {
                name,
                table: ArcRwLock::new(vec),
            }),
        })
    }
}

#[derive(Debug)]
pub(super) struct SharedTableInner<T> {
    pub(super) name: String,
    pub(super) table: ArcRwLock<StableVec<ArcRwLock<T>>>,
}

#[derive(Debug, Serialize, Deserialize)]
struct SerializedSharedTable<T> {
    pub name: String,
    pub table: IndexMap<usize, T>,
}
