use super::{owned::Owned, table::SharedTableInner};
use parking_lot::{RwLockReadGuard, RwLockWriteGuard};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};
use std::sync::{Arc, Weak};

#[derive(Debug)]
pub struct Shared<T> {
    pub(super) id: usize,
    pub(super) inner_weak: Option<Weak<SharedTableInner<T>>>,
    pub(super) tab_name: Option<String>,
}

impl<T> Shared<T> {
    pub fn new_null(id: usize, tab_name: &str) -> Self {
        Self {
            id,
            inner_weak: None,
            tab_name: Some(tab_name.to_string()),
        }
    }

    pub fn id(&self) -> usize {
        self.id
    }

    pub fn with_read<R, F>(&self, f: F) -> R
    where
        F: FnOnce(RwLockReadGuard<T>) -> R,
    {
        let inner_arc = self.inner_arc();
        let tab_guard = inner_arc.table.read();
        let entry_guard = tab_guard[self.id].read();
        f(entry_guard)
    }

    pub fn with_write<R, F>(&self, f: F) -> R
    where
        F: FnOnce(RwLockWriteGuard<T>) -> R,
    {
        let inner_arc = self.inner_arc();
        let tab_guard = inner_arc.table.read();
        let entry_guard = tab_guard[self.id].write();
        f(entry_guard)
    }

    #[must_use]
    pub fn ptr_eq(&self, other: &Self) -> bool {
        let ptr_eq = std::ptr::eq(self.inner_weak().as_ptr(), other.inner_weak().as_ptr());
        let id_eq = self.id == other.id;
        ptr_eq && id_eq
    }

    pub fn upgrade(&self) -> Option<Owned<T>> {
        let inner_arc = self.inner_arc();
        let tab_guard = inner_arc.table.read_arc();
        let entry_arc = tab_guard[self.id].clone();

        Some(Owned {
            id: self.id,
            inner_weak: self.inner_weak().clone(),
            entry_arc,
            _tab_guard: tab_guard,
        })
    }

    pub(super) fn inner_weak(&self) -> &Weak<SharedTableInner<T>> {
        self.inner_weak.as_ref().unwrap()
    }

    pub(super) fn inner_arc(&self) -> Arc<SharedTableInner<T>> {
        self.inner_weak().upgrade().unwrap()
    }
}

impl<T> Clone for Shared<T> {
    fn clone(&self) -> Self {
        Self {
            id: self.id,
            inner_weak: self.inner_weak.clone(),
            tab_name: self.tab_name.clone(),
        }
    }
}

impl<T> Serialize for Shared<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let inner_arc = self.inner_arc();
        format!("{}[{}]", inner_arc.name, self.id).serialize(serializer)
    }
}

impl<'de, T> Deserialize<'de> for Shared<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let text = String::deserialize(deserializer)?;
        let Some((tab_name, text)) = text.split_once('[') else {
            return Err(D::Error::custom("invalid reference format"));
        };
        let Some(id_text) = text.strip_suffix(']') else {
            return Err(D::Error::custom("invalid reference format"));
        };
        let id: usize = id_text
            .parse()
            .map_err(|_| D::Error::custom("invalid reference format"))?;
        Ok(Self {
            id,
            inner_weak: None,
            tab_name: Some(tab_name.to_string()),
        })
    }
}
