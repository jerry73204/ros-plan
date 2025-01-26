use parking_lot::{
    ArcRwLockReadGuard, ArcRwLockWriteGuard, RawRwLock, RwLock, RwLockReadGuard, RwLockWriteGuard,
};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::sync::{Arc, Weak};

#[derive(Debug)]
pub struct ArcRwLock<T> {
    ref_: Arc<RwLock<T>>,
}

impl<T> ArcRwLock<T> {
    pub fn new(value: T) -> Self {
        Self {
            ref_: Arc::new(RwLock::new(value)),
        }
    }

    pub fn read(&self) -> RwLockReadGuard<T> {
        self.ref_.read()
    }

    pub fn read_arc(&self) -> ArcRwLockReadGuard<RawRwLock, T> {
        RwLock::read_arc(&self.ref_)
    }

    pub fn write(&self) -> RwLockWriteGuard<T> {
        self.ref_.write()
    }

    pub fn write_arc(&self) -> ArcRwLockWriteGuard<RawRwLock, T> {
        RwLock::write_arc(&self.ref_)
    }

    pub fn downgrade(&self) -> WeakRwLock<T> {
        WeakRwLock {
            ref_: Arc::downgrade(&self.ref_),
        }
    }

    pub fn into_arc(self) -> Arc<RwLock<T>> {
        self.ref_
    }

    pub fn as_ptr(&self) -> *const RwLock<T> {
        Arc::as_ptr(&self.ref_)
    }
}

impl<T> Clone for ArcRwLock<T> {
    fn clone(&self) -> Self {
        Self {
            ref_: self.ref_.clone(),
        }
    }
}

impl<T> From<T> for ArcRwLock<T> {
    fn from(ctx: T) -> Self {
        Self {
            ref_: Arc::new(RwLock::new(ctx)),
        }
    }
}

impl<T> Serialize for ArcRwLock<T>
where
    T: Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let guard = self.read();
        guard.serialize(serializer)
        // let addr = Arc::as_ptr(&self.ref_) as usize;
        // let addr_text = format!("arc@{addr:x}");

        // let guard = self.read();
        // let mut struct_ = serializer.serialize_struct("Arc", 2)?;
        // struct_.serialize_field("addr", &addr_text)?;
        // struct_.serialize_field("data", &*guard)?;
        // struct_.end()
    }
}

impl<'de, T> Deserialize<'de> for ArcRwLock<T>
where
    T: Deserialize<'de>,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let inner = T::deserialize(deserializer)?;
        Ok(inner.into())
    }
}

#[derive(Debug, Clone)]
pub struct WeakRwLock<T> {
    ref_: Weak<RwLock<T>>,
}

impl<T> WeakRwLock<T> {
    pub fn new_null() -> Self {
        Self { ref_: Weak::new() }
    }

    pub fn upgrade(&self) -> Option<ArcRwLock<T>> {
        Some(ArcRwLock {
            ref_: self.ref_.upgrade()?,
        })
    }

    pub fn with_read<R, F>(&self, f: F) -> R
    where
        F: FnOnce(RwLockReadGuard<T>) -> R,
    {
        let arc = self.ref_.upgrade().unwrap();
        let guard = arc.read();
        f(guard)
    }

    pub fn with_write<R, F>(&self, f: F) -> R
    where
        F: FnOnce(RwLockWriteGuard<T>) -> R,
    {
        let arc = self.ref_.upgrade().unwrap();
        let guard = arc.write();
        f(guard)
    }

    pub fn as_ptr(&self) -> *const RwLock<T> {
        self.ref_.as_ptr()
    }
}

// impl<T> Serialize for WeakRwLock<T>
// where
//     T: Serialize,
// {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
//     where
//         S: Serializer,
//     {
//         let addr = self.upgrade().map(|arc| {
//             let ptr = Arc::as_ptr(&arc.ref_);
//             let addr = ptr as usize;
//             format!("weak@{addr:x}")
//         });
//         addr.serialize(serializer)
//     }
// }
