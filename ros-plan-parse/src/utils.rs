use crate::error::Error;
use parking_lot::{
    ArcRwLockReadGuard, ArcRwLockWriteGuard, RawRwLock, RwLock, RwLockReadGuard, RwLockWriteGuard,
};
use serde::{
    ser::{SerializeMap, SerializeStruct},
    Deserialize, Serialize, Serializer,
};
use stable_vec::StableVec;
use std::{
    ffi::OsString,
    fs,
    os::unix::ffi::OsStringExt,
    path::{Path, PathBuf},
    process::Command,
    sync::{Arc, Weak},
};

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
        let addr = Arc::as_ptr(&self.ref_) as usize;
        let addr_text = format!("arc@{addr:x}");

        let guard = self.read();
        let mut struct_ = serializer.serialize_struct("Arc", 2)?;
        struct_.serialize_field("addr", &addr_text)?;
        struct_.serialize_field("data", &*guard)?;
        struct_.end()
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
}

impl<T> Serialize for WeakRwLock<T>
where
    T: Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let addr = self.upgrade().map(|arc| {
            let ptr = Arc::as_ptr(&arc.ref_);
            let addr = ptr as usize;
            format!("weak@{addr:x}")
        });
        addr.serialize(serializer)
    }
}

pub fn read_toml_file<T, P>(path: P) -> Result<T, Error>
where
    T: for<'de> Deserialize<'de>,
    P: AsRef<Path>,
{
    let path = path.as_ref();
    let text = fs::read_to_string(path).map_err(|error| Error::OpenPlanFileError {
        path: path.to_owned(),
        error,
    })?;
    let data = toml::from_str(&text).map_err(|error| Error::ParsePlanFileError {
        path: path.to_owned(),
        error,
    })?;
    Ok(data)
}

pub fn find_pkg_dir(pkg: &str) -> Result<PathBuf, Error> {
    let output = match Command::new("ros2").args(["pkg", "prefix", pkg]).output() {
        Ok(path) => path,
        Err(err) => {
            return Err(Error::PackageResolutionError {
                pkg: pkg.to_string(),
                message: format!("{err}"),
            })
        }
    };

    if !output.status.success() {
        let err = String::from_utf8_lossy(&output.stderr);
        return Err(Error::PackageResolutionError {
            pkg: pkg.to_string(),
            message: format!("Command `ros2 prefix {pkg}` failed with message: {err}"),
        });
    }

    // TODO: This line only works on UNIX.
    let pkg_dir: PathBuf = {
        let mut out = output.stdout;
        assert_eq!(out.pop(), Some(b'\n'));
        OsString::from_vec(out.to_vec()).into()
    };

    Ok(pkg_dir)
}

pub fn find_plan_file_from_pkg(pkg: &str, file: &str) -> Result<PathBuf, Error> {
    let pkg_dir = find_pkg_dir(pkg)?;
    let plan_path = pkg_dir.join("share").join(pkg).join("plan").join(file);
    Ok(plan_path)
}

pub mod serde_stable_vec {
    use indexmap::IndexMap;
    use serde::{ser::SerializeMap, Deserialize, Deserializer, Serialize, Serializer};
    use stable_vec::StableVec;

    pub fn serialize<T, S>(vec: &StableVec<T>, serializer: S) -> Result<S::Ok, S::Error>
    where
        T: Serialize,
        S: Serializer,
    {
        let len = vec.iter().count();
        let mut map = serializer.serialize_map(Some(len))?;

        for (k, v) in vec.iter() {
            map.serialize_entry(&k, v)?;
        }

        map.end()
    }

    pub fn deserialize<'de, T, D>(deserializer: D) -> Result<StableVec<T>, D::Error>
    where
        T: Deserialize<'de>,
        D: Deserializer<'de>,
    {
        let map: IndexMap<usize, T> = IndexMap::deserialize(deserializer)?;

        let mut vec = StableVec::with_capacity(map.len());
        for (k, v) in map {
            vec.insert(k, v);
        }

        Ok(vec)
    }
}
