use crate::error::Error;
use parking_lot::{RwLock, RwLockReadGuard, RwLockWriteGuard};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};
use std::{
    ffi::OsString,
    fs,
    os::unix::ffi::OsStringExt,
    path::{Path, PathBuf},
    process::Command,
    sync::{Arc, Weak},
};

#[derive(Debug)]
pub struct ArcRwLock<T>(Arc<RwLock<T>>);

impl<T> ArcRwLock<T> {
    pub fn read(&self) -> RwLockReadGuard<T> {
        self.0.read()
    }

    pub fn write(&self) -> RwLockWriteGuard<T> {
        self.0.write()
    }

    pub fn downgrade(&self) -> WeakRwLock<T> {
        WeakRwLock(Arc::downgrade(&self.0))
    }

    pub fn into_arc(self) -> Arc<RwLock<T>> {
        self.0
    }
}

impl<T> Clone for ArcRwLock<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T> From<T> for ArcRwLock<T> {
    fn from(ctx: T) -> Self {
        Self(Arc::new(RwLock::new(ctx)))
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
        let addr = Arc::as_ptr(&self.0) as usize;
        let addr_text = format!("arc@{addr:x}");

        let guard = self.read();
        let mut struct_ = serializer.serialize_struct("Arc", 2)?;
        struct_.serialize_field("addr", &addr_text)?;
        struct_.serialize_field("data", &*guard)?;
        struct_.end()
    }
}

#[derive(Debug, Clone)]
pub struct WeakRwLock<T>(Weak<RwLock<T>>);

impl<T> WeakRwLock<T> {
    pub fn upgrade(&self) -> Option<ArcRwLock<T>> {
        Some(ArcRwLock(self.0.upgrade()?))
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
            let ptr = Arc::as_ptr(&arc.0);
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
