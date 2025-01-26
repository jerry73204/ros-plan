pub mod arc_rwlock;
pub mod shared_table;

use crate::error::Error;
use serde::Deserialize;
use std::{
    ffi::OsString,
    fs,
    os::unix::ffi::OsStringExt,
    path::{Path, PathBuf},
    process::Command,
};

pub fn read_yaml_file<T, P>(path: P) -> Result<T, Error>
where
    T: for<'de> Deserialize<'de>,
    P: AsRef<Path>,
{
    let path = path.as_ref();
    let text = fs::read_to_string(path).map_err(|error| Error::ReadFileError {
        path: path.to_owned(),
        error,
    })?;
    let data = serde_yaml::from_str(&text).map_err(|error| Error::ParsePlanFileError {
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
