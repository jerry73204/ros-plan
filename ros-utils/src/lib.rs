pub mod error;
pub mod node_cmdline;

pub use crate::error::PackageResolutionError;
pub use node_cmdline::NodeCommandLine;

use std::os::unix::ffi::OsStringExt;
use std::{ffi::OsString, path::PathBuf, process::Command};

pub fn find_pkg_dir(pkg: &str) -> Result<PathBuf, PackageResolutionError> {
    let output = match Command::new("ros2").args(["pkg", "prefix", pkg]).output() {
        Ok(path) => path,
        Err(err) => {
            return Err(PackageResolutionError {
                pkg: pkg.to_string(),
                message: format!("{err}"),
            })
        }
    };

    if !output.status.success() {
        let err = String::from_utf8_lossy(&output.stderr);
        return Err(PackageResolutionError {
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

pub fn find_pkg_exec(pkg: &str, exec: &str) -> Result<PathBuf, PackageResolutionError> {
    let prefix = find_pkg_dir(pkg)?;
    let path = prefix.join("lib").join(pkg).join(exec);
    Ok(path)
}

pub fn find_pkg_config(pkg: &str, config_file: &str) -> Result<PathBuf, PackageResolutionError> {
    let prefix = find_pkg_dir(pkg)?;
    let path = prefix
        .join("share")
        .join(pkg)
        .join("config")
        .join(config_file);
    Ok(path)
}

pub fn find_pkg_launch(pkg: &str, launch_file: &str) -> Result<PathBuf, PackageResolutionError> {
    let prefix = find_pkg_dir(pkg)?;
    let path = prefix
        .join("share")
        .join(pkg)
        .join("launch")
        .join(launch_file);
    Ok(path)
}
