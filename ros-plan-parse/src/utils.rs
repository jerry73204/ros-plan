pub mod arc_rwlock;
pub mod shared_table;
pub mod tree;

use crate::{
    error::Error,
    resource::{NodeOwned, Resource, Scope, ScopeTreeRef, SocketOwned},
};
use ros_plan_format::{ident::Ident, key::Key};
use serde::Deserialize;
use std::{
    ffi::OsString,
    fs,
    os::unix::ffi::OsStringExt,
    path::{Path, PathBuf},
    process::Command,
};

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

#[derive(Debug)]
pub enum ResolveNode {
    Node(NodeOwned),
    Socket(SocketOwned),
}

impl From<SocketOwned> for ResolveNode {
    fn from(v: SocketOwned) -> Self {
        Self::Socket(v)
    }
}

impl From<NodeOwned> for ResolveNode {
    fn from(v: NodeOwned) -> Self {
        Self::Node(v)
    }
}

pub fn resolve_node_entity(
    resource: &Resource,
    current: ScopeTreeRef,
    key: &Key,
) -> Option<ResolveNode> {
    let (target, node_name) = resolve_entity(resource, &current, key)?;

    let guard = target.read();
    let resolve: ResolveNode = match &guard.value {
        Scope::PlanFile(ctx) => {
            let shared = ctx.socket_map.get(node_name)?;
            shared.upgrade().unwrap().into()
        }
        Scope::Group(ctx) => {
            let node_arc = ctx.node_map.get(node_name)?;
            node_arc.upgrade().unwrap().into()
        }
    };

    Some(resolve)
}

pub fn resolve_entity<'a>(
    resource: &Resource,
    current: &ScopeTreeRef,
    key: &'a Key,
) -> Option<(ScopeTreeRef, &'a Ident)> {
    let (prefix, entity) = key.split_parent();
    let entity_name = entity.expect("the key should not be empty");

    let target = match prefix {
        None => current.clone(),
        Some(prefix) => resolve_scope(resource, current, prefix)?,
    };
    Some((target, entity_name))
}

pub fn resolve_scope(
    resource: &Resource,
    current: &ScopeTreeRef,
    key: &Key,
) -> Option<ScopeTreeRef> {
    let found = if key.is_empty() {
        current.clone()
    } else if key.is_absolute() {
        resource.find_scope(key)?
    } else {
        current.find(key)?
    };
    Some(found)
}
