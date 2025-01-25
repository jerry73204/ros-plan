pub mod arc_rwlock;
pub mod shared_table;

use crate::{
    error::Error,
    resource::Resource,
    scope::{NodeSocketOwned, NodeSocketShared, PlanSocketOwned, ScopeRefExt, ScopeShared},
};
use ros_plan_format::key::Key;
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
    let text = fs::read_to_string(path).map_err(|error| Error::OpenPlanFileError {
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

#[derive(Debug)]
pub enum FindSocket {
    Node(NodeSocketOwned),
    Plan(PlanSocketOwned),
}

impl FindSocket {
    pub fn to_node_pub_src(&self) -> Option<Vec<NodeSocketShared>> {
        Some(match self {
            FindSocket::Node(socket) => {
                if !socket.read().is_publication() {
                    return None;
                }
                vec![socket.downgrade()]
            }
            FindSocket::Plan(socket) => {
                let guard = socket.read();
                let inner = guard.as_publication()?;
                inner.src.clone().unwrap()
            }
        })
    }

    pub fn to_node_sub_dst(&self) -> Option<Vec<NodeSocketShared>> {
        Some(match self {
            FindSocket::Node(socket) => {
                if !socket.read().is_subscription() {
                    return None;
                }
                vec![socket.downgrade()]
            }
            FindSocket::Plan(socket) => {
                let guard = socket.read();
                let inner = guard.as_subscription()?;
                inner.dst.clone().unwrap()
            }
        })
    }

    pub fn to_node_server_listen(&self) -> Option<NodeSocketShared> {
        Some(match self {
            FindSocket::Node(socket) => {
                if !socket.read().is_server() {
                    return None;
                }
                socket.downgrade()
            }
            FindSocket::Plan(socket) => {
                let guard = socket.read();
                let inner = guard.as_server()?;
                inner.listen.clone().unwrap()
            }
        })
    }

    pub fn to_node_client_connect(&self) -> Option<Vec<NodeSocketShared>> {
        Some(match self {
            FindSocket::Node(socket) => {
                if !socket.read().is_client() {
                    return None;
                }
                vec![socket.downgrade()]
            }
            FindSocket::Plan(socket) => {
                let guard = socket.read();
                let inner = guard.as_client()?;
                inner.connect.clone().unwrap()
            }
        })
    }
}

impl From<NodeSocketOwned> for FindSocket {
    fn from(v: NodeSocketOwned) -> Self {
        Self::Node(v)
    }
}

impl From<PlanSocketOwned> for FindSocket {
    fn from(v: PlanSocketOwned) -> Self {
        Self::Plan(v)
    }
}

pub fn find_plan_or_node_socket(
    resource: &Resource,
    current: ScopeShared,
    socket_key: &Key,
) -> Option<FindSocket> {
    let output: FindSocket = if socket_key.is_absolute() {
        resource.find_node_socket(socket_key)?.into()
    } else if socket_key.is_relative() {
        let owned = current.upgrade().unwrap();
        let guard = owned.read();

        if let Some(socket) = guard.get_node_socket_recursive_bounded(socket_key) {
            socket.upgrade().unwrap().into()
        } else {
            let socket = guard.get_plan_socket_recursive_bounded(socket_key)?;
            socket.upgrade().unwrap().into()
        }
    } else {
        return None;
    };
    Some(output)
}

pub fn resolve_node_publication(
    resource: &Resource,
    current: ScopeShared,
    socket_key: &Key,
) -> Option<Vec<NodeSocketShared>> {
    let find_socket = find_plan_or_node_socket(resource, current, socket_key)?;
    let sockets = match find_socket {
        FindSocket::Node(socket) => {
            if !socket.read().is_publication() {
                return None;
            }
            vec![socket.downgrade()]
        }
        FindSocket::Plan(socket) => {
            let guard = socket.read();
            let inner = guard.as_publication()?;
            inner.src.clone().unwrap()
        }
    };
    Some(sockets)
}

pub fn resolve_node_subscription(
    resource: &Resource,
    current: ScopeShared,
    socket_key: &Key,
) -> Option<Vec<NodeSocketShared>> {
    let find_socket = find_plan_or_node_socket(resource, current, socket_key)?;
    let sockets = match find_socket {
        FindSocket::Node(socket) => {
            if !socket.read().is_subscription() {
                return None;
            }
            vec![socket.downgrade()]
        }
        FindSocket::Plan(socket) => {
            let guard = socket.read();
            let inner = guard.as_subscription()?;
            inner.dst.clone().unwrap()
        }
    };
    Some(sockets)
}

pub fn resolve_node_server(
    resource: &Resource,
    current: ScopeShared,
    socket_key: &Key,
) -> Option<NodeSocketShared> {
    let find_socket = find_plan_or_node_socket(resource, current, socket_key)?;
    let socket = match find_socket {
        FindSocket::Node(socket) => {
            if !socket.read().is_server() {
                return None;
            }
            socket.downgrade()
        }
        FindSocket::Plan(socket) => {
            let guard = socket.read();
            let inner = guard.as_server()?;
            inner.listen.clone().unwrap()
        }
    };
    Some(socket)
}

pub fn resolve_node_client(
    resource: &Resource,
    current: ScopeShared,
    socket_key: &Key,
) -> Option<Vec<NodeSocketShared>> {
    let find_socket = find_plan_or_node_socket(resource, current, socket_key)?;
    let sockets = match find_socket {
        FindSocket::Node(socket) => {
            if !socket.read().is_client() {
                return None;
            }
            vec![socket.downgrade()]
        }
        FindSocket::Plan(socket) => {
            let guard = socket.read();
            let inner = guard.as_client()?;
            inner.connect.clone().unwrap()
        }
    };
    Some(sockets)
}
