use crate::{
    context::{
        link::LinkCtx,
        node::{NodeCtx, NodeOwned},
        node_socket::{NodeSocketCtx, NodeSocketOwned},
        plan_socket::PlanSocketCtx,
    },
    error::Error,
    processor::shared_ref_initializer::SharedRefInitializer,
    scope::{GroupScope, PlanScope, PlanScopeOwned, ScopeRef, ScopeRefExt, ScopeShared},
    utils::shared_table::SharedTable,
};
use ros_plan_format::key::{Key, StripKeyPrefix};
use serde::{Deserialize, Serialize};
use std::{
    fmt::{self, Display},
    path::Path,
    str::FromStr,
};

#[derive(Debug, Serialize, Deserialize)]
pub struct Program {
    pub(crate) include_tab: SharedTable<PlanScope>,
    pub(crate) group_tab: SharedTable<GroupScope>,
    pub(crate) node_tab: SharedTable<NodeCtx>,
    pub(crate) link_tab: SharedTable<LinkCtx>,
    pub(crate) plan_socket_tab: SharedTable<PlanSocketCtx>,
    pub(crate) node_socket_tab: SharedTable<NodeSocketCtx>,
}

impl Program {
    pub fn save<P>(&self, path: P) -> Result<(), Error>
    where
        P: AsRef<Path>,
    {
        let text = self.to_string();
        let path = path.as_ref();
        std::fs::write(path, text).map_err(|error| Error::WriteFileError {
            path: path.to_owned(),
            error,
        })?;
        Ok(())
    }

    pub fn load<P>(path: P) -> Result<Program, Error>
    where
        P: AsRef<Path>,
    {
        let path = path.as_ref();
        let yaml_text = std::fs::read_to_string(path).map_err(|error| Error::ReadFileError {
            path: path.to_owned(),
            error,
        })?;
        let program: Self = yaml_text.parse()?;

        // Initialize internal references.
        let mut updater = SharedRefInitializer::default();
        updater.initialize(&program)?;

        Ok(program)
    }

    pub fn root(&self) -> Option<PlanScopeOwned> {
        self.include_tab.get(0)
    }

    /// Locate a scope specified by an absolute key.
    pub fn find_scope(&self, key: &Key) -> Option<ScopeShared> {
        let suffix = match key.strip_prefix("/".parse().unwrap()) {
            StripKeyPrefix::ImproperPrefix => {
                // Case: key not starting with "/"
                return None;
            }
            StripKeyPrefix::EmptySuffix => {
                // Case: key == "/"
                return Some(self.root().unwrap().downgrade().into());
            }
            StripKeyPrefix::Suffix(suffix) => {
                // Case: key starting with "/"
                suffix
            }
        };

        // Walk down to descent child nodes
        let root = self.root().unwrap();
        let guard = root.read();
        guard.global_selector().find_subscope(suffix)
    }

    /// Locate a node specified by an absolute key.
    pub fn find_node(&self, key: &Key) -> Option<NodeOwned> {
        let (Some(scope_key), Some(node_name)) = key.split_parent() else {
            return None;
        };
        let scope = self.find_scope(scope_key)?;
        let node = {
            let owned = scope.upgrade().unwrap();
            let guard = owned.read();
            let shared = guard.node_map().get(node_name)?;
            shared.upgrade().unwrap()
        };
        Some(node)
    }

    /// Locate a socket on a node specified by an absolute key.
    pub fn find_node_socket(&self, key: &Key) -> Option<NodeSocketOwned> {
        let (Some(node_key), Some(socket_name)) = key.split_parent() else {
            return None;
        };
        let node = self.find_node(node_key)?;
        let socket = {
            let guard = node.read();
            let shared = guard.socket.get(socket_name)?;
            shared.upgrade().unwrap()
        };
        Some(socket)
    }
}

impl Display for Program {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let yaml_text = serde_yaml::to_string(self).unwrap();
        f.write_str(&yaml_text)
    }
}

impl FromStr for Program {
    type Err = Error;

    fn from_str(yaml_text: &str) -> Result<Self, Self::Err> {
        let program: Self = serde_yaml::from_str(yaml_text).unwrap();
        Ok(program)
    }
}
