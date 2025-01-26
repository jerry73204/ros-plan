use crate::{
    context::{
        link::LinkContext,
        node::{NodeContext, NodeOwned},
        node_socket::{NodeSocketContext, NodeSocketOwned},
        plan_socket::PlanSocketContext,
    },
    error::Error,
    processor::{
        evaluator::Evaluator, link_resolver::LinkResolver, plan_visitor::PlanVisitor,
        shared_ref_initializer::SharedRefInitializer, socket_resolver::SocketResolver,
    },
    scope::{GroupScope, PlanScope, PlanScopeOwned, ScopeRef, ScopeRefExt, ScopeShared},
    utils::shared_table::SharedTable,
};
use indexmap::IndexMap;
use ros_plan_format::{
    expr::Value,
    key::{Key, StripKeyPrefix},
    parameter::ParamName,
};
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Serialize, Deserialize)]
pub struct Program {
    pub(crate) include_tab: SharedTable<PlanScope>,
    pub(crate) group_tab: SharedTable<GroupScope>,
    pub(crate) node_tab: SharedTable<NodeContext>,
    pub(crate) link_tab: SharedTable<LinkContext>,
    pub(crate) plan_socket_tab: SharedTable<PlanSocketContext>,
    pub(crate) node_socket_tab: SharedTable<NodeSocketContext>,
}

impl Program {
    pub fn root(&self) -> Option<PlanScopeOwned> {
        self.include_tab.get(0)
    }

    /// Construct the resource tree from the provided file.
    pub fn from_plan_file<P>(path: P) -> Result<Program, Error>
    where
        P: AsRef<Path>,
    {
        let path = path.as_ref();

        // Perform plan/group expansion
        let mut resource = {
            let mut visitor = PlanVisitor::default();
            visitor.traverse(path)?
        };

        // Perform plan socket resolution
        {
            let mut resolver = SocketResolver::default();
            resolver.traverse(&mut resource)?;
        }

        // Perform plan socket resolution
        {
            let mut resolver = LinkResolver::default();
            resolver.traverse(&mut resource)?;
        }

        Ok(resource)
    }

    /// Evaluate embedded scripts.
    pub fn eval(&mut self, args: IndexMap<ParamName, Value>) -> Result<(), Error> {
        let mut evaluator = Evaluator::default();
        evaluator.eval_resource(self, args)?;
        Ok(())
    }

    pub fn initialize_internal_references(&mut self) -> Result<(), Error> {
        let mut updater = SharedRefInitializer::default();
        updater.initialize(self)?;
        Ok(())
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
        guard.get_subscope_recursive_unbounded(suffix)
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
