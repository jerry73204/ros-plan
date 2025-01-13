use crate::{
    context::{link::LinkContext, node::NodeContext, socket::SocketContext},
    error::Error,
    processor::{
        evaluator::Evaluator, link_resolver::LinkResolver, plan_visitor::PlanVisitor,
        shared_ref_initializer::SharedRefInitializer, socket_resolver::SocketResolver,
    },
    scope::ScopeTreeRef,
    utils::shared_table::SharedTable,
};
use indexmap::IndexMap;
use ros_plan_format::{expr::Value, key::Key, parameter::ParamName};
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Serialize, Deserialize)]
pub struct Resource {
    pub(crate) root: Option<ScopeTreeRef>,
    pub(crate) node_tab: SharedTable<NodeContext>,
    pub(crate) link_tab: SharedTable<LinkContext>,
    pub(crate) socket_tab: SharedTable<SocketContext>,
}

impl Resource {
    /// Construct the resource tree from the provided file.
    pub fn from_plan_file<P>(path: P) -> Result<Resource, Error>
    where
        P: AsRef<Path>,
    {
        let path = path.as_ref();

        // Perform plan/hereplan expansion
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

    /// Locate the scope specified by an absolute key.
    pub fn resolve_absolute_key(&self, key: &Key) -> Option<ScopeTreeRef> {
        let suffix = match key.strip_prefix("/".parse().unwrap()) {
            Ok(Some(suffix)) => suffix,
            _ => return None,
        };

        // Check the key == "/" case
        let root = self.root.clone().unwrap();
        if suffix.is_empty() {
            return Some(root.clone());
        }

        // Walk down to descent child nodes
        let mut curr = root.clone();
        let mut suffix = Some(suffix.to_owned());

        while let Some(curr_suffix) = suffix {
            let (child, new_suffix) = curr.get_child(&curr_suffix)?;
            curr = child;
            suffix = new_suffix;
        }

        Some(curr)
    }
}
