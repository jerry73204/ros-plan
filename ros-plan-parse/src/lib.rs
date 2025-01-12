pub mod context;
pub mod error;
mod eval;
mod link_resolver;
mod plan_visitor;
pub mod resource;
mod socket_resolver;
mod tree;
pub mod uri;
mod utils;

use crate::{
    error::Error, eval::Evaluator, link_resolver::LinkResolver, plan_visitor::PlanVisitor,
    resource::Resource, socket_resolver::SocketResolver,
};
use indexmap::IndexMap;
use ros_plan_format::{expr::Value, parameter::ParamName};
use std::path::Path;

pub fn compile_plan_file<P>(path: P) -> Result<Resource, Error>
where
    P: AsRef<Path>,
{
    let path = path.as_ref();

    // Perform plan/hereplan expansion
    let mut resource = {
        let mut visitor = PlanVisitor::new();
        visitor.traverse(path)?
    };

    // Perform plan socket resolution
    {
        let mut resolver = SocketResolver::new();
        resolver.traverse(&mut resource)?;
    }

    // Perform plan socket resolution
    {
        let mut resolver = LinkResolver::new();
        resolver.traverse(&mut resource)?;
    }

    Ok(resource)
}

pub fn eval_resource(
    resource: &mut Resource,
    args: IndexMap<ParamName, Value>,
) -> Result<(), Error> {
    let mut evaluator = Evaluator::new();
    evaluator.eval_resource(resource, args)?;
    Ok(())
}
