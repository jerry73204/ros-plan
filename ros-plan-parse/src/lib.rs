pub mod context;
pub mod error;
pub mod processor;
pub mod resource;
pub mod tree;
pub mod utils;

use crate::{
    error::Error,
    processor::{
        evaluator::Evaluator, link_resolver::LinkResolver, plan_visitor::PlanVisitor,
        socket_resolver::SocketResolver,
    },
    resource::Resource,
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

pub fn eval_resource(
    resource: &mut Resource,
    args: IndexMap<ParamName, Value>,
) -> Result<(), Error> {
    let mut evaluator = Evaluator::default();
    evaluator.eval_resource(resource, args)?;
    Ok(())
}
