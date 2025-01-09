pub mod context;
pub mod error;
mod eval;
mod link_resolver;
mod plan_visitor;
pub mod resource;
mod socket_resolver;
mod utils;

use crate::link_resolver::LinkResolver;
use crate::socket_resolver::SocketResolver;
use crate::{error::Error, plan_visitor::PlanVisitor};
use indexmap::IndexMap;
use resource::Resource;
use ros_plan_format::{eval::Value, parameter::ParamName};
use std::path::Path;

pub fn parse_plan_file<P>(
    path: P,
    args: Option<IndexMap<ParamName, Value>>,
) -> Result<Resource, Error>
where
    P: AsRef<Path>,
{
    let path = path.as_ref();

    // Perform plan/hereplan expansion
    let mut resource = {
        let mut visitor = PlanVisitor::new();
        visitor.traverse(path, args)?
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
