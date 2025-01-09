pub mod context;
pub mod error;
mod eval;
mod link_resolver;
mod plan_visitor;
pub mod resource;
mod socket_resolver;
mod tree;
mod utils;

use crate::link_resolver::LinkResolver;
use crate::resource::Resource;
use crate::socket_resolver::SocketResolver;
use crate::{error::Error, plan_visitor::PlanVisitor};
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
