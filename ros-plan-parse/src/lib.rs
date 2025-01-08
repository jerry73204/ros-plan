mod config_serializer;
mod context;
pub mod error;
mod eval;
mod link_resolver;
mod plan_visitor;
mod socket_resolver;
mod utils;

use crate::{error::Error, plan_visitor::PlanVisitor};
use config_serializer::ConfigSerializer;
use link_resolver::LinkResolver;
use ros_plan_format::{eval::Value, parameter::ParamName};
use socket_resolver::SocketResolver;
use std::{collections::HashMap, path::Path};

pub fn parse_plan_file<P>(path: P, args: Option<HashMap<ParamName, Value>>) -> Result<(), Error>
where
    P: AsRef<Path>,
{
    let path = path.as_ref();

    // Perform plan/hereplan expansion
    let mut context = {
        let mut visitor = PlanVisitor::new();
        visitor.traverse(path, args)?
    };

    // Perform plan socket resolution
    {
        let mut resolver = SocketResolver::new();
        resolver.traverse(&mut context)?;
    }

    // Perform plan socket resolution
    let context = {
        let mut resolver = LinkResolver::new();
        resolver.traverse(context)?
    };

    // dbg!(&context);

    {
        let serializer = ConfigSerializer::new();
        let plan = serializer.convert(&context);
        let toml_text = toml::to_string_pretty(&plan).unwrap();
        // println!("{toml_text}");
    }

    Ok(())
}
