use std::{marker::PhantomData, path::Path};

use indexmap::IndexMap;
use ros_plan_format::{expr::Value, parameter::ParamName};

use crate::{
    error::Error,
    processor::{
        evaluator::Evaluator, link_resolver::LinkResolver, plan_visitor::PlanVisitor,
        socket_resolver::SocketResolver,
    },
    Program,
};

#[derive(Debug)]
pub struct Compiler {
    _priv: PhantomData<()>,
}

impl Compiler {
    pub fn new() -> Self {
        Self { _priv: PhantomData }
    }

    /// Compile the plan file into a program.
    pub fn compile<P>(&self, path: P) -> Result<Program, Error>
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
    pub fn eval(
        &self,
        program: &mut Program,
        args: IndexMap<ParamName, Value>,
    ) -> Result<(), Error> {
        let mut evaluator = Evaluator::default();
        evaluator.eval_resource(program, args)?;
        Ok(())
    }
}

impl Default for Compiler {
    fn default() -> Self {
        Self::new()
    }
}
