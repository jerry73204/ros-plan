use std::{collections::VecDeque, marker::PhantomData, path::Path};

use indexmap::IndexMap;
use ros_plan_format::{expr::Value, parameter::ParamName};

use crate::{
    error::Error,
    eval::ValueStore,
    processor::{
        evaluator::Evaluator, link_resolver::LinkResolver, program_builder::ProgramBuilder,
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
    pub fn compile<P>(&self, path: P, args: IndexMap<ParamName, Value>) -> Result<Program, Error>
    where
        P: AsRef<Path>,
    {
        let path = path.as_ref();

        // Perform plan/group expansion
        let mut builder = ProgramBuilder::default();
        let mut evaluator = Evaluator::default();

        // Load the root include
        let (mut program, root_include) = builder.load_root_include(path)?;

        // Assign arguments on the root include
        root_include.with_write(|mut guard| {
            guard.assign_arg = args
                .into_iter()
                .map(|(name, value)| (name, ValueStore::new(value.into())))
                .collect();
        });

        let mut deferred_includes: VecDeque<_> = [root_include].into();

        while let Some(include) = deferred_includes.pop_front() {
            let deferred = builder.expand_include(&mut program, include.clone())?;
            deferred_includes.extend(deferred);
            evaluator.eval(&mut program, include)?;
        }

        // Perform plan socket resolution
        {
            let mut resolver = SocketResolver::default();
            resolver.resolve(&mut program)?;
        }

        // Perform plan socket resolution
        {
            let mut resolver = LinkResolver::default();
            resolver.resolve(&mut program)?;
        }

        Ok(program)
    }
}

impl Default for Compiler {
    fn default() -> Self {
        Self::new()
    }
}
