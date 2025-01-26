use crate::{
    context::{
        link::LinkCtx, node::NodeCtx, node_socket::NodeSocketCtx, plan_socket::PlanSocketCtx,
    },
    error::Error,
    processor::shared_ref_initializer::SharedRefInitializer,
    scope::{GroupScope, PlanScope, PlanScopeShared, ScopeShared},
    selector::{AbsoluteSelector, Selector},
    utils::shared_table::SharedTable,
};
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

    pub fn root(&self) -> PlanScopeShared {
        self.include_tab.get(0).unwrap().downgrade()
    }

    pub fn absolute_selector(&self) -> AbsoluteSelector<'_> {
        AbsoluteSelector::new(self)
    }

    pub fn selector<'a, 'b>(&'a self, scope: &'b ScopeShared) -> Selector<'a, 'b> {
        Selector::new(self, scope)
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
