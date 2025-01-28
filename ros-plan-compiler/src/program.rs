use crate::{
    context::{
        link::{PubSubLinkCtx, ServiceLinkCtx},
        node::NodeCtx,
        node_socket::{NodeCliCtx, NodePubCtx, NodeSrvCtx, NodeSubCtx},
        plan_socket::{PlanCliCtx, PlanPubCtx, PlanSrvCtx, PlanSubCtx},
    },
    error::Error,
    processor::shared_ref_initializer::SharedRefInitializer,
    scope::{GroupScope, IncludeCtx, IncludeShared, PlanScope, PlanScopeShared, ScopeShared},
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
    pub(crate) plan_tab: SharedTable<PlanScope>,
    pub(crate) group_tab: SharedTable<GroupScope>,
    pub(crate) include_tab: SharedTable<IncludeCtx>,
    pub(crate) node_tab: SharedTable<NodeCtx>,
    pub(crate) pubsub_link_tab: SharedTable<PubSubLinkCtx>,
    pub(crate) service_link_tab: SharedTable<ServiceLinkCtx>,
    pub(crate) plan_pub_tab: SharedTable<PlanPubCtx>,
    pub(crate) plan_sub_tab: SharedTable<PlanSubCtx>,
    pub(crate) plan_srv_tab: SharedTable<PlanSrvCtx>,
    pub(crate) plan_cli_tab: SharedTable<PlanCliCtx>,
    pub(crate) node_pub_tab: SharedTable<NodePubCtx>,
    pub(crate) node_sub_tab: SharedTable<NodeSubCtx>,
    pub(crate) node_srv_tab: SharedTable<NodeSrvCtx>,
    pub(crate) node_cli_tab: SharedTable<NodeCliCtx>,
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

    pub fn root_include(&self) -> IncludeShared {
        self.include_tab.get(0).unwrap()
    }

    pub fn root_scope(&self) -> PlanScopeShared {
        self.plan_tab.get(0).unwrap()
    }

    pub fn absolute_selector(&self) -> AbsoluteSelector<'_> {
        AbsoluteSelector::new(self)
    }

    pub fn selector<'a, 'b>(&'a self, scope: &'b ScopeShared) -> Selector<'a, 'b> {
        Selector::new(self, scope)
    }
}

impl Default for Program {
    fn default() -> Self {
        Self {
            plan_tab: SharedTable::new("plan_tab"),
            group_tab: SharedTable::new("group_tab"),
            include_tab: SharedTable::new("include_tab"),
            node_tab: SharedTable::new("node_tab"),
            pubsub_link_tab: SharedTable::new("pubsub_link_tab"),
            service_link_tab: SharedTable::new("service_link_tab"),
            plan_pub_tab: SharedTable::new("plan_pub_tab"),
            plan_sub_tab: SharedTable::new("plan_sub_tab"),
            plan_srv_tab: SharedTable::new("plan_srv_tab"),
            plan_cli_tab: SharedTable::new("plan_cli_tab"),
            node_pub_tab: SharedTable::new("node_pub_tab"),
            node_sub_tab: SharedTable::new("node_sub_tab"),
            node_srv_tab: SharedTable::new("node_srv_tab"),
            node_cli_tab: SharedTable::new("node_cli_tab"),
        }
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
