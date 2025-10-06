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
    pub plan_tab: SharedTable<PlanScope>,
    pub group_tab: SharedTable<GroupScope>,
    pub include_tab: SharedTable<IncludeCtx>,
    pub node_tab: SharedTable<NodeCtx>,
    pub pubsub_link_tab: SharedTable<PubSubLinkCtx>,
    pub service_link_tab: SharedTable<ServiceLinkCtx>,
    pub plan_pub_tab: SharedTable<PlanPubCtx>,
    pub plan_sub_tab: SharedTable<PlanSubCtx>,
    pub plan_srv_tab: SharedTable<PlanSrvCtx>,
    pub plan_cli_tab: SharedTable<PlanCliCtx>,
    pub node_pub_tab: SharedTable<NodePubCtx>,
    pub node_sub_tab: SharedTable<NodeSubCtx>,
    pub node_srv_tab: SharedTable<NodeSrvCtx>,
    pub node_cli_tab: SharedTable<NodeCliCtx>,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn program_default_creates_instance() {
        let program = Program::default();
        // Verify tables exist by checking they can iterate (even if empty)
        program
            .plan_tab
            .with_iter(|iter| assert_eq!(iter.count(), 0));
        program
            .group_tab
            .with_iter(|iter| assert_eq!(iter.count(), 0));
        program
            .include_tab
            .with_iter(|iter| assert_eq!(iter.count(), 0));
        program
            .node_tab
            .with_iter(|iter| assert_eq!(iter.count(), 0));
    }

    #[test]
    fn program_to_string_produces_yaml() {
        let program = Program::default();
        let yaml = program.to_string();
        assert!(yaml.contains("plan_tab"));
        assert!(yaml.contains("node_tab"));
        assert!(yaml.contains("group_tab"));
    }

    #[test]
    fn program_roundtrip_serialization() {
        let program = Program::default();
        let yaml = program.to_string();
        let parsed: Result<Program, _> = yaml.parse();
        assert!(parsed.is_ok());
    }

    #[test]
    fn program_from_str_parses_yaml() {
        let yaml = r#"
plan_tab:
  name: plan_tab
  table: {}
group_tab:
  name: group_tab
  table: {}
include_tab:
  name: include_tab
  table: {}
node_tab:
  name: node_tab
  table: {}
pubsub_link_tab:
  name: pubsub_link_tab
  table: {}
service_link_tab:
  name: service_link_tab
  table: {}
plan_pub_tab:
  name: plan_pub_tab
  table: {}
plan_sub_tab:
  name: plan_sub_tab
  table: {}
plan_srv_tab:
  name: plan_srv_tab
  table: {}
plan_cli_tab:
  name: plan_cli_tab
  table: {}
node_pub_tab:
  name: node_pub_tab
  table: {}
node_sub_tab:
  name: node_sub_tab
  table: {}
node_srv_tab:
  name: node_srv_tab
  table: {}
node_cli_tab:
  name: node_cli_tab
  table: {}
"#;
        let result: Result<Program, _> = yaml.parse();
        assert!(result.is_ok());
    }

    #[test]
    fn program_table_names_are_set() {
        let program = Program::default();
        // Verify table names are properly initialized
        assert!(format!("{:?}", program.plan_tab).contains("plan_tab"));
        assert!(format!("{:?}", program.node_tab).contains("node_tab"));
    }
}
