use crate::{
    expr::{BoolExpr, TextOrExpr, ValueOrExpr},
    key::RelativeKeyOwned,
    link::{LinkCfg, LinkIdent},
    node::{NodeCfg, NodeIdent},
    parameter::ParamName,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IncludeCfg {
    pub when: Option<BoolExpr>,
    pub pkg: Option<TextOrExpr>,
    pub file: Option<TextOrExpr>,
    pub path: Option<PathBuf>,
    pub transparent: Option<bool>,

    #[serde(default)]
    pub arg: IndexMap<ParamName, ValueOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroupCfg {
    pub when: Option<BoolExpr>,

    #[serde(default)]
    pub node: IndexMap<NodeIdent, NodeCfg>,

    #[serde(default)]
    pub link: IndexMap<LinkIdent, LinkCfg>,

    #[serde(default)]
    pub include: IndexMap<RelativeKeyOwned, IncludeCfg>,

    #[serde(default)]
    pub group: IndexMap<RelativeKeyOwned, GroupCfg>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_include_with_transparent_true() {
        let yaml = r#"
path: subplan.yaml
transparent: true
"#;
        let result: Result<IncludeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert_eq!(cfg.transparent, Some(true));
    }

    #[test]
    fn parse_include_with_transparent_false() {
        let yaml = r#"
path: subplan.yaml
transparent: false
"#;
        let result: Result<IncludeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert_eq!(cfg.transparent, Some(false));
    }

    #[test]
    fn parse_include_without_transparent() {
        let yaml = r#"
path: subplan.yaml
"#;
        let result: Result<IncludeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert_eq!(cfg.transparent, None);
    }

    #[test]
    fn parse_include_with_all_fields() {
        let yaml = r#"
path: subplan.yaml
transparent: true
arg:
  param1: !i64 42
"#;
        let result: Result<IncludeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert_eq!(cfg.transparent, Some(true));
        assert_eq!(cfg.arg.len(), 1);
    }

    #[test]
    fn include_roundtrip_serialization() {
        let yaml = r#"path: test.yaml
transparent: true
"#;
        let cfg: IncludeCfg = serde_yaml::from_str(yaml).unwrap();
        let serialized = serde_yaml::to_string(&cfg).unwrap();
        let deserialized: IncludeCfg = serde_yaml::from_str(&serialized).unwrap();
        assert_eq!(deserialized.transparent, Some(true));
    }
}
