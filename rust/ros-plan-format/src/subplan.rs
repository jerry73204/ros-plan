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

    /// Namespace prefix for included nodes (only for launch files)
    pub namespace: Option<TextOrExpr>,

    /// Treat this include as a ROS 2 launch file (.launch.py/.launch.xml)
    /// When true, uses launch_loader to extract nodes
    #[serde(default)]
    pub launch: bool,

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

    #[test]
    fn parse_launch_include() {
        let yaml = r#"
path: /path/to/camera.launch.py
launch: true
namespace: /sensors
arg:
  camera_name: !str "front_camera"
  fps: !i64 30
"#;
        let result: Result<IncludeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.launch);
        assert!(cfg.namespace.is_some());
        assert_eq!(cfg.arg.len(), 2);
    }

    #[test]
    fn parse_launch_include_with_when_condition() {
        let yaml = r#"
path: /path/to/optional.launch.py
launch: true
when: $ use_optional == true $
"#;
        let result: Result<IncludeCfg, _> = serde_yaml::from_str(yaml);
        if let Err(e) = &result {
            eprintln!("Parse error: {}", e);
        }
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.launch);
        assert!(cfg.when.is_some());
    }

    #[test]
    fn parse_launch_include_default_launch_false() {
        let yaml = r#"
path: subplan.yaml
"#;
        let result: Result<IncludeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(!cfg.launch);
    }

    #[test]
    fn parse_launch_include_with_pkg() {
        let yaml = r#"
pkg: camera_package
file: launch/camera.launch.py
launch: true
"#;
        let result: Result<IncludeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.launch);
        assert!(cfg.pkg.is_some());
        assert!(cfg.file.is_some());
    }
}
