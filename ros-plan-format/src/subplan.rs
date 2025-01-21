use crate::{
    error::InvalidSubplanDeclaration, expr::ValueOrExpr, key::RelativeKeyOwned, link::LinkTable,
    node::NodeTable, parameter::ParamName,
};
use indexmap::IndexMap;
use itertools::{chain, Itertools};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedSubplanTable", into = "SerializedSubplanTable")]
pub struct SubplanTable(pub IndexMap<RelativeKeyOwned, SubplanCfg>);

impl Default for SubplanTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerializedSubplanTable {
    #[serde(default)]
    pub include: IndexMap<RelativeKeyOwned, SerializedIncludeEntry>,

    #[serde(default)]
    pub group: IndexMap<RelativeKeyOwned, GroupCfg>,
}

impl TryFrom<SerializedSubplanTable> for SubplanTable {
    type Error = InvalidSubplanDeclaration;

    fn try_from(table: SerializedSubplanTable) -> Result<Self, Self::Error> {
        macro_rules! bail {
            ($ident:expr) => {
                return Err(InvalidSubplanDeclaration::RepeatedDefinition { key: $ident });
            };
        }

        let SerializedSubplanTable { include, group } = table;
        let mut map: IndexMap<_, SubplanCfg> = IndexMap::new();

        // Check if one key is a prefix of another key.
        {
            let mut subplan_keys: Vec<_> = chain!(include.keys(), group.keys()).collect();
            subplan_keys.sort_unstable();

            for (prev, next) in subplan_keys.iter().tuple_windows() {
                if let Some(suffix) = next.as_str().strip_prefix(prev.as_str()) {
                    if suffix.starts_with('/') {
                        return Err(InvalidSubplanDeclaration::NonDisjointKeysNotAllowed {
                            short: prev.0.to_owned(),
                            long: next.0.to_owned(),
                        });
                    }
                }
            }
        }

        for (key, subplan) in include {
            let prev = map.insert(key.clone(), subplan.into());
            if prev.is_some() {
                bail!(key.0);
            }
        }
        for (key, subplan) in group {
            let prev = map.insert(key.clone(), subplan.into());
            if prev.is_some() {
                bail!(key.0);
            }
        }

        Ok(Self(map))
    }
}

impl From<SubplanTable> for SerializedSubplanTable {
    fn from(table: SubplanTable) -> Self {
        let mut include = IndexMap::new();
        let mut group = IndexMap::new();

        for (ident, subplan) in table.0 {
            match subplan {
                SubplanCfg::File(subplan) => {
                    include.insert(ident, subplan.into());
                }
                SubplanCfg::Pkg(subplan) => {
                    include.insert(ident, subplan.into());
                }
                SubplanCfg::Group(subplan) => {
                    group.insert(ident, subplan);
                }
            }
        }

        Self { include, group }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum SerializedIncludeEntry {
    File(IncludeFileCfg),
    Pkg(IncludePkgCfg),
}

impl From<SerializedIncludeEntry> for SubplanCfg {
    fn from(entry: SerializedIncludeEntry) -> Self {
        match entry {
            SerializedIncludeEntry::File(entry) => entry.into(),
            SerializedIncludeEntry::Pkg(entry) => entry.into(),
        }
    }
}

impl From<IncludePkgCfg> for SerializedIncludeEntry {
    fn from(v: IncludePkgCfg) -> Self {
        Self::Pkg(v)
    }
}

impl From<IncludeFileCfg> for SerializedIncludeEntry {
    fn from(v: IncludeFileCfg) -> Self {
        Self::File(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SubplanCfg {
    File(IncludeFileCfg),
    Pkg(IncludePkgCfg),
    Group(GroupCfg),
}

impl From<IncludeFileCfg> for SubplanCfg {
    fn from(v: IncludeFileCfg) -> Self {
        Self::File(v)
    }
}

impl From<IncludePkgCfg> for SubplanCfg {
    fn from(v: IncludePkgCfg) -> Self {
        Self::Pkg(v)
    }
}

impl From<GroupCfg> for SubplanCfg {
    fn from(v: GroupCfg) -> Self {
        Self::Group(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IncludeFileCfg {
    pub path: PathBuf,
    pub when: Option<ValueOrExpr>,
    #[serde(default)]
    pub arg: IndexMap<ParamName, ValueOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IncludePkgCfg {
    pub pkg: String,
    pub file: String,
    pub when: Option<ValueOrExpr>,
    #[serde(default)]
    pub arg: IndexMap<ParamName, ValueOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroupCfg {
    #[serde(default)]
    pub node: NodeTable,
    #[serde(default)]
    pub link: LinkTable,
    #[serde(default)]
    pub subplan: SubplanTable,
    pub when: Option<ValueOrExpr>,
}
