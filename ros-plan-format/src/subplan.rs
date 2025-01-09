use crate::{
    error::InvalidSubplanDeclaration, eval::ValueOrEval, key::NonEmptyRelativeKeyOwned,
    link::LinkTable, node::NodeTable, parameter::ParamName,
};
use indexmap::IndexMap;
use itertools::{chain, Itertools};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedSubplanTable", into = "SerializedSubplanTable")]
pub struct SubplanTable(pub IndexMap<NonEmptyRelativeKeyOwned, Subplan>);

impl Default for SubplanTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerializedSubplanTable {
    #[serde(default)]
    pub include: IndexMap<NonEmptyRelativeKeyOwned, SerializedIncludeEntry>,

    #[serde(default)]
    pub group: IndexMap<NonEmptyRelativeKeyOwned, HerePlan>,
}

impl TryFrom<SerializedSubplanTable> for SubplanTable {
    type Error = InvalidSubplanDeclaration;

    fn try_from(table: SerializedSubplanTable) -> Result<Self, Self::Error> {
        macro_rules! bail {
            ($ident:expr) => {
                return Err(InvalidSubplanDeclaration::RepeatedDefinition { key: $ident });
            };
        }

        let SerializedSubplanTable {
            include,
            group: here,
        } = table;
        let mut map: IndexMap<_, Subplan> = IndexMap::new();

        // Check if one key is a prefix of another key.
        {
            let mut subplan_keys: Vec<_> = chain!(include.keys(), here.keys()).collect();
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
        for (key, subplan) in here {
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
        let mut here = IndexMap::new();

        for (ident, subplan) in table.0 {
            match subplan {
                Subplan::File(subplan) => {
                    include.insert(ident, subplan.into());
                }
                Subplan::Pkg(subplan) => {
                    include.insert(ident, subplan.into());
                }
                Subplan::Here(subplan) => {
                    here.insert(ident, subplan);
                }
            }
        }

        Self {
            include,
            group: here,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum SerializedIncludeEntry {
    File(IncludeFromFile),
    Pkg(IncludeFromPkg),
}

impl From<SerializedIncludeEntry> for Subplan {
    fn from(entry: SerializedIncludeEntry) -> Self {
        match entry {
            SerializedIncludeEntry::File(entry) => entry.into(),
            SerializedIncludeEntry::Pkg(entry) => entry.into(),
        }
    }
}

impl From<IncludeFromPkg> for SerializedIncludeEntry {
    fn from(v: IncludeFromPkg) -> Self {
        Self::Pkg(v)
    }
}

impl From<IncludeFromFile> for SerializedIncludeEntry {
    fn from(v: IncludeFromFile) -> Self {
        Self::File(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Subplan {
    File(IncludeFromFile),
    Pkg(IncludeFromPkg),
    Here(HerePlan),
}

impl From<IncludeFromFile> for Subplan {
    fn from(v: IncludeFromFile) -> Self {
        Self::File(v)
    }
}

impl From<IncludeFromPkg> for Subplan {
    fn from(v: IncludeFromPkg) -> Self {
        Self::Pkg(v)
    }
}

impl From<HerePlan> for Subplan {
    fn from(v: HerePlan) -> Self {
        Self::Here(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IncludeFromFile {
    pub path: PathBuf,
    pub when: Option<ValueOrEval>,
    #[serde(default)]
    pub arg: IndexMap<ParamName, ValueOrEval>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IncludeFromPkg {
    pub pkg: String,
    pub file: String,
    pub when: Option<ValueOrEval>,
    #[serde(default)]
    pub arg: IndexMap<ParamName, ValueOrEval>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HerePlan {
    #[serde(default)]
    pub node: NodeTable,
    #[serde(default)]
    pub link: LinkTable,
    #[serde(default)]
    pub subplan: SubplanTable,
    pub when: Option<ValueOrEval>,
}
