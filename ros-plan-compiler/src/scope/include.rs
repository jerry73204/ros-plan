use super::PlanScopeShared;
use crate::{
    error::Error,
    eval::{BoolStore, TextStore, ValueStore},
    utils::shared_table::{Owned, Shared},
};
use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, parameter::ParamName};
use ros_utils::find_pkg_dir;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

pub type IncludeOwned = Owned<IncludeCtx>;
pub type IncludeShared = Shared<IncludeCtx>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IncludeCtx {
    pub key: KeyOwned,
    pub location: IncludeLocation,
    pub when: Option<BoolStore>,
    pub assign_arg: IndexMap<ParamName, ValueStore>,
    pub plan: Option<PlanScopeShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IncludeLocation {
    Path(PathLocation),
    PkgFile(PkgFileLocation),
}

impl IncludeLocation {
    pub fn resolve_absolute_path(&self) -> Result<Option<PathBuf>, Error> {
        match self {
            IncludeLocation::Path(location) => {
                let path = location.resolve_absolute_plan()?;
                Ok(Some(path))
            }
            IncludeLocation::PkgFile(location) => location.resolve_absolute_path(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathLocation {
    pub parent_dir: PathBuf,
    pub path: PathBuf,
}

impl PathLocation {
    pub fn resolve_absolute_plan(&self) -> Result<PathBuf, Error> {
        let Self { parent_dir, path } = self;
        let abs_path = if path.is_relative() {
            parent_dir.join(path)
        } else {
            path.to_path_buf()
        };
        Ok(abs_path)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PkgFileLocation {
    pub pkg: TextStore,
    pub file: TextStore,
}

impl PkgFileLocation {
    pub fn resolve_absolute_path(&self) -> Result<Option<PathBuf>, Error> {
        let (pkg, file) = match (self.pkg.get_stored(), self.file.get_stored()) {
            (Ok(pkg), Ok(file)) => (pkg, file),
            _ => return Ok(None),
        };
        let pkg_dir = find_pkg_dir(pkg)?;
        let plan_path = pkg_dir.join("share").join(pkg).join("plan").join(file);
        Ok(Some(plan_path))
    }
}
