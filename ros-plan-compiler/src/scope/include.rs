use super::PlanScopeShared;
use crate::{
    error::Error,
    eval::{BoolStore, TextStore, ValueStore},
    utils::{
        find_pkg_dir,
        shared_table::{Owned, Shared},
    },
};
use indexmap::IndexMap;
use ros_plan_format::parameter::ParamName;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

pub type IncludeOwned = Owned<IncludeCtx>;
pub type IncludeShared = Shared<IncludeCtx>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IncludeCtx {
    pub location: IncludeLocation,
    pub when: Option<BoolStore>,
    pub assign_arg: IndexMap<ParamName, ValueStore>,
    pub plan: Option<PlanScopeShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IncludeLocation {
    Path(PathBuf),
    PkgFile(PkgFileLocation),
}

impl IncludeLocation {
    pub fn resolve_plan(&self, cwd: &Path) -> Result<Option<PathBuf>, Error> {
        match self {
            IncludeLocation::Path(path) => {
                let path = if path.is_relative() {
                    cwd.join(path)
                } else {
                    path.to_owned()
                };
                Ok(Some(path))
            }
            IncludeLocation::PkgFile(location) => location.resolve_plan(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PkgFileLocation {
    pub pkg: TextStore,
    pub file: TextStore,
}

impl PkgFileLocation {
    pub fn resolve_plan(&self) -> Result<Option<PathBuf>, Error> {
        let (pkg, file) = match (self.pkg.get_stored(), self.file.get_stored()) {
            (Ok(pkg), Ok(file)) => (pkg, file),
            _ => return Ok(None),
        };
        let pkg_dir = find_pkg_dir(pkg)?;
        let plan_path = pkg_dir.join("share").join(pkg).join("plan").join(file);
        Ok(Some(plan_path))
    }
}
