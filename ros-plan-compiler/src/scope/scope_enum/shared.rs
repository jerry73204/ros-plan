use super::ScopeOwned;
use crate::scope::{GroupScopeShared, PlanScopeShared};

#[derive(Debug, Clone)]
pub enum ScopeShared {
    Group(GroupScopeShared),
    Include(PlanScopeShared),
}

impl ScopeShared {
    pub fn upgrade(&self) -> Option<ScopeOwned> {
        Some(match self {
            ScopeShared::Group(shared) => shared.upgrade()?.into(),
            ScopeShared::Include(shared) => shared.upgrade()?.into(),
        })
    }

    pub fn as_group(&self) -> Option<&GroupScopeShared> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_group_mut(&mut self) -> Option<&mut GroupScopeShared> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include(&self) -> Option<&PlanScopeShared> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include_mut(&mut self) -> Option<&mut PlanScopeShared> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn try_into_group(self) -> Result<GroupScopeShared, Self> {
        if let Self::Group(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_include(self) -> Result<PlanScopeShared, Self> {
        if let Self::Include(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }
}

impl From<PlanScopeShared> for ScopeShared {
    fn from(v: PlanScopeShared) -> Self {
        Self::Include(v)
    }
}

impl From<GroupScopeShared> for ScopeShared {
    fn from(v: GroupScopeShared) -> Self {
        Self::Group(v)
    }
}
