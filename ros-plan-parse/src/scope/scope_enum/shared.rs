use super::ScopeOwned;
use crate::scope::{GroupScopeShared, PlanFileScopeShared};

#[derive(Debug, Clone)]
pub enum ScopeShared {
    Group(GroupScopeShared),
    Include(PlanFileScopeShared),
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

    pub fn as_include(&self) -> Option<&PlanFileScopeShared> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include_mut(&mut self) -> Option<&mut PlanFileScopeShared> {
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

    pub fn try_into_include(self) -> Result<PlanFileScopeShared, Self> {
        if let Self::Include(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }
}

impl From<PlanFileScopeShared> for ScopeShared {
    fn from(v: PlanFileScopeShared) -> Self {
        Self::Include(v)
    }
}

impl From<GroupScopeShared> for ScopeShared {
    fn from(v: GroupScopeShared) -> Self {
        Self::Group(v)
    }
}
