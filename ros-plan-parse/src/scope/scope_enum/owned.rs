use super::{ScopeReadGuard, ScopeWriteGuard};
use crate::scope::{GroupScopeOwned, PlanScopeOwned};

#[derive(Debug)]
pub enum ScopeOwned {
    Group(GroupScopeOwned),
    Include(PlanScopeOwned),
}

impl ScopeOwned {
    pub fn read(&self) -> ScopeReadGuard<'_> {
        match self {
            ScopeOwned::Group(owned) => owned.read().into(),
            ScopeOwned::Include(owned) => owned.read().into(),
        }
    }

    pub fn write(&self) -> ScopeWriteGuard<'_> {
        match self {
            ScopeOwned::Group(owned) => owned.write().into(),
            ScopeOwned::Include(owned) => owned.write().into(),
        }
    }

    pub fn as_group(&self) -> Option<&GroupScopeOwned> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_group_mut(&mut self) -> Option<&mut GroupScopeOwned> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include(&self) -> Option<&PlanScopeOwned> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include_mut(&mut self) -> Option<&mut PlanScopeOwned> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<PlanScopeOwned> for ScopeOwned {
    fn from(v: PlanScopeOwned) -> Self {
        Self::Include(v)
    }
}

impl From<GroupScopeOwned> for ScopeOwned {
    fn from(v: GroupScopeOwned) -> Self {
        Self::Group(v)
    }
}
